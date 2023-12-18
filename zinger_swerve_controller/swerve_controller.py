# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List
import rclpy
from rclpy.clock import Clock, Time
from rclpy.duration import Duration as TimeDuration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_geometry_msgs import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf_transformations import quaternion_from_euler

from .control import BodyMotionCommand
from .drive_module import DriveModule
from .geometry import Point
from .profile import SingleVariableLinearProfile, SingleVariableSCurveProfile, TransientVariableProfile
from .states import DriveModuleMeasuredValues
from .steering_controller import DriveModuleDesiredValuesProfilePoint, ModuleFollowsBodySteeringController

class SwerveController(Node):
    def __init__(self):
        super().__init__("publisher_velocity_controller")
        # Declare all parameters
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("twist_topic", "cmd_vel")

        self.declare_parameter("position_controller_name", "position_controller")
        self.declare_parameter("velocity_controller_name", "velocity_controller")
        self.declare_parameter("cycle_fequency", 50)

        self.declare_parameter("steering_joints", ["joint1", "joint2"])
        self.declare_parameter("drive_joints", ["joint1", "joint2"])

        self.get_logger().info(f'Initializing swerve controller ...')

        self.last_velocity_command: Twist = None

        robot_base_link = self.get_parameter("robot_base_frame").value

        # publish the module steering angle
        position_controller_name = self.get_parameter("position_controller_name").value
        steering_angle_publish_topic = "/" + position_controller_name + "/" + "commands"
        self.drive_module_steering_angle_publisher = self.create_publisher(
            Float64MultiArray,
            steering_angle_publish_topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=10))

        self.get_logger().info(
            f'Publishing steering angle changes on topic "{steering_angle_publish_topic}"'
        )

        # publish the module drive velocity
        velocity_controller_name = self.get_parameter("velocity_controller_name").value
        velocity_publish_topic = "/" + velocity_controller_name + "/" + "commands"
        self.drive_module_velocity_publisher = self.create_publisher(
            Float64MultiArray,
            velocity_publish_topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=10))

        self.get_logger().info(
            f'Publishing drive velocity changes on topic "{velocity_publish_topic}"'
        )

        # publish odometry
        odom_topic = "/odom"
        self.odometry_publisher = self.create_publisher(
            Odometry,
            odom_topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=10))
        self.get_logger().info(
            f'Publishing odometry information on topic "{odom_topic}"'
        )

        self.send_odom_transform(robot_base_link)

        # Create the controller that will determine the correct drive commands for the different drive modules
        # Create the controller before we subscribe to state changes so that the first change that comes in gets
        # registered
        self.get_logger().info(f'Storing drive module information...')
        self.drive_modules = self.get_drive_modules()
        self.controller = ModuleFollowsBodySteeringController(self.drive_modules, self.get_motion_profile, self.write_log)

        # initialize the time tracking variables after we get the controller up and running
        # so that we can initialize the controller at the same time.
        self.store_time_and_update_controller_time()
        self.last_control_update_send_at = self.last_recorded_time
        self.last_velocity_command_received_at = self.last_recorded_time

        # Create the timer that is used to ensure that we publish movement data regularly
        self.cycle_time_in_hertz = self.get_parameter("cycle_fequency").value
        self.get_logger().info(
            f'Publishing changes at fequency: "{self.cycle_time_in_hertz}" Hz'
        )

        self.timer = self.create_timer(
            1.0 / self.cycle_time_in_hertz,
            self.timer_callback,
            callback_group=None,
            clock=self.get_clock())
        self.i = 0

        # Listen for state changes in the drive modules
        joint_state_topic = "joint_states"
        self.state_change_subscription = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_states_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=10)
        )

        self.get_logger().info(
            f'Listening for drive module state changes on "{joint_state_topic}"'
        )

        # Initialize the drive modules
        self.last_drive_module_state = self.initialize_drive_module_states(self.drive_modules)

        # Finally listen to the cmd_vel topic for movement commands. We could have a message incoming
        # at any point after we register so we set this subscription up last.
        twist_topic = self.get_parameter("twist_topic").value
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            twist_topic,
            self.cmd_vel_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=10))
        self.get_logger().info(
            f'Listening for movement commands on topic "{twist_topic}"'
        )

    def cmd_vel_callback(self, msg: Twist):
        if msg == None:
            return

        # If this twist message is the same as last time, then we don't need to do anything
        if self.last_velocity_command is not None:
            if msg.linear.x == self.last_velocity_command.linear.x and \
                msg.linear.y == self.last_velocity_command.linear.y and \
                msg.angular.z == self.last_velocity_command.angular.z:

                # The last command was the same as the current command. So just ignore it and move on.
                self.get_logger().info(
                    f'Received a Twist message that is the same as the last message. Taking no action. Message was: "{msg}"'
                )

                return

        self.get_logger().info(
            f'Received a Twist message that is different from the last command. Processing message: "{msg}"'
        )

        # When we get a stream of command it is possible that each command is slightly different (looking at you ROS2 nav)
        # This means we reset the starting time of the change profile each time, which starts the process all over
        # Because we don't take the current steering velocity / drive acceleration into account we assume that we
        # start from rest. That is wrong. We should be starting from a place where we have the current
        # steering velocity / drive acceleration.

        self.store_time_and_update_controller_time()
        self.controller.on_desired_state_update(
            BodyMotionCommand(
                1.0, # THIS SHOULD REALLY BE CALCULATED SOME HOW
                msg.linear.x,
                msg.linear.y,
                msg.angular.z
            )
        )

        self.last_velocity_command = msg
        self.last_velocity_command_received_at = self.last_recorded_time

    def get_drive_modules(self) -> List[DriveModule]:
        # Get the drive module information from the URDF and turn it into a list of drive modules.
        #
        # For now we don't read the URDF and just hard-code the drive modules
        robot_length = 0.35
        robot_width = 0.30

        steering_radius = 0.05

        wheel_radius = 0.04
        wheel_width = 0.05

        # store the steering joints
        steering_joint_names = self.get_parameter("steering_joints").value
        steering_joints = []
        for name in steering_joint_names:
            steering_joints.append(name)
            self.get_logger().info(
                f'Discovered steering joint: "{name}"'
            )

        # store the drive joints
        drive_joint_names = self.get_parameter("drive_joints").value
        drive_joints = []
        for name in drive_joint_names:
            drive_joints.append(name)
            self.get_logger().info(
                f'Discovered drive joint: "{name}"'
            )

        drive_modules: List[DriveModule] = []
        drive_module_name = "left_front"
        left_front = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            steering_axis_xy_position=Point(0.5 * (robot_length - 2 * steering_radius), 0.5 * (robot_width - steering_radius), 0.0),
            wheel_radius=wheel_radius,
            wheel_width=wheel_width,
            steering_motor_maximum_velocity=10.0,
            steering_motor_minimum_acceleration=0.1,
            steering_motor_maximum_acceleration=1.0,
            drive_motor_maximum_velocity=10.0,
            drive_motor_minimum_acceleration=0.1,
            drive_motor_maximum_acceleration=1.0
        )
        drive_modules.append(left_front)

        self.get_logger().info(
            f'Configured drive module: "{left_front.name}" ' +
            f'with steering link: "{left_front.steering_link_name}" ' +
            f'and drive link: "{left_front.driving_link_name}" ' +
            f'and position: ["{left_front.steering_axis_xy_position.x}", "{left_front.steering_axis_xy_position.y}"]'
        )

        drive_module_name = "left_rear"
        left_rear = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            steering_axis_xy_position=Point(-0.5 * (robot_length - 2 * steering_radius), 0.5 * (robot_width - steering_radius), 0.0),
            wheel_radius=wheel_radius,
            wheel_width=wheel_width,
            steering_motor_maximum_velocity=10.0,
            steering_motor_minimum_acceleration=0.1,
            steering_motor_maximum_acceleration=1.0,
            drive_motor_maximum_velocity=10.0,
            drive_motor_minimum_acceleration=0.1,
            drive_motor_maximum_acceleration=1.0
        )
        drive_modules.append(left_rear)

        self.get_logger().info(
            f'Configured drive module: "{left_rear.name}" ' +
            f'with steering link: "{left_rear.steering_link_name}" ' +
            f'and drive link: "{left_rear.driving_link_name}" ' +
            f'and position: ["{left_rear.steering_axis_xy_position.x}", "{left_rear.steering_axis_xy_position.y}"]'
        )

        drive_module_name = "right_rear"
        right_rear = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            steering_axis_xy_position=Point(-0.5 * (robot_length - 2 * steering_radius), -0.5 * (robot_width - steering_radius), 0.0),
            wheel_radius=wheel_radius,
            wheel_width=wheel_width,
            steering_motor_maximum_velocity=10.0,
            steering_motor_minimum_acceleration=0.1,
            steering_motor_maximum_acceleration=1.0,
            drive_motor_maximum_velocity=10.0,
            drive_motor_minimum_acceleration=0.1,
            drive_motor_maximum_acceleration=1.0
        )
        drive_modules.append(right_rear)

        self.get_logger().info(
            f'Configured drive module: "{right_rear.name}" ' +
            f'with steering link: "{right_rear.steering_link_name}" ' +
            f'and drive link: "{right_rear.driving_link_name}" ' +
            f'and position: ["{right_rear.steering_axis_xy_position.x}", "{right_rear.steering_axis_xy_position.y}"]'
        )

        drive_module_name = "right_front"
        right_front = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            steering_axis_xy_position=Point(0.5 * (robot_length - 2 * steering_radius), -0.5 * (robot_width - steering_radius), 0.0),
            wheel_radius=wheel_radius,
            wheel_width=wheel_width,
            steering_motor_maximum_velocity=10.0,
            steering_motor_minimum_acceleration=0.1,
            steering_motor_maximum_acceleration=1.0,
            drive_motor_maximum_velocity=10.0,
            drive_motor_minimum_acceleration=0.1,
            drive_motor_maximum_acceleration=1.0
        )
        drive_modules.append(right_front)

        self.get_logger().info(
            f'Configured drive module: "{right_front.name}" ' +
            f'with steering link: "{right_front.steering_link_name}" ' +
            f'and drive link: "{right_front.driving_link_name}" ' +
            f'and position: ["{right_front.steering_axis_xy_position.x}", "{right_front.steering_axis_xy_position.y}"]'
        )

        return drive_modules

    def get_motion_profile(self, start: float, end: float) -> TransientVariableProfile:
        # return SingleVariableSCurveProfile(start, end)

        return SingleVariableLinearProfile(start, end)

    def initialize_drive_module_states(self, drive_modules: List[DriveModule]) -> List[DriveModuleMeasuredValues]:
        measured_drive_states: List[DriveModuleMeasuredValues] = []
        for drive_module in self.drive_modules:

            value = DriveModuleMeasuredValues(
                drive_module.name,
                drive_module.steering_axis_xy_position.x,
                drive_module.steering_axis_xy_position.y,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            )
            measured_drive_states.append(value)

            self.get_logger().info(
                f'Initializing drive module state for module: "{drive_module.name}"'
            )

        self.store_time_and_update_controller_time()
        self.controller.on_state_update(measured_drive_states)

        return measured_drive_states

    def joint_states_callback(self, msg: JointState):
        if msg == None:
            return

        # self.get_logger().debug(
        #     f'Received a JointState message: "{msg}"'
        # )

        # It would be better if we stored this message and processed it during our own timer loop. That way
        # we wouldn't be blocking the callback.

        joint_names: List[str] = msg.name
        joint_positions: List[float] = [pos for pos in msg.position]
        joint_velocities: List[float] = [vel for vel in msg.velocity]

        measured_drive_states: List[DriveModuleMeasuredValues] = []
        for index, drive_module in enumerate(self.drive_modules):
            if drive_module.steering_link_name in joint_names and drive_module.driving_link_name in joint_names:
                steering_values_index = joint_names.index(drive_module.steering_link_name)
                drive_values_index = joint_names.index(drive_module.driving_link_name)

                value = DriveModuleMeasuredValues(
                    drive_module.name,
                    drive_module.steering_axis_xy_position.x,
                    drive_module.steering_axis_xy_position.y,
                    joint_positions[steering_values_index],
                    joint_velocities[steering_values_index],
                    0.0,
                    0.0,
                    joint_velocities[drive_values_index] * drive_module.wheel_radius,
                    0.0,
                    0.0
                )
                measured_drive_states.append(value)

                # self.get_logger().info(
                #     f'Updating joint states for: "{drive_module.name}" with: ' +
                #     f'[ steering angle: "{value.orientation_in_body_coordinates.z}", ' +
                #     f' steering velocity: "{value.orientation_velocity_in_body_coordinates.z}",' +
                #     f' velocity: "{value.drive_velocity_in_module_coordinates.x}" ] '
                # )
            else:
                # grab the previous state and just assume that's the one
                value = self.last_drive_module_state[index]
                measured_drive_states.append(value)

                # self.get_logger().debug(
                #     f'Updating joint states for: "{drive_module.name}" with: ' +
                #     f'[ steering angle: "{value.orientation_in_body_coordinates.z}", ' +
                #     f' steering velocity: "{value.orientation_velocity_in_body_coordinates.z}",' +
                #     f' velocity: "{value.drive_velocity_in_module_coordinates.x}" ] '
                # )

        # Ideally we would get the time from the message. And then check if we have gotten a more
        # recent message
        self.store_time_and_update_controller_time()
        self.controller.on_state_update(measured_drive_states)
        self.last_drive_module_state = measured_drive_states

    def publish_odometry(self):
        body_state = self.controller.body_state_at_current_time()

        msg = Odometry()
        msg.header.stamp = self.last_recorded_time.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = body_state.position_in_world_coordinates.x
        msg.pose.pose.position.y = body_state.position_in_world_coordinates.y
        msg.pose.pose.position.z = body_state.position_in_world_coordinates.z

        quat = quaternion_from_euler(0.0, 0.0, body_state.orientation_in_world_coordinates.z)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        msg.twist.twist.linear.x = body_state.motion_in_body_coordinates.linear_velocity.x
        msg.twist.twist.linear.y = body_state.motion_in_body_coordinates.linear_velocity.y
        msg.twist.twist.linear.z = body_state.motion_in_body_coordinates.linear_velocity.z

        msg.twist.twist.angular.x = body_state.motion_in_body_coordinates.angular_velocity.x
        msg.twist.twist.angular.y = body_state.motion_in_body_coordinates.angular_velocity.y
        msg.twist.twist.angular.z = body_state.motion_in_body_coordinates.angular_velocity.z

        # For now we ignore the covariances

        # self.get_logger().info(
        #     'Publishing odometry message {}'.format(msg)
        # )

        self.odometry_publisher.publish(msg)

    def send_odom_transform(self, robot_base_link: str):
        tf_static_broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = robot_base_link

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, 0.0)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        tf_static_broadcaster.sendTransform(transform)

    def store_time_and_update_controller_time(self):
        time: Time = self.get_clock().now()
        seconds = time.nanoseconds * 1e-9
        self.controller.on_tick(seconds)
        self.last_recorded_time = time

    def timer_callback(self):
        self.store_time_and_update_controller_time()

        # always send out the odometry information
        self.publish_odometry()

        # Check if we actually have a movement profile to send
        current_time = self.get_clock().now()
        trajectory_running_duration: TimeDuration = current_time - self.last_velocity_command_received_at
        # self.get_logger().debug(
        #     'Current trajectory duration {} s. Based on current time {} and sequence start time {}'.format(
        #         trajectory_running_duration,
        #         current_time,
        #         self.last_velocity_command_received_at
        #     )
        # )

        running_duration_as_float: float = trajectory_running_duration.nanoseconds * 1e-9
        # self.get_logger().debug(
        #     'Current trajectory duration {} s'.format(running_duration_as_float)
        # )

        if running_duration_as_float > self.controller.min_time_for_profile:
            # self.get_logger().debug(
            #     'Trajectory completed waiting for next command.'
            # )
            return

        next_time_step = current_time.nanoseconds * 1e-9 + 1.0 / self.cycle_time_in_hertz
        # self.get_logger().debug(
        #     'Calculating next step in profile at time {} s'.format(next_time_step)
        # )

        drive_module_states = self.controller.drive_module_state_at_future_time(next_time_step)

        # Only publish movement commands if there is a trajectory
        if len(drive_module_states) == 0:
            return

        position_msg = Float64MultiArray()
        steering_angle_values = [a.steering_angle_in_radians for a in drive_module_states]
        position_msg.data = steering_angle_values

        # Note that the controller gives the velocity in meters per second, i.e. the velocity of the wheel at the
        # contact point with the ground. But ROS wants to know the rotational velocity of the wheel
        drive_velocity_values = []
        for a in drive_module_states:
            linear_velocity = a.drive_velocity_in_meters_per_second
            wheel_radius = next((x.wheel_radius for x in self.drive_modules if x.name == a.name), 1.0)
            drive_velocity_values.append(linear_velocity / wheel_radius)

        velocity_msg = Float64MultiArray()
        velocity_msg.data = drive_velocity_values

        # Publish the next steering angle and the next velocity sets. Note that
        # The velocity is published (very) shortly after the position data, which means
        # that the velocity could lag in very tight update loops.
        #self.get_logger().info(f'Publishing steering angle data: "{position_msg}"')
        self.drive_module_steering_angle_publisher.publish(position_msg)

        #self.get_logger().info(f'Publishing velocity angle data: "{velocity_msg}"')
        self.drive_module_velocity_publisher.publish(velocity_msg)

        self.last_control_update_send_at = self.last_recorded_time

    def write_log(self, text: str):
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)

    pub = SwerveController()

    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
