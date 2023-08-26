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
from rclpy.node import Node
from tf2_geometry_msgs import TransformStamped

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf_transformations import quaternion_from_euler

from .control import BodyMotionCommand
from .drive_module import DriveModule
from .geometry import Point
from .profile import SingleVariableSCurveProfile, TransientVariableProfile
from .states import DriveModuleMeasuredValues
from .steering_controller import DriveModuleDesiredValuesProfilePoint, ModuleFollowsBodySteeringController

class SwerveController(Node):
    def __init__(self):
        super().__init__("publisher_velocity_controller")
        # Declare all parameters
        self.declare_parameter("twist_topic", "cmd_vel")

        self.declare_parameter("position_controller_name", "position_controller")
        self.declare_parameter("velocity_controller_name", "velocity_controller")
        self.declare_parameter("cycle_fequency", 50)

        self.declare_parameter("steering_joints", ["joint1", "joint2"])
        self.declare_parameter("drive_joints", ["joint1", "joint2"])

        # publish the module steering angle
        position_controller_name = self.get_parameter("position_controller_name").value
        steering_angle_publish_topic = "/" + position_controller_name + "/" + "commands"
        self.drive_module_steering_angle_publisher = self.create_publisher(Float64MultiArray, steering_angle_publish_topic, 1)

        self.get_logger().info(
            f'Publishing steering angle changes on topic "{steering_angle_publish_topic}"'
        )

        # publish the module drive velocity
        velocity_controller_name = self.get_parameter("velocity_controller_name").value
        velocity_publish_topic = "/" + velocity_controller_name + "/" + "commands"
        self.drive_module_velocity_publisher = self.create_publisher(Float64MultiArray, velocity_publish_topic, 1)

        self.get_logger().info(
            f'Publishing drive velocity changes on topic "{velocity_publish_topic}"'
        )

        # publish odometry
        odom_topic = "/odom"
        self.odometry_publisher = self.create_publisher(Odometry, odom_topic, 1)

        # Create the controller that will determine the correct drive commands for the different drive modules
        # Create the controller before we subscribe to state changes so that the first change that comes in gets
        # registered
        self.drive_modules = self.get_drive_modules()
        self.controller = ModuleFollowsBodySteeringController(self.drive_modules, self.get_scurve_profile)

        # Create the timer that is used to ensure that we publish movement data regularly
        cycle_time_in_hertz = self.get_parameter("cycle_fequency").value
        self.timer = self.create_timer(1.0 / cycle_time_in_hertz, self.timer_callback)
        self.i = 0

        # Listen for state changes in the drive modules
        self.state_change_subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_states_callback,
            10
        )

        # Initialize the drive modules
        self.last_drive_module_state = self.initialize_drive_module_states()

        # Finally listen to the cmd_vel topic for movement commands. We could have a message incoming
        # at any point after we register so we set this subscription up last.
        twist_topic = self.get_parameter("twist_topic").value
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            twist_topic,
            self.cmd_vel_callback,
            10)
        self.get_logger().info(
            f'Listening for movement commands on topic "{twist_topic}"'
        )

    def cmd_vel_callback(self, msg: Twist):
        if msg == None:
            return

        self.update_controller_time()
        self.controller.on_desired_state_update(
            BodyMotionCommand(
                2.0, # THIS SHOULD REALLY BE CALCULATED SOME HOW
                msg.linear.x,
                msg.linear.y,
                msg.angular.z
            )
        )

    def get_drive_modules(self) -> List[DriveModule]:
        # Get the drive module information from the URDF and turn it into a list of drive modules.
        #
        # For now we don't read the URDF and just hard-code the drive modules
        robot_length = 0.35
        robot_width = 0.32

        steering_radius = 0.05

        wheel_radius = 0.04
        wheel_width = 0.05

        # store the steering joints
        steering_joint_names = self.get_parameter("steering_joints")
        steering_joints = []
        for name in steering_joint_names:
            steering_joints.append(name)

        # store the drive joints
        drive_joint_names = self.get_parameter("drive_joints")
        drive_joints = []
        for name in drive_joint_names:
            drive_joints.append(name)

        drive_modules: List[DriveModule] = []
        drive_module_name = "left_front"
        left_front = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            steering_axis_xy_position=Point(0.5 * (robot_length - 2 * wheel_radius), 0.5 * (robot_width - steering_radius), 0.0),
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

        drive_module_name = "left_rear"
        left_rear = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            steering_axis_xy_position=Point(-0.5 * (robot_length - 2 * wheel_radius), 0.5 * (robot_width - steering_radius), 0.0),
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

        drive_module_name = "right_rear"
        right_rear = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            steering_axis_xy_position=Point(-0.5 * (robot_length - 2 * wheel_radius), -0.5 * (robot_width - steering_radius), 0.0),
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

        drive_module_name = "right_front"
        right_front = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            steering_axis_xy_position=Point(0.5 * (robot_length - 2 * wheel_radius), -0.5 * (robot_width - steering_radius), 0.0),
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

        return drive_modules

    def get_scurve_profile(self, start: float, end: float) -> TransientVariableProfile:
        return SingleVariableSCurveProfile(start, end)

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

        self.update_controller_time()
        self.controller.on_state_update(measured_drive_states)

        return measured_drive_states

    def joint_states_callback(self, msg: JointState):
        if msg == None:
            return

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
                    joint_velocities[drive_values_index],
                    0.0,
                    0.0
                )
                measured_drive_states.append(value)
            else:
                # grab the previous state and just assume that's the one
                value = self.last_drive_module_state[index]
                measured_drive_states.append(value)

        # Ideally we would get the time from the message. And then check if we have gotten a more
        # recent message
        self.update_controller_time()
        self.controller.on_state_update(measured_drive_states)
        self.last_drive_module_state = measured_drive_states

    def publish_odometry(self):
        body_state = self.controller.body_state_at_current_time()

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
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

        self.odometry_publisher.publish(msg)

    def timer_callback(self):
        self.update_controller_time()

        # send out Odom
        self.publish_odometry()

        # Technically we only need to send updates if:
        # - The desired end-state has changed
        # - The current state doesn't match the trajectory
        time: Time = self.get_clock().now()
        points: List[DriveModuleDesiredValuesProfilePoint] = self.controller.drive_module_profile_points_from_now_till_end(time.nanoseconds * 1e-9) # THIS NEEDS TO BE SIM TIME IF RUNNING IN GAZEBO

        steering_angle_points: List[JointTrajectoryPoint] = []
        drive_velocity_points: List[JointTrajectoryPoint] = []
        for desired_value in points:

            steering_angle = JointTrajectoryPoint()
            steering_angle.positions = [a.steering_angle_in_radians for a in desired_value.drive_module_states]
            steering_angle.time_from_start = Duration(sec=desired_value.time)
            steering_angle_points.append(steering_angle)

            drive_velocity = JointTrajectoryPoint()
            drive_velocity.velocities = [a.drive_velocity_in_meters_per_second for a in desired_value.drive_module_states]
            drive_velocity.time_from_start = Duration(sec=desired_value.time)
            drive_velocity_points.append(drive_velocity)

        position_msg = JointTrajectory()
        position_msg.joint_names = (x.steering_link_name for x in self.drive_modules) # we can probably optimze this away, at some point
        position_msg.points.extend(steering_angle_points)

        velocity_msg = JointTrajectory()
        velocity_msg.joint_names = (x.driving_link_name for x in self.drive_modules)
        velocity_msg.points.extend(drive_velocity_points)

        # Publish the next steering angle and the next velocity sets. Note that
        # The velocity is published (very) shortly after the position data, which means
        # that the velocity could lag in very tight update loops.
        self.get_logger().info(f'Publishing steering angle data: "{position_msg}"')
        self.drive_module_steering_angle_publisher.publish(position_msg)

        self.get_logger().info(f'Publishing velocity angle data: "{velocity_msg}"')
        self.drive_module_velocity_publisher.publish(velocity_msg)

    def update_controller_time(self):
        time: Time = self.get_clock().now()
        seconds = time.nanoseconds * 1e-9
        self.controller.on_tick(seconds)

def main(args=None):
    rclpy.init(args=args)

    pub = SwerveController()

    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
