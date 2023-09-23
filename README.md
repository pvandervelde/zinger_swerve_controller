# zinger_swerve_controller

Provides the swerve controller code for the zinger robot.

## Dependencies

The configurations in this repository assume you have the following prerequisites installed on the
device on which you want to run this code. That device might be an Ubuntu machine or a physical
robot using Raspberry Pi OS.

1. [ROS humble](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html) with the
   `robot_state_publisher`, the `joint_state_broadcaster` and the
   [ros_control](https://control.ros.org/master/index.html) packages.
1. A working [ROS workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

Also the following packages should be present:

1. [zinger_description](https://github.com/pvandervelde/zinger_description) - Contains the geometric
  description of the Zinger robot for ROS to work with.

## Contents

This repository contains different folders for the different parts of the robot description.

* The config files that provide the configurations for the ROS control actuators and the test publishers
  * [config/swerve.yaml](config/swerve.yaml) defines the settings for the swerve controller.
* The launch directory contains the launch files
  * [launch/swerve_controller.launch.py](launch/swerve_controller.launch.py) - Launches the controller node.
* The source code for the swerve controller
  * [zinger_swerve_controller/control_model.py](zinger_swerve_controller/control_model.py) - Defines the inverse and forward
    kinematics.
  * [zinger_swerve_controller/control_profile.py](zinger_swerve_controller/control_profile.py) - Defines the body and module
    motion profiles.
  * [zinger_swerve_controller/control.py](zinger_swerve_controller/control.py) - Defines the different motion control commands
    that can be specified.
  * [zinger_swerve_controller/drive_module.py](zinger_swerve_controller/drive_module.py) - Defines the properties for a
    drive module.
  * [zinger_swerve_controller/errors.py](zinger_swerve_controller/errors.py) - Defines custom errors.
  * [zinger_swerve_controller/geometry.py](zinger_swerve_controller/geometry.py) - Defines standard geometry elements.
  * [zinger_swerve_controller/profile.py](zinger_swerve_controller/profile.py) - Defines the motion control
    profile that describes how the steering angle and the drive velocity change over time when they are changed from an
    initial value to a target value. The only current implementation is the s-curve motion profile.
  * [zinger_swerve_controller/states.py](zinger_swerve_controller/states.py) - Defines the data structures used to contain
    information about the current motion states.
  * [zinger_swerve_controller/steering_controller.py](zinger_swerve_controller/steering_controller.py) - Responsible
    for calculating the steering angles and drive velocities of the modules based on the initial state and the
    desired final state.
  * [zinger_swerve_controller/swerve_controller.py](zinger_swerve_controller/swerve_controller.py) - The
    controller that sends the control commands to the different joints in the drive modules. Additionally sends
    the odometry messages.

## Kinematics

### Models

The [model](zinger_swerve_controller/control_model.py) describes the inverse and forward kinematics. There are many different
algorithms available in the literature. At the moment the following algorithms are implemented:

* A [simple kinematics model](https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383/5)

In the future the aim is to also implement a 3D force based model devised by
[Neal Seegmiller](https://scholar.google.co.nz/citations?hl=en&user=H10kxZgAAAAJ&view_op=list_works&sortby=pubdate) and
described in the following papers:

* [Dynamic Model Formulation and Calibration for Wheeled Mobile Robots - 2014](https://scholar.google.co.nz/citations?view_op=view_citation&hl=en&user=H10kxZgAAAAJ&sortby=pubdate&citation_for_view=H10kxZgAAAAJ:ufrVoPGSRksC)
* [Enhanced 3D kinematic modeling of wheeled mobile robots - 2014](https://scholar.google.co.nz/citations?view_op=view_citation&hl=en&user=H10kxZgAAAAJ&sortby=pubdate&citation_for_view=H10kxZgAAAAJ:YsMSGLbcyi4C)
* [Recursive kinematic propagation for wheeled mobile robots - 2015](https://scholar.google.co.nz/citations?view_op=view_citation&hl=en&user=H10kxZgAAAAJ&sortby=pubdate&citation_for_view=H10kxZgAAAAJ:Y0pCki6q_DkC)
* [High-Fidelity Yet Fast Dynamic Models of Wheeled Mobile Robots - 2016](https://scholar.google.co.nz/citations?view_op=view_citation&hl=en&user=H10kxZgAAAAJ&sortby=pubdate&citation_for_view=H10kxZgAAAAJ:Y0pCki6q_DkC)

#### Simple kinematics model

The simple kinematic model is derived from the 2D geometric relations between the robot rotational centre and the position,
angle and velocity of the drive modules. This model makes the following assumptions

* The drive modules are connected to the robot body in a fixed location and at a fixed angle.
* There is no suspension in the drive modules.
* The robot is moving on a flat, horizontal surface.
* The wheel steering axis for a drive module goes through the centre of the wheel in a vertical direction, so the wheel
  contact point is always inline with the steering axis.

The following image shows these relationships between the robot body degrees of freedom and the drive module degrees of
freedom.

<figure
    style="float:left"
    width="560"
    height="315">
<img alt="Kinematics diagram for the multi-wheel steering controller" src="doc/kinematics.png" />
<figcaption>Kinematics diagram for the multi-wheel steering controller</figcaption>
</figure>

In this image the variables are as follows:

* `V` - The linear velocity vector for the robot body, consisting of `V_x`, the velocity in the x direction, and `V_y`,
  the velocity in the y direction.
* `W` - The rotational velocity for the robot body. Due to the 2D nature of the model this variable is a scalar not a
  vector. The rotational velocity is taken as positive going counter clock-wise.
* `r_i` - The position vector of the i-th drive module relative to the robot rotational centre.
* `alpha_i` - The angle in radians of the i-th drive module as measured in the coordinate system for that drive module.
  The angle is measured as positive going counter-clock wise from the x-axis of the drive module coordinate system. Note
  that the `alpha_i` angle in the image signifies a negative angle.
* `v_i` - The velocity of the contact patch between the ground and the wheel of the i-th drive module, measured in meters
  per second. It should be noted that this is a scalar value, not a vector.

For these variables it is important to note that all coordinate system related variables are measured in the robot
coordinate system unless otherwise specified.

Note that the rotational centre for the robot is generally either the middle or the centre of gravity, however this does
is not required for this model to work. The rotational centre can be placed anywhere. It could theoretically be moved
around over the course of time, however the current implementation does not allow for changes to the rotational centre.

By using the information displayed in the image we can derive the following equations for the relation between the
velocity of the robot body and the velocity and angle of each drive module.

    v_i = v + W x r_i

    alpha = acos (v_i_x / |v_i|)
          = asin (v_i_y / |v_i|)

To make both the forward and inverse kinematics calculations possible it is better to put these equations in a matrix
equation as follows. The state of the drive modules can be found with the following equation:

    V_i = |A| * V

where

* `V` - The state vector for the robot body = `[v_x, v_y, w]^T`
* `|A|` - The state matrix that translates the body state to the drive module state
* `V_i` - The state vector for the drive modules = `[v_1_x, v_1_y, v_2_x, v_2_y, ... , v_n_x, v_n_y]^T`

The state matrix is an `[2 * n ; 3]` matrix where `n` is the number of drive modules. For the current example this is:

    [
        1.0   0.0   -module_1.y
        0.0   1.0   module_1.x
        1.0   0.0   -module_2.y
        0.0   1.0   module_2.x
        1.0   0.0   -module_3.y
        0.0   1.0   module_3.x
        1.0   0.0   -module_4.y
        0.0   1.0   module_4.x
    ]

In order to determine the robot body motion from the drive module state we can invert the equation

    V = |A|^-1 * V_i

It should be noted that this will be an overdetermined system, there are `n * 2` values available and three variables
(`v_x, v_y, w`). Thus the inverse of `|A|` needs to be the [pseudo-inverse](https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse)
of `|A|` which provides a least-squares fit of the values to the available variables.

### Controllers

The [controller](zinger_swerve_controller/multi_wheel_steering_controller.py) is responsible for planning the profile that each
drive module should follow to move the robot from the current movement state to the desired movement state. Currently
the following controllers are implemented:

* A controller that controls the robot through the drive modules and assumes linear behaviour for the drive modules. In
  other words the controller will plan a linear profile from the current state to the desired state.

In the future the aim is to implement additional controllers that use control profiles which are more realistic.

## Usage

The `zinger_swerve_controller` package can be launched by using one of the two following command lines. The first
command line is for use when running the robot in the Gazebo simulator

    ros2 launch zinger_swerve_controller swerve_controller.launch.py use_sim_time:=true

When using the controller on a physical robot use

    ros2 launch zinger_swerve_controller swerve_controller.launch.py
