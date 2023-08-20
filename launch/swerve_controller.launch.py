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

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config = PathJoinSubstitution(
        [
            FindPackageShare("zinger_swerve_controller"),
            "config",
            "swerve.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="zinger_swerve_controller",
                executable="swerve_controller",
                name="swerve_controller",
                parameters=[config],
                output="both",
            )
        ]
    )
