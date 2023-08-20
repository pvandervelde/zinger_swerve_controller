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

class Point(object):

    def __init__(self, x_in_meters: float, y_in_meters: float, z_in_meters: float):
        self.x = x_in_meters
        self.y = y_in_meters
        self.z = z_in_meters

class Orientation(object):

    def __init__(self, x_orientation_in_radians: float, y_orientation_in_radians: float, z_orienation_in_radians: float):
        self.x = x_orientation_in_radians
        self.y = y_orientation_in_radians
        self.z = z_orienation_in_radians

class Vector3(object):

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z