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