import numpy as np
from robot_math.transformations import loc2glob_pose, glob2loc_pose

def ticks_to_movement_vector(left, right, wheelbase, wheel_perimeter, ticks_per_rotation):
    left_dist  = left/ticks_per_rotation*wheel_perimeter
    right_dist = right/ticks_per_rotation*wheel_perimeter

    trans_dist  = (left_dist+right_dist)/2
    angle       = (right_dist-left_dist)/wheelbase

    if angle == 0:
        return np.array([trans_dist, 0, 0])

    radius = trans_dist/angle

    x = radius*np.sin(angle)
    y = radius*(1-np.cos(angle))
    return np.array([x,y,angle])

class DeadReckoner():
    def __init__(self, wheelbase, wheel_perimeter, ticks_per_rotation, left_port, right_port):
        self.wheelbase              = wheelbase
        self.wheel_perimeter        = wheel_perimeter
        self.ticks_per_rotation     = ticks_per_rotation

        self.position = np.zeros(3)

        self.previous_encoders = None

        self.left_port = left_port
        self.right_port = right_port


    def update_position(self, movement_vec):
        self.position = loc2glob_pose(movement_vec, self.position)




