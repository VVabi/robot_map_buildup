import numpy as np

def loc2glob_pose(local_pose, global_center):
    c, s = np.cos(global_center[2]), np.sin(global_center[2])
    rot_mat = np.array(((c, -s), (s, c)))

    rotated_point = np.dot(rot_mat, local_pose[:2])

    new_angle  = local_pose[2]+global_center[2]

    while  new_angle > np.pi:
        new_angle = new_angle-2*np.pi

    while new_angle <= -np.pi:
        new_angle = new_angle+2*np.pi

    final_point = rotated_point+global_center[:2]

    ret =  np.append(final_point, new_angle)
    
    
    #A = np.array([[c, -s, global_center[0]], [s, c, global_center[1]], [0,0,1]])

    #t = np.array([local_pose[0],  local_pose[1], 1])

    #x = np.dot(A, t)

    #x = x[:2]

    #print(np.linalg.norm(x-ret[:2]))

    return ret

def glob2loc_pose(glob_pose, global_center):
    c, s = np.cos(-global_center[2]), np.sin(-global_center[2])
    rot_mat = np.array(((c, -s), (s, c)))

    point = glob_pose[:2]-global_center[:2]
    point = np.dot(rot_mat, point)

    new_angle  = glob_pose[2]-global_center[2]

    while  new_angle > np.pi:
        new_angle = new_angle-np.pi

    while new_angle <= -np.pi:
        new_angle = new_angle+np.pi

    return np.append(point, new_angle)


if __name__ == "__main__":
    a = np.array([7, 14, 0])
    b = np.array([7, 7, -np.pi])
    print(glob2loc_pose(a, b))




