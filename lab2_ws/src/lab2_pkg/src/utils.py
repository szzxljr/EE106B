import numpy as np
from math import sin, cos, atan2
from geometry_msgs.msg._Point import Point
import tf.transformations as tfs
from geometry_msgs.msg import Pose, PoseStamped

def length(vec):
    return math.sqrt(vec.dot(vec))

def normalize(vec):
    return vec / length(vec)
    
def vec(*args):
    if len(args) == 1:
        if type(args[0]) == tuple:
            return np.array(args[0])
        elif type(args[0]) == Point:
            return np.array((args[0].x, args[0].y, args[0].z))
        else:
            return np.array(args)
    else:
        return np.array(args)

def hat(v):
    if v.shape == (3, 1) or v.shape == (3,):
        return np.array([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])
    elif v.shape == (6, 1) or v.shape == (6,):
        return np.array([
                [0, -v[5], v[4], v[0]],
                [v[5], 0, -v[3], v[1]],
                [-v[4], v[3], 0, v[2]],
                [0, 0, 0, 0]
            ])
    else:
        raise ValueError

def adj(g):
    if g.shape == (4, 4):
        R = g[0:3,0:3]
        p = g[0:3,3]
        result = np.zeros((6, 6))
        result[0:3,0:3] = R
        result[0:3,3:6] = hat(p) * R
        result[3:6,3:6] = R
        return result
    else:
        raise ValueError

def twist_from_tf(g):
    return vec(g[0,2], g[1,2], atan2(g[1,0], g[0,0]))

def rotation2d(theta):
    return np.array([
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)]
        ])

def rigid(twist):
    return np.array([
            [cos(twist[2]), -sin(twist[2]), twist[0]],
            [sin(twist[2]), cos(twist[2]), twist[1]],
            [0, 0, 1]
        ])

def look_at_general(eye, center, up):
    f = tfs.unit_vector(center - eye)
    u = tfs.unit_vector(up)
    s = tfs.unit_vector(np.cross(f,u))
    u = np.cross(s,f) 

    result = np.eye(4)
    result[0:3,0] = -s
    result[0:3,1] = -f
    result[0:3,2] = u
    result[0:3,3] = eye
    return result

def look_at(direction, up):
    z = tfs.unit_vector(direction)
    x = tfs.unit_vector(np.cross(up, z))
    y = np.cross(z,x).reshape(-1,1)
    x = x.reshape(-1,1)
    z = z.reshape(-1,1)
    return np.hstack((x, y, z))

def rotate_vector(vec, q):
    return tfs.quaternion_matrix(q).dot()

def create_pose_from_rigid_transform(g):
    position = tfs.translation_from_matrix(g)
    quaternion = tfs.quaternion_from_matrix(g)
    wpose = Pose()
    wpose.position.x = position[0]
    wpose.position.y = position[1]
    wpose.position.z = position[2]
    wpose.orientation.x = quaternion[0]
    wpose.orientation.y = quaternion[1]
    wpose.orientation.z = quaternion[2]
    wpose.orientation.w = quaternion[3]
    return wpose