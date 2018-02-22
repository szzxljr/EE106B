# may need more imports
import numpy as np
from utils import vec, adj

def cal_distance(point1, point2):
    """point1, 2 is np.array 1x3 a
       return the distance scalar"""
    return np.linalg.norm(point1.reshape((3, 1)) - point2.reshape((3, 1)))

def cal_angle(point1, point2):
    """point1, 2 is np.array 1x3 a
       return the angle in rad"""
    lx, ly = np.linalg.norm(point1.reshape((3, 1))), np.linalg.norm(point2.reshape((3, 1)))
    cos_angle = point1.dot(point2) / (lx * ly)
    angle = np.arccos(cos_angle)
    if angle > np.pi / 2:
        return np.pi - angle
    else:
        return angle

def compute_force_closure(contacts, normals, mu):

#def compute_force_closure(contacts, normals, num_facets, mu, gamma, object_mass):
    """ Compute the force closure of some object at contacts, with normal vectors stored in normals
        You can use the line method described in HW2.  if you do you will not need num_facets

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    # YOUR CODE HERE
    grasp_candidates = []
    grasp_candidates_normals = []
    factor = 30
    length = contacts.shape[0] 
    for i in range(0, length, factor):
        for j in range(0, length, factor):
            c1, c2 = contacts[i],  contacts[j]
            if c1[2] == c2[2] or normals[i].dot(normals[j]) > 0 or c1[2]<0.02 or c2[2]<0.02:
                continue
            distance = cal_distance(c1, c2)
            
            if distance < 0.01 or distance > 0.04:
                continue
            else:
                pointVector = (c1 - c2)
                theta1 = cal_angle(pointVector, normals[i])
                theta2 = cal_angle(pointVector, normals[j])
                coneangle = np.arctan(mu)
                if theta1 < coneangle and theta2 < coneangle:
                    grasp_candidates.append(list(c1) + list(c2))
                    grasp_candidates_normals.append(list(normals[i]) + list(normals[j]))

    return np.array(grasp_candidates), np.array(grasp_candidates_normals)



# defined in the book on page 219
def get_grasp_map(contacts, normals, num_facets, mu, gamma):
    """ Compute the grasp map given the contact points and their surface normals

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient

    Returns
    -------
    :obj:`numpy.ndarray` grasp map
    """
    # YOUR CODE HERE
    pass

def contact_forces_exist(contacts, normals, num_facets, mu, gamma, desired_wrench):
    """ Compute whether the given grasp (at contacts with surface normals) can produce the desired_wrench.
        will be used for gravity resistance. 

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    desired_wrench : :obj:`numpy.ndarray`
        potential wrench to be produced

    Returns
    -------
    bool : whether contact forces can produce the desired_wrench on the object
    """
    # YOUR CODE HERE
    pass

def compute_gravity_resistance(contacts, normals, num_facets, mu, gamma, object_mass):
    """ Gravity produces some wrench on your object.  Computes whether the grasp can produce and equal and opposite wrench

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    # YOUR CODE HERE (contact forces exist may be useful here)
    pass

# def compute_custom_metric(contacts, normals, num_facets, mu, gamma, object_mass):
def compute_custom_metric(contacts, normals, mu):
    """ I suggest Ferrari Canny, but feel free to do anything other metric you find. 

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    # YOUR CODE HERE :)
    scores = np.zeros((contacts.shape[0], 1))
    for i in range(contacts.shape[0]):
        c1 = contacts[i][0:3]
        c2 = contacts[i][3:]
        n1, n2 = normals[i][0:3], normals[i][3:]
        theta1, theta2 = cal_angle(c1 - c2, n1), cal_angle(c1 - c2, n2)
        theta = cal_angle(c1 - c2, np.array([0, 0, 1]))
        score1 = (1- ( np.power(theta1,2)+np.power(theta2,2) )/2/np.power(np.arctan(mu),2) )*100
        score2 = (1- np.power( theta/np.pi*2 ,2) )*100
        score = score1*0.1 + score2 * 0.9
        scores[i] = score
    max_index = np.where(scores==np.max(scores))[0][0]
    return contacts[max_index], normals[max_index]

TAG2OBJ = {'pawn': [-.06, .11]}


    