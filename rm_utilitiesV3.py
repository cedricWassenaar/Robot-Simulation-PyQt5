# Utility functions for the course Robot Modelling
# Rufus Fraanje (p.r.fraanje@hhs.nl), sept. 2016
#
# Additional functions added for more functionality
# Cédric Wassenaar (C.M.Wassenaar@student.hhs.nl), sept. 2018
# Joeri Verkerk (J.Verkerk-01@student.hhs.nl), sept. 2018
###############################################################################
import numpy as np
from numpy import cos, sin


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    """
    Check if input is a correct matrix
    :param R:
    :return:
    """
    Rt = np.transpose(R.copy())
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def inverse_kinematics_wrist(R):
    """
    Calculates the inverse kinematics of the wrist of the robot
    :param R:
    :return:
    """
    minplus = 1
    t5 = np.arctan2(minplus * np.sqrt(1 - (R[2, 2]**2)), R[2, 2])
    t4 = np.arctan2(minplus * R[1, 2], minplus * R[0, 2])
    t6 = np.arctan2(minplus * R[2, 1], minplus * -R[2, 0])

    R_check = np.array([[cos(t4) * cos(t5) * cos(t6) - sin(t4) * sin(t6) - R[0, 0], -cos(t4) * cos(t5) * sin(t6) - sin(t4) * cos(t6) - R[0, 1], cos(t4) * sin(t5) - R[0, 2]],
                        [sin(t4) * cos(t5) * cos(t6) + cos(t4) * sin(t6) - R[1, 0], -sin(t4) * cos(t5) * sin(t6) + cos(t4) * cos(t6) - R[1, 1], sin(t4) * sin(t5) - R[1, 2]],
                        [-sin(t5) * cos(t6) - R[2, 0],                              sin(t5) * sin(t6) - R[2, 1],                                cos(t5) - R[2, 2]]])

    return np.array([t4, t5, t6]), R_check


def make_rotation_matrix(axis, angle):
    """
    make a rotation matrix based on an angle and specified axis
    :param axis: string that specifies over which axis will be rotated
    :param angle: rotation angle in radians
    :return: rotation matrix
    """
    if axis == "x":
        return np.array([[1, 0,             0],
                         [0, cos(angle), -sin(angle)],
                         [0, sin(angle), cos(angle)]])

    elif axis == "y":
        return np.array([[cos(angle), 0, -sin(angle)],
                         [0,             1, 0],
                         [sin(angle), 0, cos(angle)]])

    elif axis == "z":
        return np.array([[cos(angle), -sin(angle), 0],
                         [sin(angle), cos(angle),  0],
                         [0,             0,              1]])


def make_DH_matrix(DH_parameters):
    """
    make a homogenious matrix based on the Denavit Hartenberg Convention
    :param DH_parameters: array of 4 with all DH parameters
    :return: DH matrix
    """
    from numpy import cos, sin
    length = DH_parameters[0]
    twist = DH_parameters[1]
    offset = DH_parameters[2]
    angle = DH_parameters[3]

    return np.array([[cos(angle), -sin(angle) * cos(twist), sin(angle) * sin(twist),  length * cos(angle)],
                     [sin(angle), cos(angle) * cos(twist),  -cos(angle) * sin(twist), length * sin(angle)],
                     [0,          sin(twist),               cos(twist),               offset],
                     [0,          0,                        0,                        1]])


def interpolate(values, precision):
    """Create positionvalues within the given trajectory
        precision = amount of subvalues"""
    nr_values = len(values)
    solution = []
    for nr in range(0, nr_values):
        if nr < nr_values - 1:
            delta_val = np.subtract(values[nr + 1], values[nr])
            x_val = np.true_divide(delta_val, precision)
            for x in range(0, precision):
                solution.append(np.add(values[nr], np.multiply(x_val, x)))
        else:
            break
    solution = np.array(solution)
    return solution


def make_homogenious_matrix(rotation, translation):
    return np.vstack((np.hstack((rotation, translation)), np.array([0, 0, 0, 1])))


# function for the inverse kinematics of a 3DOF robot
def inverse_algorithm_3DOF(arms, points, elbow_down=False):
    """Inverse kinematics of a scara robot.
    Inputs:
    arms: 3-element array/list with arm lengths
    point2: 3-element array with (x,y,z) coordinate of end point
    elbow_down (optional): True/False boolean to determine
    which solution needs to be returned
    Output:
    angles: 3-element array/list with angles in radians(!)
    """
    x = points[0]
    y = points[1]
    z = points[2]

    d1 = arms[0]
    d2 = arms[1]
    d3 = arms[2]

    s = z - d1
    r = np.sqrt(x**2 + y**2)
    c = np.sqrt(r**2 + s**2)

    beta = np.arctan2(s, r)
    alpha = np.arccos(np.minimum(1, ((-d3**2 + d2**2 + c**2) / (2 * d2 * c))))

    theta1 = np.arctan2(y, x)

    upper_cos = (-c**2 + d3**2 + d2**2)
    lower_cos = (2 * d3 * d2)
    if abs(upper_cos) > abs(lower_cos):
        return [0, 0, 0], True

    if elbow_down:
        theta2 = beta - alpha
        theta3 = np.radians(180) - np.arccos(np.minimum(1, (upper_cos / lower_cos)))
    else:
        theta2 = beta + alpha
        theta3 = -(np.radians(180) - np.arccos(np.minimum(1, (upper_cos / lower_cos))))

    angles = [theta1, theta2, theta3, 0]
    return angles, False


def kin_planar_forward(arms, angles):
    """Forward kinematics of a 2-link planar robot.

    Inputs:
        arms: 2-element array/list with arm lengths
        angles: 2-element array/list with angles in radians(!)

    Output:
        point2: 2-element numpy array with (x,y) coordinate of end point

    """
    x1 = arms[0] * np.cos(angles[0])
    y1 = arms[0] * np.sin(angles[0])
    x2 = x1 + arms[1] * np.cos(angles[0] + angles[1])
    y2 = y1 + arms[1] * np.sin(angles[0] + angles[1])
    points = np.array([x2, y2])
    return points


def kin_planar_inverse(arms, points, elbow_down=True):
    """Inverse kinematics of a 2-link planar robot.

    Inputs:
        arms: 2-element array/list with arm lengths
        point2: 2-element array with (x,y) coordinate of end point
        elbow_down (optional): True/False boolean to determine
                           which solution needs to be returned

    Output:
        angles: 2-element array/list with angles in radians(!)

    """
    x = points[0]
    y = points[1]
    a1 = arms[0]
    a2 = arms[1]
    D = (x ** 2 + y ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2)
    f = np.sqrt(1 - (D ** 2))
    if elbow_down:
        theta2 = np.arctan2(f, D)
    else:
        theta2 = np.arctan2(-f, D)
    theta1 = np.arctan2(y, x) - np.arctan2((a2 * np.sin(theta2)), (a1 + a2 * np.cos(theta2)))
    angles = np.array([theta1, theta2])
    return angles


def sphere():
    import pyqtgraph.opengl as gl
    sphere_data= gl.MeshData.sphere(rows=8,
                                    cols=16)
    obj = gl.GLMeshItem(meshdata=sphere_data,
                        smooth=False,
                        drawFaces=True,
                        faceColor=(0.2, 0.3, 0.4, 1),
                        drawEdges=False,
                        edgeColor=(0.2, 0.3, 0.4, 1))
    return obj


# cylinder is a convenience function to create a cylinder shape in
# pyqtgraph/OpenGL, it gives you a number of vertices distributed over the
# surface of the cylinder and triangular shaped faces that cover the whole
# surface of the cylinder
# cylinders are being used to visualize joints
def cylinder(radius, height, N):
    """Calculates vertices and faces for a cylinder for visualisation in
    pyqtgraph/OpenGL.

    Inputs:
        radius: radius of the cylinder
        height: height of the cylinder
        N: number of segments to approximate the circular shape of the cylinder

    Outputs:
        vertices: array with on each row the (x,y,z) coordinates of the vertices
        faces: array with triangular faces of the cylinder

    Note:
        The cylinder is a circle in the x,y plane with center at (0,0) that is
        extruded along the z-axis.

    """
    import scipy.spatial
    t = np.linspace(0, 2 * np.pi, N, endpoint=False).reshape(N, 1)
    vertices = np.zeros((2 * N, 3))
    vertices[0:N, :] = np.hstack((radius * np.cos(t), radius * np.sin(t), np.zeros((N, 1))))
    vertices[N:2 * N, :] = vertices[0:N, :] + np.hstack((np.zeros((N, 2)), height * np.ones((N, 1))))
    faces = np.zeros((N - 2 + 2 * N + N - 2, 3), dtype=np.uint)
    # bottom, makes use of Delaunay triangulation contained in Scipy's
    # submodule spatial (which on its turn makes use of the Qhull library)
    faces[0:N - 2, :] = scipy.spatial.Delaunay(vertices[0:N, 0:2], furthest_site=True, qhull_options='QJ').simplices[:,
                        -1::-1]
    # sides
    for i in range(N - 1):
        faces[N - 2 + 2 * i, :] = np.array([i, i + 1, N + i + 1], dtype=np.uint)
        faces[N - 2 + 2 * i + 1, :] = np.array([i, N + i + 1, N + i], dtype=np.uint)
    # final one between the last and the first:
    faces[N - 2 + 2 * (N - 1), :] = np.array([N - 1, 0, N], dtype=np.uint)
    faces[N - 2 + 2 * (N - 1) + 1, :] = np.array([N - 1, N, 2 * N - 1], dtype=np.uint)
    # top
    faces[N - 2 + 2 * N:N - 2 + 2 * N + N - 2, :] = N + faces[0:N - 2, -1::-1]

    return vertices, faces


# simular to the cylinder, but not for creating a box-shaped object
# boxes are used to visualize links
def box(size=(1, 1, 1)):
    """Calculates vertices and faces for a box for visualisation in
    pyqtgraph/OpenGL.

    Inputs:
        size: 3 element array/list with the width,depth,height, i.e.
              the dimensions along the x, y and z-axis.

    Outputs:
        vertices: array with on each row the (x,y,z) coordinates of the vertices
        faces: array with triangular faces of the box

    Note:
        The box is between (0,0,0) and (size[0],size[1],size[2]), note that
        negative sizes are not prevented but result in strange artifacts because
        it changes the orientation of the faces of the box (inside becomes
        outside).

    """
    vertices = np.zeros((8, 3))
    faces = np.zeros((12, 3), dtype=np.uint)
    xdim = size[0]
    ydim = size[1]
    zdim = size[2]
    vertices[0, :] = np.array([0, ydim, 0])
    vertices[1, :] = np.array([xdim, ydim, 0])
    vertices[2, :] = np.array([xdim, 0, 0])
    vertices[3, :] = np.array([0, 0, 0])
    vertices[4, :] = np.array([0, ydim, zdim])
    vertices[5, :] = np.array([xdim, ydim, zdim])
    vertices[6, :] = np.array([xdim, 0, zdim])
    vertices[7, :] = np.array([0, 0, zdim])

    faces = np.array([
        # bottom (clockwise, while looking from top)
        [2, 1, 0],
        [3, 2, 0],
        # sides (counter-clock-wise)
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
        # top (counter-clockwise)
        [4, 5, 6],
        [4, 6, 7]
    ], dtype=np.uint)

    return vertices, faces


def rotate_xyz(angles):
    """
    Calculates the rotations matrix for xyz angles
    (x,y,z)
    :param angles:
    :return:
    """
    x, y, z = angles
    rotate_x = np.array([[1, 0, 0],
                         [0, np.cos(x), np.sin(x)],
                         [0, -np.sin(x), np.cos(x)]])

    rotate_y = np.array([[np.cos(y), 0, -np.sin(y)],
                         [0, 1, 0],
                         [np.sin(y), 0, np.cos(y)]])

    rotate_z = np.array([[np.cos(z), -np.sin(z), 0],
                         [np.sin(z), np.cos(z), 0],
                         [0, 0, 1]])
    y_z = np.dot(rotate_z, rotate_y)
    x_y_z = np.dot(y_z, rotate_x)
    return x_y_z




