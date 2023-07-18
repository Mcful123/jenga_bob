import numpy as np
from scipy.linalg import expm
from jenga_header import *
from math import degrees

w_1 = np.array([0, 0, 1])
q_1 = np.array([-0.150, 0.150, 0.01])
v_1 = np.cross(-w_1, q_1)

w_2 = np.array([0, 1, 0])
q_2 = np.array([-0.150, 0.270, 0.162])
v_2 = np.cross(-w_2, q_2)

w_3 = np.array([0, 1, 0])
q_3 = np.array([0.094, 0.270, 0.162])
v_3 = np.cross(-w_3, q_3)

w_4 = np.array([0, 1, 0])
q_4 = np.array([0.307, 0.177, 0.162])
v_4 = np.cross(-w_4, q_4)

w_5 = np.array([1, 0, 0])
q_5 = np.array([0.307, 0.260, 0.162])
v_5 = np.cross(-w_5, q_5)

w_6 = np.array([0, 1, 0])
q_6 = np.array([0.390, 0.260, 0.162])
v_6 = np.cross(-w_6, q_6)

PI = np.pi

def Get_MS():
    # =================== Your code starts here ====================#
    # Fill in the correct values for a1~6 and q1~6, as well as the M matrix
    M = np.eye(4)
    S = np.zeros((6, 6))
    S[0] = np.concatenate((w_1, v_1))
    S[1] = np.concatenate((w_2, v_2))
    S[2] = np.concatenate((w_3, v_3))
    S[3] = np.concatenate((w_4, v_4))
    S[4] = np.concatenate((w_5, v_5))
    S[5] = np.concatenate((w_6, v_6))

    M = np.array(
        [[0, -1, 0, 0.390], [0, 0, -1, 0.401], [1, 0, 0, 0.2155], [0, 0, 0, 1]]
    )

    # ==============================================================#
    return M, S


def S_m(S_vec):
    S = np.zeros((4, 4))

    # Fill in Rotation
    S[0][1] = -S_vec[2]
    S[0][2] = S_vec[1]
    S[1][0] = S_vec[2]
    S[1][2] = -S_vec[0]
    S[2][0] = -S_vec[1]
    S[2][1] = S_vec[0]

    S[0][3] = S_vec[3]
    S[1][3] = S_vec[4]
    S[2][3] = S_vec[5]

    return S


def ros_to_ur(theta1, theta2, theta3, theta4, theta5, theta6):
    return_value = [None, None, None, None, None, None]
    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5 * PI)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value


def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

    print("Foward kinematics calculated:\n")

    # =================== Your code starts here ====================#
    theta = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
    T = np.eye(4)

    M, S = Get_MS()

    T = np.matmul(expm(theta6 * S_m(S[5])), M)
    T = np.matmul(expm(theta5 * S_m(S[4])), T)
    T = np.matmul(expm(theta4 * S_m(S[3])), T)
    T = np.matmul(expm(theta3 * S_m(S[2])), T)
    T = np.matmul(expm(theta2 * S_m(S[1])), T)
    T = np.matmul(expm(theta1 * S_m(S[0])), T)

    print(T)

    return T


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""


def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    # =================== Your code starts here ====================#

    xWgrip = xWgrip + 0.15
    yWgrip = yWgrip - 0.15
    zWgrip = zWgrip - 0.01

    x_cen = xWgrip - 0.0535 * np.cos(yaw_WgripDegree * np.pi / 180)
    y_cen = yWgrip - 0.0535 * np.sin(yaw_WgripDegree * np.pi / 180)
    z_cen = zWgrip

    # print(f"center {x_cen}, {y_cen}, {z_cen}")

    # Find the angle between world and the center
    omega = np.arctan2(y_cen, x_cen)
    theta1_inv = np.arcsin(0.110 / np.sqrt(x_cen ** 2 + y_cen ** 2))
    theta1 = omega - theta1_inv
    # print(f"{omega}, {theta1_inv}, {theta1}")

    theta6 = np.pi / 2 + theta1 - (yaw_WgripDegree * np.pi) / 180

    x3_end = x_cen + 0.110 * np.sin(theta1) - 0.083 * np.cos(theta1)
    y3_end = y_cen - 0.083 * np.sin(theta1) - 0.110 * np.cos(theta1)
    z3_end = z_cen + 0.141

    D = np.sqrt(x3_end ** 2 + y3_end ** 2)
    H = z3_end - 0.152

    theta3_big = np.arccos(
        (D ** 2 + H ** 2 - 0.244 ** 2 - 0.213 ** 2) / (-2 * 0.244 * 0.213)
    )
    theta3 = PI - theta3_big

    theta2_small = np.arctan2(H, D)
    theta2_theta2_small = np.arcsin(
        0.213 * np.sin(theta3_big) / np.sqrt(D ** 2 + H ** 2)
    )
    theta2 = -theta2_theta2_small - theta2_small
    theta4 = -theta2 - theta3

    theta5 = -PI / 2
    # ==============================================================#
    lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
    return ros_to_ur(theta1, theta2, theta3, theta4, theta5, theta6)
