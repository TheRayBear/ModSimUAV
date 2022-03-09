import numpy as np


# Rotation Matrix: Body Frame to Inertial Frame
def rotMat_body2inertial(phi, theta, psi): #Rotates Body from body frame to vehical frame
    R_b_w = np.array([[np.cos(theta)*np.cos(psi), np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.cos(psi)+np.sin(phi)*np.sin(psi)],
                         [np.cos(theta)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi), np.cos(phi)*np.sin(theta)*np.sin(psi)-np.sin(phi)*np.cos(psi)],
                         [-np.sin(theta), np.sin(phi)*np.cos(theta), np.cos(phi)*np.cos(theta)]])
    return R_b_w

# Rotation Matrix: Body Frame to Stability Frame
def rotMat_body2stability(alpha): #Body Frame to Stability Frame
    R_b_s = np.array([[np.cos(alpha) , 0, np.sin(alpha)],
                      [0             , 1 ,0            ],
                      [-np.sin(alpha), 0 ,np.cos(alpha)]])
    return R_b_s

# Rotation Matrix: Body Frame to Wind Frame
def rotMat_body2Wind(alpha, beta):
    R_b_w = np.array=([[ np.cos(beta)*np.cos(alpha), np.sin(beta),  np.cos(beta)*np.sin(alpha)],
                       [-np.sin(beta)*np.cos(alpha), np.cos(beta), -np.sin(beta)*np.sin(alpha)],
                       [-np.sin(alpha)             , 0           , np.cos(alpha)              ]])
    return R_b_w

# Rotation Matrix: Gyro Frame to Body Frame
def rotMat_Gyro2Body(phi,theta):
    R_g_b = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                      [0, np.cos(phi)              , -np.sin(phi)             ],
                      [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])
    return R_g_b

def rotMat_body2inertial_Wind(phi, theta, psi):
    R_b_w_wind=np.array([[np.cos(theta)*np.cos(psi), np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.cos(psi)+np.sin(phi)*np.sin(psi)],
                         [np.cos(theta)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi), np.cos(phi)*np.sin(theta)*np.sin(psi)-np.sin(phi)*np.cos(psi)],
                         [-np.sin(theta), np.sin(phi)*np.cos(theta), np.cos(phi)*np.cos(theta)]])
    return R_b_w_wind

def rotMat_body2inertial_Plane(phi, theta, psi): #Rotates Body from body frame to vehical frame
    R_b_w_plane = np.array([[np.cos(-theta) * np.cos(psi), np.sin(-phi) * np.sin(-theta) * np.cos(psi) - np.cos(-phi) * np.sin(psi), np.cos(-phi) * np.sin(-theta) * np.cos(psi) + np.sin(-phi) * np.sin(psi)],
                            [np.cos(-theta) * np.sin(psi), np.sin(-phi) * np.sin(-theta) * np.sin(psi) + np.cos(-phi) * np.cos(psi), np.cos(-phi) * np.sin(-theta) * np.sin(psi) - np.sin(-phi) * np.cos(psi)],
                            [-np.sin(-theta), np.sin(-phi) * np.cos(-theta), np.cos(-phi) * np.cos(-theta)]])
    return R_b_w_plane