import numpy as np
from numpy import pi
from Trim_Conditions import compute_tf_coefficients
from Constants import *


'''

# maximum possible rudder command
delta_r_max = 20*pi/180
# Roll command when delta_r_max is achieved
beta_max = 3
# pick natural frequency to achieve delta_a_max for step of phi_max
zeta_beta = 0.707
'''

# Proper Transfer Control Function for Roll
# T_phic_phi=tf([0,0,kp_phi*a_phi2],[1,a_phi1+kd_phi*a_phi2,kp_phi*a_phi2])
# def AutoPilot_TF_Functions():
#     a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_beta1, a_beta2, a_V1, a_V2, a_V3 = compute_tf_coefficients (states, control_input)

#     T_phic_phi=tf([0,0,kp_phi*a_phi2],[1,a_phi1+kd_phi*a_phi2,kp_phi*a_phi2])




def kPkDki_Calc(states, control_input):
    # Get Transfer Function Coefficients
    a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_beta1, a_beta2, a_V1, a_V2, a_V3 = compute_tf_coefficients (states, control_input)
    
    #Calculate Kp, omega_n, and Kd for phi (roll)
    kp_phi=delta_a_max/error_phi_max
    omega_n_phi=np.sqrt(kp_phi*a_phi2)
    kd_phi = (2*zeta_phi*omega_n_phi-a_phi1)/a_phi2

    Vg=np.sqrt(states[3]**2+states[4]**2)
    # Calculate Kp, and Ki for X
    omega_n_x = omega_n_phi/W_X
    kp_x = 2*zeta_x*omega_n_x*Vg/gravity
    ki_x = omega_n_x**2*Vg/gravity

    # Calculate Kp, and Kd for theta (pitch)
    kp_theta=delta_e_max/error_theta_max*np.sign(a_theta3)
    omega_n_theta = np.sqrt(a_theta2+kp_theta*abs(a_theta3))
    kd_theta=(2*zeta_theta*omega_n_theta-a_theta1)/a_theta2
    K_theta_DC=kp_theta*a_theta3/(a_theta2 +kp_theta*a_theta3)
    omega_n_h = omega_n_theta/W_h

    # Calculate Kp, and ki for sidslip angle (beta)
    ki_beta=omega_n_beta**2/a_beta2
    kp_beta=(2*zeta_beta*omega_n_beta-a_beta1)/a_beta2

    # Calculate Kp, Ki for altitude
    Va=np.sqrt(states[3]**2+states[4]**2+states[5]**2)

    ki_h = omega_n_h**2/(K_theta_DC*Va)
    kp_h = 2*zeta_h*omega_n_h/(K_theta_DC*Va)
    
    # Calculate Kp, Ki for airspeed based on pitch
    omega_n_V2 = omega_n_theta/W_V2
    ki_V2 = -omega_n_V2**2/(K_theta_DC*gravity)
    kp_V2 = (a_V1 - 2*zeta_V2*omega_n_V2)/(K_theta_DC*gravity)

    #Calculate Kp, ki for airspeed based on throttle
    ki_V = omega_n_V**2/a_V2
    kp_V = (2*zeta_V*omega_n_V - a_V1)/a_V2


    return [kp_phi, kd_phi, kp_x, ki_x, kp_theta, kd_theta, ki_beta, kp_beta, ki_h, kp_h, ki_V2, kp_V2, ki_V, kp_V]


# PID Loops for Each control surface / commanded input
def phi_PID_loop(phi_commanded, states, kp_phi, kd_phi):
    phi=states[6]
    p=states[9]
    error_phi=phi_commanded-phi

    delta_a_PID = kp_phi*error_phi-kd_phi*p
    return delta_a_PID

def course_PID_loop(heading_commanded, states, kp_x, ki_x):
    psi=states[8]
    heading_commanded=heading_commanded*pi/180
    error_heading = heading_commanded-psi

    phi_commanded=kp_x*error_heading+ki_x*error_heading
    return phi_commanded

def theta_PID_loop(theta_commanded, states, kp_theta, kd_theta):
    theta=states[7]
    q=states[10]
    error_theta=theta_commanded-theta

    delta_e_PID = kp_theta*error_theta-kd_theta*q
    return delta_e_PID

def Altitude(altitude_commanded, states, kp_h, ki_h):
    altitude = -states[2]
    error_altitude = altitude_commanded-altitude

    theta_commanded=kp_h*error_altitude+ki_h*error_altitude
    return(theta_commanded)

def Airspeed_Pitch(airspeed_commanded, states, kp_V2, ki_V2):
    Va=np.sqrt(states[3]**2+states[4]**2+states[5]**2)
    error_airspeed = airspeed_commanded-Va

    theta_commanded = kp_V2*error_airspeed+ki_V2*error_airspeed
    return theta_commanded

def Airspeed_Throttle(airspeed_commanded, states, kp_V, ki_V, trim_throttle_position):
    Va=Va=np.sqrt(states[3]**2+states[4]**2+states[5]**2)
    error_airspeed = airspeed_commanded-Va

    delta_t_PID = trim_throttle_position+kp_V*error_airspeed +ki_V*error_airspeed
    return(delta_t_PID)


def Longitudinal_State_Controller(commanded_input, states, trim_controls, KpKdKi_Values):
    kp_phi, kd_phi, kp_x, ki_x, kp_theta, kd_theta, ki_beta, kp_beta, ki_h, kp_h, ki_V2, kp_V2, ki_V, kp_V = KpKdKi_Values
    altitude_commanded = commanded_input[1]
    airspeed_commanded = commanded_input[2]
    trim_throttle=trim_controls[3]
    h=-states[2]
    if h<=50: #Takeoff
        delta_t=1
        theta_commanded=theta_takeoff
    elif h <altitude_commanded: #Climb
        delta_t=1
        theta_commanded=Airspeed_Pitch(airspeed_commanded, states, kp_V2, ki_V2)
    elif h==altitude_commanded: #@ Commanded Altitude
        delta_t=Airspeed_Throttle(airspeed_commanded, states, kp_V, ki_V, trim_throttle)
        theta_commanded=Altitude(altitude_commanded, states, kp_h, ki_h)
    elif h>altitude_commanded:
        delta_t=0
        theta_commanded=Airspeed_Pitch(airspeed_commanded, states, kp_V2, ki_V2)
    delta_e=theta_PID_loop(theta_commanded, states, kp_theta, kd_theta)
    return delta_e, delta_t



def AutoPilot(commanded_input, states, trim_controls, KpKdKi_Values):
    heading_commanded  = commanded_input[0]
    altitude_commanded = commanded_input[1]
    airspeed_commanded = commanded_input[2]
    kp_phi, kd_phi, kp_x, ki_x, kp_theta, kd_theta, ki_beta, kp_beta, ki_h, kp_h, ki_V2, kp_V2, ki_V, kp_V = KpKdKi_Values

    delta_e, delta_t=Longitudinal_State_Controller(commanded_input, states, trim_controls, KpKdKi_Values)
    commanded_phi=course_PID_loop(heading_commanded, states, kp_x, ki_x)
    delta_a=phi_PID_loop(commanded_phi, states, kp_phi, kd_phi)
    
    control_input=trim_controls
    control_input[0]=delta_e
    control_input[1]=delta_a
    control_input[3]=delta_t
    
    return control_input


'''
TODO Develop autopilot Tranfer Functions to graph 
def AutoPilot_transfer_functions():
    T_phic_phi=tf([0,0,kp_phi*a_phi2],[1,a_phi1+kd_phi*a_phi2,kp_phi*a_phi2])
'''