import numpy as np
from numpy import pi
from Trim_Conditions import compute_tf_coefficients
from Constants import *

def kPkDki_Calc(states, control_input):
    # Get Transfer Function Coefficients
    a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_beta1, a_beta2, a_V1, a_V2, a_V3 = compute_tf_coefficients (states, control_input)
    
    #Calculate Kp, omega_n, and Kd for phi (roll)
    kp_phi=delta_a_max/phi_max
    omega_n_phi=np.sqrt(kp_phi*a_phi2)
    kd_phi = (2*zeta_phi*omega_n_phi-a_phi1)/a_phi2
    ki_phi=0

    # Calculate Kp, and Ki for X
    Vg=np.sqrt(states[3]**2+states[4]**2+states[5]**2)
    omega_n_x = omega_n_phi/W_X
    kp_x = 2*zeta_x*omega_n_x*Vg/gravity
    ki_x = omega_n_x**2*Vg/gravity
    kd_x=0

    # Calculate Kp, and Kd for theta (pitch)
    kp_theta=delta_e_max/theta_max*np.sign(a_theta3)
    omega_n_theta = np.sqrt(a_theta2+kp_theta*abs(a_theta3))
    kd_theta=(2*zeta_theta*omega_n_theta-a_theta1)/a_theta2
    K_theta_DC=kp_theta*a_theta3/(a_theta2 +kp_theta*a_theta3)
    omega_n_h = omega_n_theta/W_h
    ki_theta=0

    # Calculate Kp, and ki for sidslip angle (beta)
    ki_beta=omega_n_beta**2/a_beta2
    kp_beta=(2*zeta_beta*omega_n_beta-a_beta1)/a_beta2
    kd_beta=0

    # Calculate Kp, Ki for altitude
    Va=np.sqrt(states[3]**2+states[4]**2+states[5]**2)

    ki_h = omega_n_h**2/(K_theta_DC*Va)
    kp_h = 2*zeta_h*omega_n_h/(K_theta_DC*Va)
    kd_h = 0

    # Calculate Kp, Ki for airspeed based on pitch
    omega_n_V2 = omega_n_theta/W_V2
    ki_V2 = -omega_n_V2**2/(K_theta_DC*gravity)
    kp_V2 = (a_V1 - 2*zeta_V2*omega_n_V2)/(K_theta_DC*gravity)
    kd_V2 = 0

    #Calculate Kp, ki for airspeed based on throttle
    ki_V = omega_n_V**2/a_V2
    kp_V = (2*zeta_V*omega_n_V - a_V1)/a_V2
    kd_V = 0

    return [kp_phi, ki_phi, kd_phi, kp_x, ki_x, kd_x, kp_theta, ki_theta, kd_theta, kp_beta, ki_beta, kd_beta, kp_h, ki_h, kd_h, kp_V2, ki_V2, kd_V2, kp_V, ki_V, kd_V, K_theta_DC]

def Limit_Control_inputs(control_input, limits):
    max_value=limits[0]
    min_value=limits[1]
    if control_input>=max_value:
        control_input=max_value
    elif control_input<=min_value:
        control_input=min_value
    return(control_input)

def PID_Loop(commanded_value, current_value, derivative_value, limits:list, PID_Vals:list, KpKiKd:list, dt):
    kp=KpKiKd[0]
    ki=KpKiKd[1]
    kd=KpKiKd[2]

    PID_error=PID_Vals[0]
    PID_integrator=PID_Vals[1]
    PID_differentiator=PID_Vals[2]

    error=commanded_value-current_value
    PID_integrator=PID_integrator+(dt/2)*(error+PID_error)
    PID_differentiator=derivative_value
    u=kp*error + ki*PID_integrator+kd*PID_differentiator

    u_saturated=Limit_Control_inputs(u, limits)

    if ki!=0:
        PID_integrator=PID_integrator + dt/ki*(u_saturated-u)

    PID_Vals=[error, PID_integrator, PID_differentiator]
    return u_saturated, PID_Vals

def AutoPilot(HeadAltAir, states, KpKiKd_values, dt, trim_throttle, PID_Values):
    Heading_Commanded  = HeadAltAir[0]*np.pi/180
    Altitude_Commanded = HeadAltAir[1]
    Airspeed_Commanded = HeadAltAir[2]

    #Unpack PID_Constants
    KpKiKd_phi   = KpKiKd_values[0:3]
    KpKiKd_chi   = KpKiKd_values[3:6]
    KpKiKd_theta = KpKiKd_values[6:9]
    KpKiKd_beta  = KpKiKd_values[9:12]
    KpKiKd_h     = KpKiKd_values[12:15]
    KpKiKd_V2    = KpKiKd_values[15:18]
    KpKiKd_V     = KpKiKd_values[18:21]
    
    #Unpack States
    pn=states[0]
    pe=states[1]
    pd=states[2]
    u=states[3]
    v=states[4]
    w=states[5]
    phi=states[6]
    theta=states[7]
    psi=states[8]
    p=states[9]
    q=states[10]
    r=states[11]

    Va=np.sqrt(u**2+v**2+w**2)
    Altitude=-pd

    PID_Values_phi   = PID_Values[0:3]
    PID_Values_chi   = PID_Values[3:6]
    PID_Values_theta = PID_Values[6:9]
    PID_Values_beta  = PID_Values[9:12]
    PID_Values_h     = PID_Values[12:15]
    PID_Values_V2    = PID_Values[15:18]
    PID_Values_V     = PID_Values[18:21]

    #Course PID
    phi_commanded, PID_Values_chi = PID_Loop(Heading_Commanded, psi, 0, [phi_max, -phi_max], PID_Values_chi, KpKiKd_chi, dt)

    #Roll PID
    delta_a, PID_Values_phi = PID_Loop(phi_commanded, phi, -p, [delta_a_max, -delta_a_max], PID_Values_phi, KpKiKd_phi, dt)
    
    # SideSlip PID
    beta=np.arcsin(v/Va)
    delta_r, PID_Values_beta = PID_Loop(0, beta, 0, [delta_r_max, -delta_r_max], PID_Values_beta, KpKiKd_beta, dt)

    # #Longitudinal State Controller
    # if Altitude<=Takeoff_Altitude: # Takeoff
    #     delta_t=1
    #     theta_commanded=theta_takeoff
    # elif Altitude<Altitude_Commanded-Altitude_Hold_Radius and Altitude>Takeoff_Altitude:
    #     delta_t=1
    #     theta_commanded, PID_Values_V2 = PID_Loop(Airspeed_Commanded, Va, 0, [theta_max, -theta_max], PID_Values_V2, KpKiKd_V2,dt)
    # elif Altitude>=Altitude_Commanded-Altitude_Hold_Radius and Altitude<=Altitude_Commanded+Altitude_Hold_Radius:
    #     delta_t, PID_Values_V = PID_Loop(Airspeed_Commanded, Va, 0, [1,0], PID_Values_V, KpKiKd_V, dt)
    #     delta_t=delta_t+trim_throttle
    #     theta_commanded, PID_Values_h  = PID_Loop(Altitude_Commanded, Altitude, 0, [theta_max, -theta_max], PID_Values_h, KpKiKd_h, dt)
    # elif Altitude>Altitude_Commanded + Altitude_Hold_Radius:
    #     delta_t=0
    #     theta_commanded, PID_Values_V2 = PID_Loop(Airspeed_Commanded, Va, 0, [theta_max, -theta_max], PID_Values_V2, KpKiKd_V2,dt)

    # delta_e, PID_Values_theta = PID_Loop(theta_commanded, theta, -q, [delta_e_max, -delta_e_max], PID_Values_theta, KpKiKd_theta, dt)
    
    PID_Values[0:3]   = PID_Values_phi
    PID_Values[3:6]   = PID_Values_chi
    PID_Values[6:9]   = PID_Values_theta
    PID_Values[9:12]  = PID_Values_beta
    PID_Values[12:15] = PID_Values_h
    PID_Values[15:18] = PID_Values_V2
    PID_Values[18:21] = PID_Values_V

    delta_e=0
    delta_t=0

    return [-delta_e, delta_a, delta_r, delta_t], PID_Values

def AutoPilot_transfer_functions(states, control_input, KpKdKi_Values):
    from control.matlab import tf
    Va=np.sqrt(states[3]**2 + states[4]**2 + states[5]**2)
    Vg=Va
    kp_phi, ki_phi, kd_phi, kp_x, ki_x, kd_x, kp_theta, ki_theta, kd_theta, kp_beta, ki_beta, kd_beta, kp_h, ki_h, kd_h, kp_V2, ki_V2, kd_V2, kp_V, ki_V, kd_V, K_theta_DC = KpKdKi_Values
    a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_beta1, a_beta2, a_V1, a_V2, a_V3 = compute_tf_coefficients (states, control_input)

    T_phi_phic        = tf([0,0,kp_phi*a_phi2],[1,a_phi1+kd_phi*a_phi2,kp_phi*a_phi2])
    T_X_Xc            = tf([0,kp_x*gravity/Vg,ki_x*gravity/Vg],[1, kp_x*gravity/Vg, ki_x*gravity/Vg])
    T_beta_betac      = tf([0, a_beta2*kp_beta, a_beta2*ki_beta],[1, a_beta1+a_beta2*kp_beta, a_beta2*ki_beta])
    T_theta_thetac    = tf([0, 0, kp_theta*a_theta3],[1, a_theta1+kd_theta*a_theta3, a_theta2+kp_theta*a_theta3])
    T_h_hc            = tf([0, K_theta_DC*Va*kp_h, K_theta_DC*Va*kp_h*ki_h/kp_h], [1, K_theta_DC*Va*kp_h, K_theta_DC*Va*ki_h])
    T_Va_Vac_Pitch    = tf([0, -K_theta_DC*gravity*kp_V2, -K_theta_DC*gravity*kp_V2*ki_V2/kp_V2],[1, a_V1-K_theta_DC*gravity*kp_V2, -K_theta_DC*gravity*ki_V2])
    T_Va_Vac_Throttle = tf([0, a_V2*kp_V, a_V2*ki_V], [1, a_V1+a_V2*kp_V, a_V2*ki_V])
    transfer_functions = [T_phi_phic, T_X_Xc, T_beta_betac, T_theta_thetac, T_h_hc, T_Va_Vac_Pitch, T_Va_Vac_Throttle]
    tf_names = ['Roll', 'Chi (Course', 'Beta (Sideslip)', 'Theta', 'Altitude', 'Pitch Velocity', 'Throttle Airspeed']
    print(T_phi_phic)
    print(T_beta_betac)
    print(T_theta_thetac)
    print(T_h_hc)
    print(T_Va_Vac_Pitch)
    print(T_Va_Vac_Throttle)
    return transfer_functions, tf_names

'''
# def Longitudinal_State_Controller(HeadAltAir, Va, Altitude, Trim_Throttle, KpKdKi_Values):
    
#     Altitude_Commanded=HeadAltAir[1]
#     Airspeed_Commanded=HeadAltAir[2]



#     h=-states[2]

#     if h<=50: #Takeoff
#         delta_t=1
#         theta_commanded=theta_takeoff

#     elif h <Altitude_Commanded: #Climb
#         delta_t=1
#         theta_commanded=PID_Loop(Airspeed_Commanded, Va, 0, [-theta_max, theta_max], PID_Vals_V2, KpKiKd_V2, dt)

#     elif h==altitude_commanded: #@ Commanded Altitude
#         delta_t=Airspeed_Throttle(airspeed_commanded, states, kp_V, ki_V, trim_throttle)
#         theta_commanded=Altitude(altitude_commanded, states, kp_h, ki_h)
#     elif h>altitude_commanded:
#         delta_t=0
#         theta_commanded=Airspeed_Pitch(airspeed_commanded, states, kp_V2, ki_V2)
#     delta_e=theta_PID_loop(theta_commanded, states, kp_theta, kd_theta)
#     return delta_e, delta_t
'''