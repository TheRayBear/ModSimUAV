import numpy as np

#Example PID Loop

def rho_PID_loop(phi_commanded, states, kp_phi, kd_phi):
    phi=states[6]
    p=states[9]

    error_phi=phi_commanded-phi

    delta_a_PID = kp_phi*error_phi-kd_phi*p

    return delta_a_PID