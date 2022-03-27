import numpy as np
import scipy as sc
from control.matlab import *
from numpy import sin, cos
from Dynamics import *
from numpy import pi
from Constants import *

def compute_trim_states_input(x,Va,Y,R):
    alpha=x[0]
    beta=x[1]
    phi=x[2]
    u=Va*cos(alpha)*cos(beta)
    v=Va*sin(beta)
    w=Va*sin(alpha)*cos(beta)
    theta=alpha+Y
    p=(-Va/R)*sin(theta)
    q=(Va/R)*sin(phi)*cos(theta)
    r=(Va/R)*cos(phi)*cos(theta)
    
    x_trim=np.array([0,0,0,u,v,w,phi,theta,0,p,q,r])
      
    C_L=C_L_0+C_L_alpha*alpha
    C_D=C_D_0+C_D_alpha*alpha
    
    C_X=-C_D*cos(alpha)+C_L*sin(alpha)
    C_X_q=-C_D_q*cos(alpha)+C_L_q*sin(alpha)
    C_X_delta_e=-C_D_delta_e*cos(alpha)+C_L_delta_e*sin(alpha)
    
    C_Z=-C_D*sin(alpha)-C_L*cos(alpha)
    C_Z_q=-C_D_q*sin(alpha)-C_L_q*cos(alpha)
    C_Z_delta_e=-C_D_delta_e*sin(alpha)-C_L_delta_e*cos(alpha)
    
    d_e=(((Jxz*(p**2-r**2)+(Jx-Jz)*p*r)/(0.5*rho*(Va**2)*c*S_wing))-C_m_0-C_m_alpha*alpha-C_m_q*((c*q)/(2*Va)))/C_m_delta_e
    
    d_t=np.sqrt(((2*mass*(-r*v+q*w+gravity*sin(theta))-rho*(Va**2)*S_wing*(C_X+C_X_q*((c*q)/(2*Va))+C_X_delta_e*d_e))/(rho*S_prop*C_prop*k_motor**2))+((Va**2)/(k_motor**2)))
    
    temp_1=np.linalg.inv(np.array([[C_l_delta_a, C_l_delta_r],
                    [C_n_delta_a, C_n_delta_r]]))
    temp_2=np.array([[((-Gamma_1*p*q+Gamma_2*q*r)/(0.5*rho*(Va**2)*S_wing*b))-C_l_0-C_l_beta*beta-C_l_p*((b*p)/(2*Va))-C_l_r*((b*r)/(2*Va))],
                     [((-Gamma_7*p*q+Gamma_1*q*r)/(0.5*rho*(Va**2)*S_wing*b))-C_n_0-C_n_beta*beta-C_n_p*((b*p)/(2*Va))-C_n_r*((b*r)/(2*Va))]])
    
    temp_3=np.matmul(temp_1,temp_2)
    
    d_a=temp_3[0]
    
    d_r=temp_3[1]
        
    u_trim=np.array([d_e,d_a,d_r,d_t])
    # print("Trimmed [a*,B*,phi*]:")
    # print(x_trim)
    return (x_trim, u_trim)

def compute_trim_cost(x,Va,Y,R):
  
  #inputs
  alpha=x[0]
  beta=x[1]
  phi=x[2]

  #compute X_dot_star
  x_dot=np.array([[0],
                  [0],
                  [-Va*sin(Y)], # I am using Pd_dot not hdot..that is why there is a sign change
                  [0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [Va/R],
                  [0],
                  [0],
                  [0]])
  
  #compute trimmed states
  x_trim, u_trim=compute_trim_states_input(x,Va,Y,R)
  d_e=u_trim[0]
  d_a=u_trim[1]
  d_r=u_trim[2]
  d_t=u_trim[3]
  
  f_x, f_y, f_z, tau_phi, tau_theta, tau_psi=Forces_and_Moments(x_trim, [d_e, d_a, d_r, d_t], 0)
  U=np.array([f_x,f_y,f_z,tau_phi,tau_theta,tau_psi])
  
  #trimmed_inputs=np.array([d_e,d_t,d_a,d_r])

  states_dot=EquationsOfMotion(.1,x_trim,U) # 
  states_dot=np.array(states_dot)
  states_dot=states_dot.reshape(12,1)
  
  J=np.linalg.norm(x_dot-states_dot)**2

  return J

def compute_trim(Va, Y, R): # this is the function that is called to compute trim
    x0 = np.array([0,0,0])
    res = sc.optimize.minimize(lambda x: compute_trim_cost(x,Va,Y,R), x0, method='nelder-mead',options={'xatol': 1e-8, 'disp': True})
    x_trim, u_trim=compute_trim_states_input(res.x,Va,Y,R)
    return (x_trim, u_trim)

def compute_tf_coefficients(states_trim, control_input):
    d_e=control_input[0]
    d_a=control_input[1]
    d_r=control_input[2]
    d_t=control_input[3]

    Va_trim = np.sqrt(states_trim[3]**2 +states_trim[4]**2 +states_trim[5]**2)
    alpha_trim = np.arctan(states_trim[5]/states_trim[3])
    Va=Va_trim
    
    #[d_e,d_t,d_a,d_r] u_trim
    #$ define transfer function constants
    a_phi1   = -1/2*rho*Va**2*S_wing*b*C_p_p * b/(2*Va)
    a_phi2   = 1/2*rho*Va**2*S_wing*b*C_p_delta_a
    
    a_theta1 = -rho*Va**2*c*S_wing/(2*Jy)*C_m_q*c/(2*Va)
    a_theta2 = -rho*Va**2*c*S_wing/(2*Jy)*C_m_alpha
    a_theta3 = -rho*Va**2*c*S_wing/(2*Jy)*C_m_delta_e
   
    a_beta1     = -rho*Va*S_wing/(2*mass)*C_Y_beta
    a_beta2     =  rho*Va*S_wing/(2*mass)*C_Y_delta_r

    a_V1     = rho*Va_trim*S_wing/mass*(C_D_0+C_D_alpha*alpha_trim + C_D_delta_e*d_e)+rho*S_prop/mass*C_prop*Va_trim
    a_V2     = rho*S_prop/mass*C_prop*k_motor**2*d_t
    a_V3     = gravity
    
    return a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_beta1, a_beta2, a_V1, a_V2, a_V3

def compute_tf_models(x_trim, u_trim):
    a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_beta1, a_beta2, a_V1, a_V2, a_V3 = compute_tf_coefficients(x_trim, u_trim)

    Va_trim = np.sqrt(x_trim[3]**2 +x_trim[4]**2 +x_trim[5]**2)
    theta_trim = x_trim[7]      
    # define transfer functions
    T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0])
    T_chi_phi       = tf([gravity/Va_trim],[1,0])
    T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2])
    T_h_theta       = tf([Va_trim],[1,0])
    T_h_Va          = tf([theta_trim],[1,0])
    T_Va_delta_t    = tf([a_V2],[1,a_V1])
    T_Va_theta      = tf([-a_V3],[1,a_V1])
    T_beta_delta_r     = tf([a_beta2],[1,a_beta1])
    print('................Open Loop Transfer Functions.............')
    print('T_phi_delta_a=', T_phi_delta_a)
    print('T_theta_delta_e=', T_theta_delta_e)
    print('T_h_theta=', T_h_theta)
    print('T_beta_delta_r =', T_beta_delta_r)
    print('T_phi_delta_a=', T_phi_delta_a)

    return([T_phi_delta_a, T_chi_phi, T_theta_delta_e, T_h_theta,  T_h_Va, T_Va_delta_t, T_Va_theta, T_beta_delta_r])