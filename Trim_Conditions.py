import numpy as np
import scipy as sc
from control.matlab import *
from numpy import sin, cos
from Dynamics import *

#Define Constants used for Functions

#Forces and Moments 
Jx = 0.824
Jy = 1.135
Jz = 1.759
Jxz= 0 #.120 #Moment Coupling Constant

#Calculate Gamma
Gamma=Jx*Jz-Jxz**2
Gamma_1=Jxz*(Jx-Jy+Jz)/Gamma
Gamma_2=(Jz*(Jz-Jy)+Jxz**2)/Gamma
Gamma_3=Jz/Gamma
Gamma_4=Jxz/Gamma
Gamma_5=(Jz-Jx)/Jy
Gamma_6=Jxz/Jy
Gamma_7=((Jx-Jy)*Jx+Jxz**2)/Gamma
Gamma_8=Jx/Gamma

#Aircraft Constants
S_wing        = 0.55
b             = 2.90
c             = 0.19
S_prop        = 0.2027
rho           = 1.2682
e             = 0.9
AR            = b**2/S_wing
C_L_0         = 0.23
C_D_0         = 0.043
C_m_0         = 0.0135
C_L_alpha     = 5.61
C_D_alpha     = 0.030
C_m_alpha     = -2.74
C_L_q         = 7.95
C_D_q         = 0.0
C_m_q         = -38.21
C_L_delta_e   = 0.13
C_D_delta_e   = 0.0135
C_m_delta_e   = -0.99
M             = 50
alpha0        = 0.47
epsilon       = 0.16
C_D_p         = 0.0
C_Y_0         = 0.0
C_ell_0       = 0.0
C_n_0         = 0.0
C_Y_beta      = -0.98
C_ell_beta    = -0.13
C_n_beta      = 0.073
C_Y_p         = 0.0
C_ell_p       = -0.51
C_n_p         = -0.069
C_Y_r         = 0.0
C_ell_r       = 0.25
C_n_r         = -0.095
C_Y_delta_a   = 0.075
C_ell_delta_a = 0.17
C_n_delta_a   = -0.011
C_Y_delta_r   = 0.19
C_ell_delta_r = 0.0024
C_n_delta_r   = -0.069
C_prop        = 1
k_motor       = 80 #80
k_T_p         = 0
k_omega       = 0

gravity       = 9.806650
mass          = 13.5


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
    
    temp_1=np.linalg.inv(np.array([[C_ell_delta_a, C_ell_delta_r],
                    [C_n_delta_a, C_n_delta_r]]))
    temp_2=np.array([[((-Gamma_1*p*q+Gamma_2*q*r)/(0.5*rho*(Va**2)*S_wing*b))-C_ell_0-C_ell_beta*beta-C_ell_p*((b*p)/(2*Va))-C_ell_r*((b*r)/(2*Va))],
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

  #Va=35
  #R=99999999999
  #Y=0

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
