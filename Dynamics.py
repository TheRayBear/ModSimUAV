import numpy as np
from RotationalMatricies import *
from Wind import *
from scipy.integrate import solve_ivp


#Forces and Moments 
Jx = 0.824
Jy = 1.135
Jz = 1.759
Jxz= 0.120 #Moment Coupling Constant

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


def Forces_and_Moments(states, control_input, dt):
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

    #Unpack User_Inputs
    delta_e = control_input[0]
    delta_a = control_input[1]
    delta_r = control_input[2]
    delta_t = control_input[3]

    Va, alpha, beta, Vw = air_data(states, dt)

    #Calculate simplifications for later
    ca=np.cos(alpha)
    sa=np.sin(alpha)

    #Compute Graivational Forces
    f_x = -mass*gravity*np.sin(theta)
    f_y =  mass*gravity*np.cos(theta)*np.sin(phi)
    f_z =  mass*gravity*np.cos(theta)*np.cos(phi)

    #Compute Lift and Drag Forces
    tmp1=np.exp(-M*(alpha-alpha0))
    tmp2=np.exp(M*(alpha+alpha0))
    sigma = (1+tmp1+tmp2)/((1+tmp1)*(1+tmp2))
    CL=(1-sigma)*(C_L_0+C_L_alpha*alpha) + np.sign(alpha)*sigma*2*sa*sa*ca

    CD=C_D_p+1/(np.pi*e*AR)*(C_L_0+C_L_alpha*alpha)**2
    
    CX_a   = -CD*ca + CL*sa
    CXq_a  = -C_D_q*ca + C_L_q*sa
    CXdele = -C_D_delta_e*ca + C_L_delta_e*sa
    CZ_a   = -CD*sa - CL*ca
    CZq_a  = -C_D_q*sa - C_L_q*ca
    CZdele = -C_D_delta_e*sa - C_L_delta_e*ca

    #Compute Aerodynamic and Control Forces
    f_x = f_x + 1/2*rho*Va**2*S_wing*(CX_a+CXq_a*c/(2*Va)*q+CXdele*delta_e)
    f_y = f_y + 1/2*rho*Va**2*S_wing*(C_Y_0 + C_Y_beta*beta + C_Y_p*p*b/(2*Va) + C_Y_r*r*b/(2*Va) + C_Y_delta_a * delta_a + C_Y_delta_r *delta_r)
    f_z = f_z + 1/2*rho*Va**2*S_wing*(CZ_a + CZq_a*c*q/(2*Va) + CZdele*delta_e)

    #Engine Thrust
    f_x = f_x + .5*rho*S_prop*C_prop*(k_motor**2*delta_t**2-Va**2)


    #Calculate Aerodynamic Torques
    tau_phi   = 1/2*rho*Va**2*S_wing*b*(C_ell_0 + C_ell_beta*beta + C_ell_p*p*b/(2*Va) + C_ell_r*r*b/(2*Va) + C_ell_delta_a*delta_a + C_ell_delta_r*delta_r)
    tau_theta = 1/2*rho*Va**2*S_wing*c*(C_m_0 + C_m_alpha*alpha + C_m_q*c*q/(2*Va) + C_m_delta_e*delta_e)
    tau_psi   = 1/2*rho*Va**2*S_wing*b*((C_n_0+C_n_beta*beta) + (C_n_p*p + C_n_r*r)*b/(2*Va) + C_n_delta_a*delta_a + C_n_delta_r*delta_r)

    # Engine Torque
    tau_phi = tau_phi + -k_T_p*(k_omega*delta_t)**2


    return (f_x, f_y, f_z, tau_phi, tau_theta, tau_psi)

def EquationsOfMotion(t,states, Forces):
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
    
    #Unpack Forces and Moments
    fx = Forces[0]
    fy = Forces[1]
    fz = Forces[2]
    l = Forces[3]
    m = Forces[4]
    n = Forces[5]                                                                                                  
    
    R=rotMat_body2inertial(phi, theta, psi)
    [[pnDot], [peDot],[pdDot]] = np.matmul(R,[[u],[v],[w]])

    [[uDot], [vDot], [wDot]] = np.array([[r*v-q*w],[p*w-r*u],[q*u-p*v]])+1/mass*np.array([[fx],[fy],[fz]])

    R=rotMat_Gyro2Body(phi, theta)
    [[phiDot],[thetaDot],[psiDot]] = np.matmul(R,np.array([[p],[q],[r]]))

    [[pDot], [qDot], [rDot]] = np.array([[Gamma_1*p*q-Gamma_2*q*r],
                                         [Gamma_5*p*r-Gamma_6*(p**2-r**2)],
                                         [Gamma_7*p*q-Gamma_1*q*r]])+\
                               np.array([[Gamma_3*l+Gamma_4*n],
                                         [m/Jy],
                                         [Gamma_4*l+Gamma_8*n]])
    
    xDot=[pnDot,peDot,pdDot,uDot,vDot,wDot,phiDot,thetaDot,psiDot,pDot,qDot,rDot]
    return(xDot)

def integrate(states,dt,control_input):
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
    
    fx, fy, fz, Mx, My, Mz = Forces_and_Moments(states, control_input, dt)
    FandM = [fx, fy, fz, Mx, My, Mz]

    s = solve_ivp(lambda t, y: EquationsOfMotion(t, y, FandM), [0,dt], [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r])
    states = s.y[:,-1].T
    return states

