import numpy as np
from control import *
from control.matlab import *
from RotationalMatricies import *


def wind(phi,theta,psi,Va,dt):
    # steady state wind in inertial frame
 
    wn=0
    we=0
    wd=0
    
    # gust params
    Lu=200
    Lv=200
    Lw=50
    sigma_u=1.06 # 2.12
    sigma_v=sigma_u
    sigma_w=0.7 # 1.4
    
    au=sigma_u*np.sqrt(2*Va/Lu)
    av=sigma_v*np.sqrt(3*Va/Lv)
    aw=sigma_w*np.sqrt(3*Va/Lw)
    
    # transfer functions
    
    num_u=[0,au]
    den_u=[1,Va/Lu]
    sys_u=tf(num_u,den_u)
    
    num_v=[av,av*Va/(np.sqrt(3)*Lv)]
    den_v=[1,2*Va/Lv, (Va/Lv)**2]
    sys_v=tf(num_v,den_v)
    
    num_w=[aw,aw*Va/(np.sqrt(3)*Lw)]
    den_w=[1,2*Va/Lw, (Va/Lw)**2]
    sys_w=tf(num_w,den_w)
    
    # 
    T=[0, dt]
    X0=0.0
    white_noise_u= 0*np.random.normal(0,1,1) # what should be the variance for the white noise ?
    white_noise_v= 0*np.random.normal(0,1,1)
    white_noise_w= 0*np.random.normal(0,1,1)
    
    y_u, T, x_u=lsim(sys_u,white_noise_u[0], T, X0)
    y_v, T, x_v=lsim(sys_v,white_noise_v[0], T, X0)
    y_w, T, x_w=lsim(sys_w,white_noise_w[0], T, X0)
    # gust components
    wg_u=y_u[1]
    wg_v=y_v[1]
    wg_w=y_w[1]
    Ws_v=np.array([wn, we, wd]) # steady state wind comp in vehicle frame
    R=rotMat_body2inertial(phi,theta,psi) # we want inertial to body
    Ws_b=np.matmul(R.T,Ws_v.T).T # steady state wind comp in vehicle frame
    Wg_b=np.array([wg_u, wg_v, wg_w])
    Vw=Wg_b+Ws_b
    
    return Vw

def air_data(states, dt):
    #Steady State Wind in Inertial Frame
    u=states[3]
    v=states[4]
    w=states[5]
    phi=states[6]
    theta=states[7]
    psi=states[8]
    Va0 = np.sqrt(u**2 + v**2 + w**2)
    Vw=wind(phi, theta, psi, Va0, dt)
    Va_b=np.array([u-Vw[0], v-Vw[1], w-Vw[2]])
    Va=np.sqrt(Va_b[0]**2+Va_b[1]**2+Va_b[2]**2)
    alpha=np.arctan2(Va_b[2], Va_b[0])
    beta=np.arcsin(Va_b[1]/Va)
    return (Va, alpha, beta, Vw)