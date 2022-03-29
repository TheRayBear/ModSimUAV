from numpy import pi
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
C_l_0         = 0.0
C_n_0         = 0.0
C_Y_beta      = -0.98
C_l_beta      = -0.13
C_n_beta      = 0.073
C_Y_p         = 0.0
C_l_p         = -0.51
C_n_p         = -0.069
C_Y_r         = 0.0
C_l_r         = 0.25
C_n_r         = -0.095
C_Y_delta_a   = 0.075
C_l_delta_a   = 0.17
C_n_delta_a   = -0.011
C_Y_delta_r   = 0.19
C_l_delta_r   = 0.0024
C_n_delta_r   = -0.069
C_prop        = 1
k_motor       = 80 #80
k_T_p         = 0
k_omega       = 0

gravity       = 9.806650
mass          = 13.5

#Transfer Function Constants
C_P_0    = Gamma_3*C_L_0 + Gamma_4*C_n_0
C_p_beta = Gamma_3*C_l_beta + Gamma_4*C_n_beta
C_p_p    = Gamma_3*C_l_p + Gamma_4*C_n_p
C_p_r    = Gamma_3*C_l_r + Gamma_4*C_n_r
C_p_delta_a = Gamma_3*C_l_delta_a + Gamma_4*C_n_delta_a
C_p_delta_r = Gamma_3*C_l_delta_r + Gamma_4*C_n_delta_r 
C_r_0    = Gamma_4*C_L_0 + Gamma_8*C_n_0
C_r_beta = Gamma_4*C_l_beta + Gamma_8*C_n_beta
C_r_p    = Gamma_4*C_l_p + Gamma_8*C_n_p
C_r_r    = Gamma_4*C_l_r + Gamma_8*C_n_r
C_r_delta_a = Gamma_4*C_l_delta_a + Gamma_8*C_n_delta_a
C_r_delta_r = Gamma_4*C_l_delta_r + Gamma_8*C_n_delta_r

# Autopilot Constants
# - Roll (Phi)
delta_a_max = 45*pi/180 # .5      # Max Aileron Command
phi_max = 15*pi/180 #pi/6  # Max Phi Allowed
zeta_phi = .7 #.7         # Natural Frequency for Roll

# - Course (X)
zeta_x = .8 #1.79 #.8           # Natural Frequency for Course
W_X = 15              # Seperation Parameter for calculating omega_X

# - Sideslip (beta)
zeta_beta = .707 #1 # .86 #.707
omega_n_beta=3
delta_r_max=20*pi/180

# - Pitch (Theta)
delta_e_max = 45*pi/180 # .5 # 45*pi/180
theta_max = 30*pi/180
zeta_theta = .8 # 2 #.8

# - Altitude
zeta_h= .82 #.92
W_h=16

# - Airspeed (Pitch)
W_V2 = 10
zeta_V2= .82 #.92 # .82

# - Airspeed (Throttle)
zeta_V = .707 #.907 # .707
omega_n_V = 3

theta_takeoff=10*pi/180

Takeoff_Altitude=5 #Meters
Altitude_Hold_Radius=5 #Meters
