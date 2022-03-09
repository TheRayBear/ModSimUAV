#Model and Simulation HW 7 - UAV  - Ryan Raber

# Import libraries

# Note - You will need to install numpy-stl, threading, and inputs for all aspects of this program to work.
# You will also need the delorean.stl file included with submission to run it on your own system.

# Rev .1 - Updateable Control

#%% from code import interact
import mpl_toolkits.mplot3d.axes3d as p3
from mpl_toolkits import mplot3d
from tkinter import *
import numpy as np
from stl import mesh
#from math import cos, radians, sin
import matplotlib.pyplot as plt
from matplotlib import animation, colors, cm
from matplotlib.widgets import Slider, Button
from inputs import get_gamepad
from scipy.integrate import solve_ivp
import math
import threading
from control.matlab import *
from RotationalMatricies import *
from UAVPlots import PlotCharts
from Wind import wind

#%%Bulk Simulation Parameters
Jx= 0.824
Jy= 1.135
Jz= 1.759
Jxz= 0 #.120
gravity=9.806650
mass=13.5

# aerodynamic parameters
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

#Calculate Gamma
Gamma=Jx*Jz-Jxz**2
Gamma_1=Jxz*(Jx-Jy+Jz)/Gamma
Gamma_2=Jz*(Jz-Jy)+Jxz**2
Gamma_3=Jz/Gamma
Gamma_4=Jxz/Gamma
Gamma_5=(Jz-Jx)/Jy
Gamma_6=Jxz/Jy
Gamma_7=((Jx-Jy)*Jx+Jxz**2)/Gamma
Gamma_8=Jx/Gamma

#Tables for Final Graphs
graph_data=[]

#Initial Controller Inputs
del_pitch=0
del_roll=0
del_yaw=0
del_throttle=0
throttle_pos=0

plane_pos_x=0
plane_pos_y=0
plane_pos_z=0

def Reset():
    global states
    global ui
    states=[0,0,0,35,0,0,0,0,0,0,0,0]
    ui=[0,0,0,0,0,0]

#Controller Input Backend
class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self): # return the buttons/triggers that you care about in this methode
        x = self.LeftJoystickX
        y = self.LeftJoystickY
        rx = self.RightJoystickX
        ry = self.RightJoystickY
        a = self.A
        b = self.B # b=1, x=2
        x_button = self.X
        y_button = self.Y
        rb = self.RightBumper
        lb = self.LeftBumper
        rt = self.RightTrigger
        lt = self.LeftTrigger

        return [x, y, rx, ry, a, b, x_button, y_button, rb, lb, rt, lt]


    def _monitor_controller(self):
        while True:
            deadzone=.1
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    #This Spaghetti BS allows me to set a deadzone on the controller. IE - it wont try and move when the physical stick is not being moved. - RR
                    value= event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                    if abs(value)<=deadzone:
                        self.LeftJoystickY=0
                    else:
                        self.LeftJoystickY=value
                elif event.code == 'ABS_X':
                    value= event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                    if abs(value)<=deadzone:
                        self.LeftJoystickX=0
                    else:
                        self.LeftJoystickX=value
                elif event.code == 'ABS_RY':
                    value = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                    if abs(value)<=deadzone:
                        self.RightJoystickY=0
                    else:
                        self.RightJoystickY=value
                elif event.code == 'ABS_RX':
                    value = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                    if abs(value)<=deadzone:
                        self.RightJoystickX=0
                    else:
                        self.RightJoystickX=value
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.X = event.state
                elif event.code == 'BTN_WEST':
                    self.Y = event.state
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state

#Define what Controller Inputs control what parameters
def Controller_Input(): #Controls Map for standard xbox controller
    joy = XboxController()
    while True:
        CtrlFactor=1
        throttleCtrlFactor=1/10000
        global del_pitch
        global del_roll
        global del_yaw
        global throttle_pos
        throttle_pos_init=throttle_pos
        del_roll = (joy.read()[1])*CtrlFactor
        del_pitch = (joy.read()[0])*CtrlFactor
        del_yaw = (joy.read()[10]-joy.read()[11])*CtrlFactor
        #con_fx = (joy.read()[3])*CtrlFactor
        #con_fy = (joy.read()[2])*CtrlFactor
        #con_fz = (joy.read()[9]-joy.read()[8])*CtrlFactor
        #FlightSimWindow.canvas.draw_idle()
        print(joy.read()) #For Debugging Purposes - AT LEAST IT WAS SUPPOSED TO BE, commenting out this line prevents the whole function from working, and I have no idea why.
        if joy.read()[5]==1:
            print('Backend Sucessfully Quit - Have a nice day!')
            plt.close()
            quit()
        if joy.read()[7]==1: #Press X to reset Position and Rotation
            Reset()         
        if throttle_pos+(joy.read()[6]*throttleCtrlFactor-joy.read()[4]*throttleCtrlFactor)>=1:
            throttle_pos = 1
        elif throttle_pos+(joy.read()[6]*throttleCtrlFactor-joy.read()[4]*throttleCtrlFactor)<=0:
            throttle_pos = 0
        else:
            throttle_pos = throttle_pos+(joy.read()[6]*throttleCtrlFactor-joy.read()[4]*throttleCtrlFactor)       

#Draw Plane
def DrawPlane(pos_ned, phi, theta, psi, throttle_pos):

    # Clear OLD_DATA
    axis1.clear()
    axis2.clear()
    # axis3.clear()

    R = rotMat_body2inertial(phi, theta, psi)

    plane_pos_x = (pos_ned[1] + eastOffset) #*simScaleFactor
    plane_pos_y = (pos_ned[0] + northOffset)#*simScaleFactor
    plane_pos_z = -(pos_ned[2] + downOffset) #*simScaleFactor 

    #Plot Plane
    axis1.set_xlim((plane_pos_x-1, plane_pos_x+1))
    axis1.set_ylim((plane_pos_y-1, plane_pos_y+1))
    axis1.set_zlim((plane_pos_z-1, plane_pos_z+1))
    axis1.set_title('UAV Import and Controller\nPress \'X\' to reset - Press \'B\' to close ')
    axis1.set_xlabel('East Axis (m)')
    axis1.set_ylabel('North Axis (m)')
    axis1.set_zlabel('Down Axis (m)')

    #Plot Throttle Position    
    throttleangle=-6*np.pi/4*throttle_pos+5*np.pi/4
    axis2.set_title('Throttle')
    axis2.plot(([0,throttleangle]), ([0,1]), 'k')
    axis2.set_yticklabels([])
    axis2.set_xticklabels([])
    axis2.set_aspect('equal')
    
    
    
    
    
    # Create Mesh from STL File
    planeMesh = mesh.Mesh.from_file("delorean.stl") # Include STL file name here
    planeMesh.rotate_using_matrix(R)
    planeMesh.x += simScaleFactor * (pos_ned[1] + eastOffset)  
    planeMesh.y += simScaleFactor * (pos_ned[0] + northOffset)
    planeMesh.z -= simScaleFactor * (pos_ned[2] + downOffset)
    collection = mplot3d.art3d.Poly3DCollection(planeMesh.vectors * stlScaleFactor, edgecolor = 'k', lw = 0.1)   # Create collection
    collection.set_facecolor(planeColor)
    axis1.add_collection3d(collection)     # Add collection to plot
    
def air_data(states, dt):
    #Steady State Wind in Inertial Frame
    u=states[3]
    v=states[4]
    w=states[5]
    phi=states[6]
    theta=states[7]
    psi=states[8]
    Vw=wind(phi, theta, psi, Va0, dt)
    Va_b=np.array([u-Vw[0], v-Vw[1], w-Vw[2]])
    Va=np.sqrt(Va_b[0]**2+Va_b[1]**2+Va_b[2]**2)
    alpha=np.arctan2(Va_b[2], Va_b[0])
    beta=np.arctan2(Va_b[1], Va)
    return (Va, alpha, beta, Vw)

def Forces_and_Moments(states, delta_e, delta_a, delta_r, delta_t):
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

    Va, alpha, beta, Vw = air_data(states, dt)

    qbar = .5*rho*Va**2
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
    CL=(1-sigma)*(C_L_0+C_L_alpha*alpha)

    CD=C_D_0+1/(np.pi*e*AR)*(C_L_0+C_L_alpha*alpha)**2
    CL=CL+np.sign(alpha)*sigma*2*sa*sa*ca
    
    #Compute and add Aerodynamic Forces
    f_x = f_x + qbar*S_wing*((-CD*ca + CL*sa) + (-C_D_q*ca + C_L_q*sa)*c*q/(2*Va))
    f_y = f_y + qbar*S_wing*((C_Y_0 + C_Y_beta*beta) + (C_Y_p*p + C_Y_r*r)*b/(2*Va))
    f_z = f_z + qbar*S_wing*((-CD*sa-CL*ca)+(-C_D_q*sa -C_L_q*ca)*c*q/(2*Va))

    #Calculate Aerodynamic Torques
    tau_phi = qbar*S_wing*b*((C_ell_0 + C_ell_beta*beta) + (C_ell_p*p + C_ell_r*r)*b/(2*Va))
    tau_theta = qbar*S_wing*c*((C_m_0 + C_m_alpha*alpha) + C_m_q*c*q/(2*Va))
    tau_psi = qbar*S_wing*b*((C_n_0+C_n_beta*beta) + (C_n_p*p + C_n_r*r)*b/(2*Va))

    #Compute and add Control Forces
    f_x=f_x + qbar*S_wing*(-C_D_delta_e*ca+C_L_delta_e*sa)*delta_e
    f_y=f_y + qbar*S_wing*(C_Y_delta_a*delta_a + C_Y_delta_r*delta_r)
    f_z=f_z + qbar*S_wing*(C_D_delta_e*sa+C_L_delta_e*ca)*delta_e

    #Compute Control Torques
    tau_phi = tau_phi + qbar*S_wing*b*(C_ell_delta_a*delta_a + C_ell_delta_r*delta_r)
    tau_theta = tau_theta + qbar*S_wing*c*C_m_delta_e*delta_e
    tau_psi = tau_psi + qbar*S_wing*b*(C_n_delta_a*delta_a+C_n_delta_r*delta_r)

    #Engine Thrust
    f_x = f_x + .5*rho*S_prop*C_prop*(k_motor**2*delta_t**2-Va**2)

    return (f_x, f_y, f_z, tau_phi, tau_theta, tau_psi)



## STL Formatting
northOffset = 0                     # Translational offsets
eastOffset = 0
downOffset = 0

stlScaleFactor = .05        # Scale plane and simulation parameters by equivalent amounts
simScaleFactor = 1 / stlScaleFactor

# Simulation Parameters
planeColor=[156/255, 156/255, 156/255]
T = 20
dt = 0.017
t_step = T / dt

#Initial Inputs
ui=[]
states=[]
Reset()
Va0=1

FlightSimWindow = plt.figure(figsize = (15, 15))
axis1 = FlightSimWindow.add_subplot(2,3,(1,5), projection = '3d')
axis2 = FlightSimWindow.add_subplot(2,3,3, projection = 'polar')

pos_ned=[states[0], states[1], states[2]]
phi=states[6]
theta=states[7]
psi=states[8]

DrawPlane(pos_ned,phi,theta,psi,throttle_pos)


#Start Listening for Controller Input
Controller_Type='XBox' #Options are 'XBox' or 'Yoke'
if Controller_Type=='XBox':
    ControllerThread=threading.Thread(target=Controller_Input)
#elif Controller_Type=='Yoke':
    #ControllerThread=threading.Thread(target=Controller_Input_Yoke)
ControllerThread.start()

def Dynamics(t,states, ui):
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
    
    #Unpack Inputs
    fx = ui[0]
    fy = ui[1]
    fz = ui[2]
    Mx = ui[3]
    My = ui[4]
    Mz = ui[5]                                                                                                  
    
    R=rotMat_body2inertial(phi, theta, psi)
    [[pnDot], [peDot],[pdDot]] = np.matmul(R,[[u],[v],[w]])

    [[uDot], [vDot], [wDot]] = np.array([[r*v-q*w],[p*w-r*u],[q*u-p*v]])+1/mass*np.array([[fx],[fy],[fz]])

    R=rotMat_Gyro2Body(phi, theta)
    [[phiDot],[thetaDot],[psiDot]] = np.matmul(R,np.array([[p],[q],[r]]))

    [[pDot], [qDot], [rDot]] = np.array([[Gamma_1*p*q-Gamma_2*q*r],
                                         [Gamma_5*p*r-Gamma_6*(p**2-r**2)],
                                         [Gamma_7*p*q-Gamma_1*q*r]])+\
                               np.array([[Gamma_3*Mx+Gamma_4*Mz],
                                         [My/Jy],
                                         [Gamma_4*Mx+Gamma_8*Mz]])
    
    xDot=[pnDot,peDot,pdDot,uDot,vDot,wDot,phiDot,thetaDot,psiDot,pDot,qDot,rDot]
    return(xDot)

def integrate(states,dt):
    global ui
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
    
    fx, fy, fz, Mx, My, Mz = Forces_and_Moments(states, del_pitch, del_roll, del_yaw, throttle_pos)
    ui=[fx,fy,fz,Mx,My,Mz]

    s = solve_ivp(lambda t, y: Dynamics(t, y, ui), [0,dt], [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r])
    states = s.y[:,-1].T
    return states

def update_plane(i):
    global t
    global states
    global ui
    global graph_data
    t=i*dt
       
    states=integrate(states, dt)

    pn=states[0]
    pe=states[1]
    pd=states[2]
    phi=states[6]
    theta=states[7]
    psi=states[8]
    
    #print(pn, pe, pd, phi, theta, psi)  

    pos_ned = np.array([pn, pe, pd])

    DrawPlane(pos_ned,phi,theta,psi,throttle_pos)

    t_data=[t]
    t_data.extend(states)
    graph_data.append(t_data)

    return (pos_ned)

anim = animation.FuncAnimation(FlightSimWindow, update_plane, frames = int(t_step),  repeat=False) #interval=1,

plt.show()

PlotCharts(graph_data, ['Time (s)', 'n (m)', 'e (m)', 'd (m)', 'u (m/s)', 'v (m/s)', 'w (m/s)', 'phi (rad)', 'theta (rad)', 'psi (rad)', 'p (rad)', 'q (rad)', 'r (rad)'])

#print(graph_data)
