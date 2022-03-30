#Model and Simulation HW 7 - UAV

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
import math
import threading
from control.matlab import *
from RotationalMatricies import *
from Draw_Plots import *
# from Wind import *
from Dynamics import *
from Trim_Conditions import *
from AutoPilot import *

import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning) 

#Initial Conditions
Va0=0 #Meters
Initial_Altitude=0 #meters
T = 50
# dt=.05
dt =  0.017

#Initial Trim Conditions
Va=35 #40
Y=0 #.2
# R=400
R=999999999999999


#Tables for Final Graphs
graph_data=[]
tf_data=[]

def Reset():
    global states
    global user_input
    global FandM
    states=[0,0,-Initial_Altitude,Va0,0,0,0,0,0,0,0,0]
    FandM = [0,0,0,0,0,0]
    user_input=[0,0,0,0]


'''
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
        CtrlFactor=.5
        throttleCtrlFactor=1/10000
        global user_input
        user_input[1] = -(joy.read()[1])*CtrlFactor #Roll
        user_input[0] = -(joy.read()[0])*CtrlFactor #Pitch
        user_input[2] = (joy.read()[11]-joy.read()[10])*CtrlFactor #Yaw
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
        if user_input[3]+(joy.read()[6]*throttleCtrlFactor-joy.read()[4]*throttleCtrlFactor)>=1:
            user_input[3] = 1
        elif user_input[3]+(joy.read()[6]*throttleCtrlFactor-joy.read()[4]*throttleCtrlFactor)<=0:
            user_input[3] = 0
        else:
            user_input[3] = user_input[3]+(joy.read()[6]*throttleCtrlFactor-joy.read()[4]*throttleCtrlFactor)       
'''

## STL Formatting
northOffset = 0                     # Translational offsets
eastOffset = 0
downOffset = 0

t_step = T / dt

#Initial Inputs
user_input=[]
FandM=[]
states=[]
Reset()

'''
axdown = plt.axes([0.2, 0.25, 0.0225, 0.63])
throttle_slider = Slider(
    ax=axdown,
    label="Throttle",
    valmin=0,
    valmax=1,
    valinit=0,
    valstep=0.05,
    orientation="vertical"
)
axroll = plt.axes([0.25, 0.08, 0.65, 0.03])
roll_slider = Slider(
    ax=axroll,
    label='Inputted Roll',
    valmin=-.5,
    valmax=.5,
    valstep=0.1,
    valinit=0,
    
)
axyaw = plt.axes([0.25, 0.04, 0.65, 0.03])
yaw_slider = Slider(
    ax=axyaw,
    label='Inputted Yaw',
    valmin=-.5,
    valmax=.5,
    valstep=0.1,
    valinit=0,
    
)
axpitch = plt.axes([0.25, 0.00, 0.65, 0.03])
pitch_slider = Slider(
    ax=axpitch,
    label="Inputted Pitch",
    valmin=-.5,
    valmax=.5,
    valinit=0,
    valstep=0.1,
)

def update(val):
    global user_input
    user_input[1]=pitch_slider.val
    user_input[0]=-roll_slider.val
    user_input[2]=yaw_slider.val
    user_input[3]=throttle_slider.val
    FlightSimWindow.canvas.draw_idle()

# plt.show()
roll_slider.on_changed(update)
pitch_slider.on_changed(update)
yaw_slider.on_changed(update)
throttle_slider.on_changed(update)
'''

FlightSimWindow = plt.figure(figsize = (15, 15))
axis1 = FlightSimWindow.add_subplot(2,4,(1,6), projection = '3d')
axis2 = FlightSimWindow.add_subplot(2,4,3, projection = 'polar')
axis3 = FlightSimWindow.add_subplot(2,4,4)
axis4 = FlightSimWindow.add_subplot(2,4,7)
axis5 = FlightSimWindow.add_subplot(2,4,8)

Draw_Plane_STL(states, FlightSimWindow, user_input[3])

'''
# #Start Listening for Controller Input
# Controller_Type='XBox' #Options are 'XBox' or 'Yoke'
# if Controller_Type=='XBox':
#     ControllerThread=threading.Thread(target=Controller_Input)
# #elif Controller_Type=='Yoke':
#     #ControllerThread=threading.Thread(target=Controller_Input_Yoke)
# ControllerThread.start()
'''

trim_states, trim_controls = compute_trim(Va, Y, R)
controls=list(trim_controls)

KpKiKd_Values=kPkDki_Calc(trim_states, trim_controls)

trim_states[0]=states[0]
trim_states[1]=states[1]
trim_states[2]=states[2]
states=list(trim_states)

# Trim_Transfer_Functions = compute_tf_models(states, trim_controls)

autopilot_commanded_states=[0, 20, 35] # Initial Commanded States

AP_TFs, AP_TF_Names = AutoPilot_transfer_functions(states, trim_controls, KpKiKd_Values)

PID_Values=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

# This takes command line inputs and sets inputs.
def Commander():
    global autopilot_commanded_states
    while True:
        new_heading=float(input('Heading:'))
        new_altitude=float(input('Altitude:'))
        new_airspeed=float(input('Airspeed:'))
        autopilot_commanded_states=[new_heading, new_altitude, new_airspeed]
FlightCommander=threading.Thread(target=Commander)
FlightCommander.start()

def update_plane(i):
    global t
    global states
    global graph_data
    global autopilot_commanded_states
    global PID_Values
    t=i*dt  
    
    controls, PID_Values = AutoPilot(autopilot_commanded_states, states, KpKiKd_Values, dt, trim_controls[3], PID_Values)    
    
    controls[2]=trim_controls[2]
    states=integrate(states, dt, controls)

    Draw_Plane_STL(states, FlightSimWindow, controls[3], [northOffset, eastOffset, downOffset])

    t_data=[t]
    t_data.extend(states)
    graph_data.append(t_data)
    return (states)

anim = animation.FuncAnimation(FlightSimWindow, update_plane, frames = int(t_step),  repeat=False) #interval=1,

plt.show()


PlotCharts(graph_data, ['Time (s)', 'n (m)', 'e (m)', 'd (m)', 'u (m/s)', 'v (m/s)', 'w (m/s)', 'phi (rad)', 'theta (rad)', 'psi (rad)', 'p (rad)', 'q (rad)', 'r (rad)'])

# PlotTFStepResponse(Trim_Transfer_Functions, ['T_phi_delta_a', 'T_chi_phi','T_theta_delta_e', 'T_h_theta',  'T_h_Va', 'T_Va_delta_t', 'T_Va_theta', 'T_beta_delta_r'])

PlotTFStepResponse(AP_TFs, AP_TF_Names)