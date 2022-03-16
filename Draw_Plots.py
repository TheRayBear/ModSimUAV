from RotationalMatricies import *
from stl import mesh
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from RotationalMatricies import *
from control.matlab import step
import matplotlib.patches as patches

# General Sim Settings
stl_file_location='delorean.stl'
planeColor = [156/255, 156/255, 156/255]
stlScaleFactor = .05        # Scale plane and simulation parameters by equivalent amounts
simScaleFactor = 1 / stlScaleFactor

#Generate and Plot State Charts
def PlotCharts(data, key):
    data=np.array(data)
    if data.shape[1] != len(key):
        print('Labels Not Correct, Check and try again')
    else:
        PlotWindow = plt.figure(figsize = (15, 15))
        subplotHeight=int((data.shape[1]-1)/2)
        subplotWidth=2
        indexpos=np.concatenate((np.arange(0,data.shape[1]-1,2),np.arange(1, data.shape[1],2)))
        data=np.flipud(np.rot90(data, k=1, axes=(0,1)))
        for i in range(data.shape[0]-1):
            ax=PlotWindow.add_subplot(subplotHeight, subplotWidth, indexpos[i]+1)
            ax.plot(data[0],data[i+1])
            ax.set_xlabel(key[0])
            ax.set_ylabel(key[i+1])
        PlotWindow.tight_layout(pad=6)
        PlotWindow.suptitle('UAV State Graphs')
        PlotWindow.savefig('StatePlots.png')
        plt.show()

def PlotTFStepResponse(data, key):
    data=np.array(data)
    if len(data) != len(key):
        print('Labels Not Correct, Check and try again')
    else:
        PlotWindow = plt.figure(figsize = (15, 15))
        subplotHeight=int((data.shape[0])/2)
        subplotWidth=2
        for i in range(len(data)):
            y_out, t = step(data[i]) 

            ax=PlotWindow.add_subplot(subplotHeight, subplotWidth, i+1)
            ax.plot(t,y_out)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(key[i])
        PlotWindow.tight_layout(pad=6)
        PlotWindow.suptitle('UAV Transfer Function Step Responses')
        PlotWindow.savefig('TFPlots.png')
        plt.show()

def Draw_Throttle_Pos(axis, throttle_pos):
    throttleangle=-6*np.pi/4*throttle_pos+5*np.pi/4
    axis.set_title('Throttle')
    axis.plot(([0,throttleangle]), ([0,1]), 'k')
    axis.set_yticklabels([])
    axis.set_xticklabels([])
    axis.set_aspect('equal')

def Draw_Altimeter(axis, altitude):
    axis.set_title('Altitude')
    axis.plot([0], [altitude], '*', color='red')
    axis.set_ylim((altitude-5,altitude+5))

def Draw_Attitude(axis, states):
    phi=states[6]
    theta=states[7]
    psi=states[8]

    #Calculate Center Line Angle

    X1 = np.cos(phi)
    Y1 = np.sin(theta)-np.sin(phi)
    
    X0 = -np.cos(phi)
    Y0 = np.sin(theta)+np.sin(phi)

    X_center=0
    Y_center=(Y0+Y1)/2
    # background = patches.Rectangle((-1,-1),2,2, color='#a3cbef', fill=True)
    # ground = patches.Rectangle((X0-1, Y0-1), 3, 2, angle=phi, color='brown')

    axis.set_title('Attitude')
    # axis.add_patch(background)
    # axis.add_patch(ground)
    axis.plot(([-1,1]),([0,0]), 'gray')
    axis.plot(([X0,X1]),([Y0,Y1]), 'k')
    axis.scatter(X_center, Y_center ,s=150, color='red')
    axis.set_yticklabels([])
    axis.set_xticklabels([])
    axis.set_aspect('equal')
    axis.set_xlim((-1,1))
    axis.set_ylim((-1,1))


def Draw_Plane_STL(states, figure, throttle_pos, offsets = [0,0,0]):
    
    allaxis=figure.get_axes()
    axis1=allaxis[0]
    axis2=allaxis[1]
    axis3=allaxis[2]
    axis4=allaxis[3]

    #Unpack inputed variables
    northOffset = offsets[0]
    eastOffset = offsets[1]
    downOffset = offsets[2]
    pn=states[0]
    pe=states[1]
    pd=states[2]
    phi=states[7]
    theta=states[6]
    psi=states[8]


    # Clear OLD_DATA
    axis1.clear()
    axis2.clear()
    axis3.clear()
    axis4.clear()

    R = rotMat_body2inertial_STL(phi, theta, psi)

    plane_pos_x = (pe + eastOffset) #*simScaleFactor
    plane_pos_y = (pn + northOffset)#*simScaleFactor
    plane_pos_z = (pd + downOffset) #*simScaleFactor 

    #Plot Plane
    axis1.set_xlim((plane_pos_x-1, plane_pos_x+1))
    axis1.set_ylim((plane_pos_y-1, plane_pos_y+1))
    axis1.set_zlim((-plane_pos_z-1, -plane_pos_z+1))
    axis1.set_title('UAV Import and Controller\nPress \'X\' to reset - Press \'B\' to close ')
    axis1.set_xlabel('East Axis (m)')
    axis1.set_ylabel('North Axis (m)')
    axis1.set_zlabel('Down Axis (m)')

    #Plot Throttle Position and Altitude
    Draw_Throttle_Pos(axis2, throttle_pos)
    Draw_Altimeter(axis3, -plane_pos_z)
    Draw_Attitude(axis4, states)

    # Create Mesh from STL File
    planeMesh = mesh.Mesh.from_file("delorean.stl") # Include STL file name here
    planeMesh.rotate_using_matrix(R)
    planeMesh.x += simScaleFactor * plane_pos_x  
    planeMesh.y += simScaleFactor * plane_pos_y
    planeMesh.z -= simScaleFactor * plane_pos_z
    collection = mplot3d.art3d.Poly3DCollection(planeMesh.vectors * stlScaleFactor, edgecolor = 'k', lw = 0.1)   # Create collection
    collection.set_facecolor(planeColor)
    axis1.add_collection3d(collection)     # Add collection to plot
    return figure

def planeVerticesFaces():
    fuse_l1 = 2.8
    fuse_l2 = 0.1
    fuse_l3 = 5
    fuse_w = 1
    fuse_h = 1
    wing_l = 1.5
    wing_w = 7
    tail_l = 1
    tail_h = 1.5
    tailwing_w = 3.3
    tailwing_l = 0.75
    prop_l = 2
    prop_w = 0.1
    size = 0.2
    f = np.array([[ fuse_l1, 0, 0 ], #v1r - Front Fuselage Point
    [fuse_l2, fuse_w/2, -fuse_h/2], #v2r - Top Right Fuse Point
    [fuse_l2, -fuse_w/2, -fuse_h/2], #v3r - Top Left Fuse Point
    [fuse_l2, -fuse_w/2, fuse_h/2], #v4r - Bottom Left Fuse Point
    [fuse_l2, fuse_w/2, fuse_h/2], #v5r - Bottom Right Fuse Point
    [-fuse_l3, 0, 0], #v6r - Rear Fuse Point
    [0, wing_w/2, 0], #v7r - Front Right Wing Point
    [-wing_l, wing_w/2, 0],  #v8r  - Rear Right Wing Point
    [-wing_l, -wing_w/2, 0], #v9r - Rear Left Wing Point
    [0, -wing_w/2, 0], #v10r - Front Left Wing Point
    [tailwing_l - fuse_l3, tailwing_w/2, 0], #v11r - Front Right H_Stab Point
    [-fuse_l3, tailwing_w/2, 0], #v12r - Rear Right H_Stab Point
    [-fuse_l3, -tailwing_w/2, 0], #v13r - Rear Left H_Stab Point
    [-fuse_l3 + tailwing_l, -tailwing_w/2, 0], #v14r - Front Right H_Stab Point
    [-fuse_l3 + tailwing_l, 0, 0], #v15r - Front Bottom V_Stab Point
    [-fuse_l3 + (tailwing_l/2), 0, -tail_h], #v16r - Front Top V_Stab Point
    [-fuse_l3, 0, -tail_h], #v17r - Back Top V_Stab Point
    [fuse_l1, prop_l/2, prop_w/2], #v18r - Bottom Right Prop Point
    [fuse_l1, prop_l/2, -prop_w/2], #v19r - Top Right Prop Point
    [fuse_l1, -prop_l/2, -prop_w/2], #v20r - Top Left Prop Point
    [fuse_l1, -prop_l/2, prop_w/2]])  #v21r - Bottom Right Prop Point
    f = f * size
    return f

def Get_Plane_Wireframe(states):
    

    #Unpack inputed variables
    pn=states[0]
    pe=states[1]
    pd=states[2]
    phi=states[6]
    theta=states[7]
    psi=states[8]

    v=planeVerticesFaces()

    pos_ned=np.array([pn, pe, pd])
    # create m by n copies of pos_ned and used for translation
    ned_rep= np.tile(pos_ned, (21,1))
    R=rotMat_body2inertial(phi,theta,psi)
    #rotate 
    vr=np.matmul(R,v.T).T
    # translate
    vr=vr+ned_rep
    # rotate for plotting x=n, y=e, z=-pd
    R_plot=np.array([[0, 1, 0],
                    [1, 0, 0],
                    [0, 0, -1]])

    vr=np.matmul(R_plot,vr.T).T
    v1r=vr[0,:]
    v2r=vr[1,:]
    v3r=vr[2,:]
    v4r=vr[3,:]
    v5r=vr[4,:]
    v6r=vr[5,:]
    v7r=vr[6,:]
    v8r=vr[7,:]
    v9r=vr[8,:]
    v10r=vr[9,:]
    v11r=vr[10,:]
    v12r=vr[11,:]
    v13r=vr[12,:]
    v14r=vr[13,:]
    v15r=vr[14,:]
    v16r=vr[15,:]
    v17r=vr[16,:]
    v18r=vr[17,:]
    v19r=vr[18,:]
    v20r=vr[19,:]
    v21r=vr[20,:]

    # Define points to create faces
    fuse_x = [v1r[0], v2r[0], v3r[0],v1r[0], v4r[0], v5r[0], v6r[0], v4r[0], v3r[0], v6r[0], v2r[0],v5r[0],v1r[0]]
    fuse_y = [v1r[1], v2r[1], v3r[1],v1r[1], v4r[1], v5r[1], v6r[1], v4r[1], v3r[1], v6r[1], v2r[1],v5r[1],v1r[1]]
    fuse_z = [v1r[2], v2r[2], v3r[2],v1r[2], v4r[2], v5r[2], v6r[2], v4r[2], v3r[2], v6r[2], v2r[2],v5r[2],v1r[2]]
    fusecoords=[fuse_x,fuse_y, fuse_z]

    wing_x = [v7r[0], v8r[0], v9r[0], v10r[0],v7r[0]]
    wing_y = [v7r[1], v8r[1], v9r[1], v10r[1],v7r[1]]
    wing_z = [v7r[2], v8r[2], v9r[2], v10r[2],v7r[2]]
    wingcoords=[wing_x, wing_y, wing_z]

    hstab_x = [v11r[0], v12r[0], v13r[0], v14r[0], v11r[0]]
    hstab_y = [v11r[1], v12r[1], v13r[1], v14r[1], v11r[1]]
    hstab_z = [v11r[2], v12r[2], v13r[2], v14r[2], v11r[2]]
    hstabcoords = [hstab_x, hstab_y, hstab_z]

    vstab_x = [v6r[0], v15r[0], v16r[0], v17r[0],v6r[0]]
    vstab_y = [v6r[1], v15r[1], v16r[1], v17r[1],v6r[1]]
    vstab_z = [v6r[2], v15r[2], v16r[2], v17r[2],v6r[2]]
    vstabcoords = [vstab_x, vstab_y, vstab_z]

    prop_x = [ v18r[0], v19r[0], v20r[0], v21r[0], v18r[0]]
    prop_y = [ v18r[1], v19r[1], v20r[1], v21r[1], v18r[1]]
    prop_z = [ v18r[2], v19r[2], v20r[2], v21r[2], v18r[2]]
    propcoords = [prop_x, prop_y, prop_z]

    return (fusecoords, wingcoords, hstabcoords, vstabcoords, propcoords)