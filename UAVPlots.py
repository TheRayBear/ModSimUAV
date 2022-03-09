import numpy as np
import matplotlib.pyplot as plt
from RotationalMatricies import *
from stl import mesh
from mpl_toolkits import mplot3d


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
        PlotWindow.savefig('Mod Sim HW 6 Plots no Coupling - Ryan Raber')
        
        plt.show()


#Draw Plane
def DrawPlane(pos_ned, phi, theta, psi, throttle_pos):
    global northOffset                     # Translational offsets
    global eastOffset 
    global downOffset 
    global simScaleFactor
    global bodycolor
    global stlScaleFactor
    
    
    global axis1
    global axis2
    

    # Clear OLD_DATA
    axis1.clear()
    #axis2.clear()

    R = rotMat_body2inertial(phi, theta, psi)

    #axis1.plot(North_cor_x, North_cor_y, North_cor_z, 'r', lw = 2)
    #axis1.plot(East_cor_x, East_cor_y, East_cor_z, 'b', lw = 2)
    #axis1.plot(Down_cor_x, Down_cor_y, Down_cor_z, 'g', lw = 2)

    #axis1.text(0, 1, 0, 'N')
    #axis1.text(1, 0, 0, 'E')
    #axis1.text(0, 0, -1, 'D')

    axis1.set_xlim((-1, 1))
    axis1.set_ylim((-1, 1))
    axis1.set_zlim((-1, 1))
    axis1.set_title('UAV Import and Controller\nPress \'X\' to reset - Press \'B\' to close ')
    axis1.set_xlabel('East Axis (m)')
    axis1.set_ylabel('North Axis (m)')
    axis1.set_zlabel('Down Axis (m)')
    
    # axis2.set_title('Throttle Position')
    # axis2.set_ylabel('Percent')
    # axis2.bar(1, throttle_pos, color='red')
    # axis2.set_ylim(0,1.1)

    # Create Mesh from STL File

    planeMesh = mesh.Mesh.from_file("C:\\Users\\Ryan\\OneDrive - University of Cincinnati\\Model and Sim\\delorean.stl") # Include STL file name here
    planeMesh.rotate_using_matrix(R)
    planeMesh.x += simScaleFactor * (pos_ned[1] + eastOffset)
    planeMesh.y += simScaleFactor * (pos_ned[0] + northOffset)
    planeMesh.z -= simScaleFactor * (pos_ned[2] + downOffset)
    collection = mplot3d.art3d.Poly3DCollection(planeMesh.vectors * stlScaleFactor, edgecolor = 'k', lw = 0.1)   # Create collection
    planeColor = bodycolor                                                                                          # Set face colors (in RGB percentages i.e. [R, G, B])
    collection.set_facecolor(planeColor)
    axis1.add_collection3d(collection) # Add collection to plot
