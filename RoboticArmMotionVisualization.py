import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d # Package for 3D plotting
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Rectangle

class robot_arm():
    x_init = [1.5,0,0.1]
    x_product = [1,1.5]

    
    l1 = 1

    security = 0.005
    
    obstacle_center = np.array([0.0, 1.5])  # center of cylindrical obstacle
    obstacle_radius = 0.2  # radius of cylindrical obstacle
    obstacle_height = 0.2  # height of cylindrical obstacle

    # define path steps
    if x_product[0] > 0 :
        Xn = np.array([x_init,[x_init[0],x_init[1],1], [x_product[0], x_product[1],1], [x_product[0],x_product[1],0.1]])
    if x_product[0] <= 0 and x_product[1] <= 1.5:
        Xn = np.array([x_init,[0,1.5,1],[0,1.5-(0.2+0.005+0.001), 1],[x_product[0],x_product[1],1],  [x_product[0],x_product[1],0.1]])
    if x_product[0] < 0 and x_product[1] > 1.5 :
        Xn = np.array([x_init,[0,1.5,1],[0,1.5+(0.2+0.005+0.001), 1],[x_product[0],x_product[1],1],  [x_product[0],x_product[1],0.1]])

    lim_angle = np.array((-np.pi, np.pi))
    lim_length_d2 = np.array((1, 2))
    lim_length_d3 = np.array((0, 0.9))

    # ---------------------------------------------------------------------------------------------------------
    def __init__(self):
        pass
    # ---------------------------------------------------------------------------------------------------------
    # Direct kinematics function - receives the angles of the robot and returns the position of the end-effector
    def direct_kinematics(self, q):
        x = q[1]*np.cos(q[0])
        y = q[1]*np.sin(q[0])
        z = self.l1-q[2]
    
        return np.array([x, y, z]) # Position of end-effector
    
    
    def inv_kinematics(self, X):         
        tetha1 = np.arctan2(X[1], X[0])
        d2 = ((X[0]**(2))+(X[1]**(2)))**(0.5)
        d3 = 0.9
        values = np.array([tetha1,d2,d3])
        return values

    # ---------------------------------------------------------------------------------------------------------
    # Checks if the end-effector is in collision with the obstacle, returns: True - in collision, False - no collision
    def collision_checker(self,q):
        x = self.direct_kinematics(q)
        radius = (self.x_product[0]**2+self.x_product[1]**2)**(0.5)

        distance = np.linalg.norm(x[:2] - self.obstacle_center[:2])
        if distance < self.obstacle_radius+self.security and 0 < x[2] < self.obstacle_height+self.security:
            return True
        if -2< self.x_product[0] <-1 and self.x_product[1]<self.security:
            raise Exception("Error : the final position is to close to the workspace limit")
        if 1 <= radius <= 1+ self.security :
            raise Exception("Error : the final position is to close to the workspace limit")
        if 2 >= radius >= 2- self.security :
            raise Exception("Error : the final position is to close to the workspace limit")
        if radius > 2 or radius < 1 :
            raise Exception("Error : the final position is outside the workspace")
        
        # Check if angle & lengths in limit
        if (x[0] < self.lim_angle[0]) or (x[0] > self.lim_angle[1]):
            return True 
        if (x[1] < self.lim_length_d2[0]) or (x[1] > self.lim_length_d2[1]):
            return True
        if (x[2] < self.lim_length_d3[0]) or (x[2]> self.lim_length_d3[1]):
            return True
        
        
        
        return False # No collsion

    # ---------------------------------------------------------------------------------------------------------

    def theta_pol(self, X, T=1):
        Tt = np.linspace(0, T, 100)
        Q = []
        for i in range(1, len(X)):
            q1 = self.inv_kinematics(X[i-1])             
            q2 = self.inv_kinematics(X[i])

            a1 = np.zeros((3,))             
            a0 = q1            
            a3 = (q2 - q1) / (T**3 - 1.5*T**4)             
            a2 = -1.5 * a3 * T**2              
            [Q.append(a0 + a1*t + a2 * t**2 + a3 * t**3) for t in Tt]
        Q=np.array(Q)

        X = np.array([self.direct_kinematics(q) for q in Q])
        return Q, X
    
    #------------------------------------
    # Plot the robot in 3D

    def plot_3D(self,location):

        # X=np.array([0,0,0],[0,0,self.l1],######################)
        # ax.plot3D(X[:,0],X[:,1],X[:,2],"-ok", markersize = 10)
        # xyz = self.direct_kinematics(self, q)
        # ax.scatter3D(xyw[0], xyz[1],xyz[2], color = "gold")
        # ax.scatter3D(0,0,0, color = "limegreen")

        fig = plt.figure()
        ax = plt.axes(projection = "3d")

        a= location
        Q ,X = self.theta_pol(a, 1)
        ax.plot3D(X[:,0],X[:,1],X[:,2])

        red = (1, 0, 0, 1)
        grey = (0.5, 0.5, 0.5, 1)
        z_cylinder = np.linspace(0,0.2,100)
        z_boudaries = np.linspace(0,0.05,100)

        r_cylinder = 0.2
        tetha_cylinder = np.linspace(0,2*np.pi, 100)
        X_cylinder = r_cylinder*np.cos(tetha_cylinder)
        Y_cylinder = r_cylinder*np.sin(tetha_cylinder) + 1.5
        a_cylinder,Z_cylinder = np.meshgrid(tetha_cylinder,z_cylinder)

        r_int = 1
        tetha_int = np.linspace(0,np.pi, 100)
        X_int = r_int*np.cos(tetha_int)
        Y_int = r_int*np.sin(tetha_int)
        a_int,Z_int = np.meshgrid(tetha_int,z_boudaries)

        r_ext = 2
        tetha_ext = tetha_int
        X_ext = r_ext*np.cos(tetha_int)
        Y_ext = r_ext*np.sin(tetha_int)
        a_ext,Z_ext = np.meshgrid(tetha_ext,z_boudaries)

        # Define the rectangle vertices
        left_rect_x = np.array([-2, -2, -1, -1])
        left_rect_y = np.array([0, 0, 0, 0])
        left_rect_z = np.array([0, 0.05, 0.05, 0])
        left_rectangle_verts = np.vstack((left_rect_x, left_rect_y, left_rect_z)).T

        right_rect_x = np.array([1, 1, 2, 2])
        right_rect_y = np.array([0, 0, 0, 0])
        right_rect_z = np.array([0, 0.05, 0.05, 0])
        right_rectangle_verts = np.vstack((right_rect_x, right_rect_y, right_rect_z)).T

        # Define the rectangle faces
        left_rectangle_faces = [[0, 1, 2], [0, 1, 3]]
        right_rectangle_faces = [[0, 1, 2], [0, 1, 3]]

        # Create the rectangle
        left_rectangle = Poly3DCollection([left_rectangle_verts[face] for face in left_rectangle_faces], alpha=0.5)
        left_rectangle.set_facecolor('grey')
        right_rectangle = Poly3DCollection([right_rectangle_verts[face] for face in right_rectangle_faces], alpha=0.5)
        right_rectangle.set_facecolor('grey')

        # Add the rectangle to the plot
        ax.add_collection3d(right_rectangle)
        ax.add_collection3d(left_rectangle)

        ax.plot_surface(X_cylinder,Y_cylinder,Z_cylinder, color = red)
        ax.plot_surface(X_int,Y_int,Z_int, color = grey)
        ax.plot_surface(X_ext,Y_ext,Z_ext, color = grey)
        #ax.scatter3D(location[0],location[1],location[2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        # ax.set_xlim([-2, 2])
        # ax.set_ylim([-2, 2])
        # ax.set_zlim([0, 0.25])
        ax.set_axis_on()
        ax.grid(True)
        plt.show()

        


# ==============================================
#G = robot_arm() # Generate robot
robot = robot_arm()

if robot.x_product[0] > 0 :
    Xn = np.array([robot.x_init,[robot.x_init[0],robot.x_init[1],1], [robot.x_product[0], robot.x_product[1],1], [robot.x_product[0],robot.x_product[1],0.1]])
if robot.x_product[0] <= 0 and robot.x_product[1] <= 1.5:
    Xn = np.array([robot.x_init,[robot.x_init[0],robot.x_init[1],1],[0,1.5-(0.2+0.005+0.001), 1],[robot.x_product[0],robot.x_product[1],1],  [robot.x_product[0],robot.x_product[1],0.1]])
if robot.x_product[0] < 0 and robot.x_product[1] > 1.5 :
    Xn = np.array([robot.x_init,[robot.x_init[0],robot.x_init[1],1],[0,1.5+(0.2+0.005+0.001), 1],[robot.x_product[0],robot.x_product[1],1],  [robot.x_product[0],robot.x_product[1],0.1]])


final_position = [robot.x_product[0],robot.x_product[1], 0.1]
tetha1_d2_d3 = robot.inv_kinematics(final_position)
if robot.collision_checker(tetha1_d2_d3):
    print("Warning: End-effector in collision!!!")
else:
    print("No collision.")

robot.plot_3D(Xn)