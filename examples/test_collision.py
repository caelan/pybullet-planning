#!/usr/bin/env python
# Basic RRT implementation to demonstrate collision avoidance functionality
# Nikolaus Correll, University of Colorado at Boulder
#
# pip3 install pybullet-planning
from __future__ import print_function

import random

import numpy as np
from pybullet_tools.utils import * 
import numpy as np

# RRT Parameters
Dq = 0.8
K=2000

RED = RGBA(1, 0, 0, 1)
TAN = RGBA(0.824, 0.706, 0.549, 1)

def main(floor_width=8.0):
    # Creates a pybullet world and a visualizer for it
    connect(use_gui=True)
    set_camera_pose(camera_point=[3, -3, 3], target_point=unit_point()) # Sets the camera's position
    
    # Bodies are described by an integer index
    floor = create_box(w=floor_width, l=floor_width, h=0.001, color=TAN) # Creates a tan box object for the floor
    set_point(floor, [0, 0, -0.001 / 2.]) # Sets the [x,y,z] translation of the floor

    obstacles=[]
    waypoints=[]

    with LockRenderer(): # Temporarily prevents the renderer from updating for improved loading efficiency
        with HideOutput(): # Temporarily suppresses pybullet output
            robot = load_model(TIAGO_URDF) # TURTLEBOT_URDF | ROOMBA_URDF # Loads a robot from a *.urdf file
            assign_link_colors(robot)
            robot_z = stable_z(robot, floor) # Returns the z offset required for robot to be placed on floor
            set_point(robot, [0, 0, robot_z]) # Sets the z position of the robot

            # Deploy random obstacles
            for _ in range(3):
                obstacles.append(create_box(w=2, l=2, h=0.1, color=RED)) # Creates a red box obstacle
                collision = True
                while collision:
                    collision=False
                    x = random.uniform(-floor_width/3., floor_width/3.)
                    y  = random.uniform(-floor_width/3., floor_width/3.)
                    yaw = random.uniform(-np.pi, np.pi)
                    set_point(obstacles[-1], [x, y, 0.1 / 2.]) # Sets the [x,y,z] position of the obstacle
                    set_euler(obstacles[-1], [0, 0, yaw]) #  Sets the [roll,pitch,yaw] orientation of the obstacle
                    for obstacle in obstacles[:-1]:
                        if collision == False:
                            collision = pairwise_collision(obstacles[-1], obstacle) # Checks whether robot is currently colliding with obstacle
           
            # Pick random start and goal
            for _ in range(2):
                waypoints.append(create_box(0.25,0.25,0.05,color=YELLOW))
                collision = True
                while collision:
                    collision=False
                    x = random.uniform(-floor_width/2., floor_width/2.)
                    y  = random.uniform(-floor_width/2., floor_width/2.)
                    set_point(waypoints[-1], [x, y, 0.05 / 2.]) # Sets the [x,y,z] position of the obstacle
                    for obstacle in obstacles:
                         if collision == False:
                            collision = pairwise_collision(waypoints[-1], obstacle) # Checks whether robot is currently colliding with obstacle
            qstart=tuple(get_point(waypoints[0])[0:2])
            qgoal=tuple(get_point(waypoints[1])[0:2])
   
    
                
    print(qstart,qgoal)
    set_all_static()

    # Joints are also described by an integer index
    # The tiago has explicit joints representing x, y, theta
    x_joint = joint_from_name(robot, 'x') # Looks up the robot joint named 'x'
    y_joint = joint_from_name(robot, 'y') # Looks up the robot joint named 'y'
    theta_joint = joint_from_name(robot, 'theta') # Looks up the robot joint named 'theta'
    
    # Helper function to be used within RRT to check whether a path from 
    # configuration a to configuration b is collision-free. 
    def openpath(a,b):
        steps=10
        (x,y) = (np.linspace(a[0],b[0],steps),np.linspace(a[1],b[1],steps))
    
        for i in range(len(x)):
            set_joint_position(robot, x_joint, x[i]) # Sets the current value of the x joint
            set_joint_position(robot, y_joint, y[i]) # Sets the current value of the y joint       
            set_joint_position(robot,theta_joint,np.arctan2(y[i]-a[1],x[i]-a[0])) # Makes the robot look toward the next waypoint
            for obstacle in obstacles:
                collision = pairwise_collision(robot, obstacle) # Checks whether robot is currently colliding with obstacle
                if(collision):
                    return False
        return True

    qnew=()
    G={qstart : []}
    k=0

    while qnew != qgoal and k<K:
        # a compute random configuration
        qrand = (random.uniform(-floor_width/2., floor_width/2.),random.uniform(-floor_width/2., floor_width/2.))
        if np.random.rand()<0.1:
            qrand=qgoal
        # b find nearest vertex to qrand that is already in g
        mindist = float("inf")
        for g in G:
            dist=np.sqrt((qrand[0]-g[0])**2+(qrand[1]-g[1])**2)
            if dist < mindist:
                qnear=g
                mindist=dist
        # c move Dq from qnear to qrand
        if mindist <= Dq:
            qnew = qrand
        else:
            qnew = (qnear[0]+(qrand[0]-qnear[0])/mindist*Dq,qnear[1]+(qrand[1]-qnear[1])/mindist*Dq)
        # d connect goal
        gdist=np.sqrt((qnew[0]-qgoal[0])**2+(qnew[1]-qgoal[1])**2)
        if gdist<Dq:
            qnew=qgoal

        # e if edge is collision-free, add qnew to G
        if openpath(qnear,qnew):
            with LockRenderer(): # Temporarily prevents the renderer from updating for improved loading efficiency
                with HideOutput(): # Temporarily suppresses pybullet output

                    G[qnew]=[(dist,qnear)] # Add node to tree
                    G[qnear].append((dist,qnew))  # Add edge to tree
            
                    # Draw marker
                    set_point(create_capsule(0.05, 0.02, color = GREEN),[qnew[0], qnew[1], 0.02/2.])

                    edge=create_cylinder(0.01,np.sqrt((qnew[0]-qnear[0])**2+(qnew[1]-qnear[1])**2))
                    set_point(edge,[qnew[0]+(qnear[0]-qnew[0])/2,qnew[1]+(qnear[1]-qnew[1])/2,0.02/2.])
                    set_euler(edge,[np.pi/2,0,np.arctan2(qnear[1]-qnew[1],qnear[0]-qnew[0])+np.pi/2])
                    k=k+1
        else:
            with LockRenderer(): # Temporarily prevents the renderer from updating for improved loading efficiency
                with HideOutput(): # Temporarily suppresses pybullet output
                    set_point(create_capsule(0.05, 0.05, color = YELLOW),[qnew[0], qnew[1], 0.02/2.])
                    edge=create_cylinder(0.01,np.sqrt((qnew[0]-qnear[0])**2+(qnew[1]-qnear[1])**2),color=RED)
                    set_point(edge,[qnew[0]+(qnear[0]-qnew[0])/2,qnew[1]+(qnear[1]-qnew[1])/2,0.02/2.])
                    set_euler(edge,[np.pi/2,0,np.arctan2(qnear[1]-qnew[1],qnear[0]-qnew[0])+np.pi/2])
                    qnew=[] # delete qnew to avoid to trigger the end of the while loop if qnew==qgoal                            
        wait_for_duration(0.1) # Like sleep() but also updates the viewer

    wait_if_gui() # Like raw_input() but also updates the viewer
    # Destroys the pybullet world
    disconnect()

if __name__ == '__main__':
    main()
