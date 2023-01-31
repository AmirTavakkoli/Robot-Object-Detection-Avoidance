import os
import math
import numpy as np
import pygame
from Robot import Graphics, Robot, Ultrasonic


clear = lambda: os.system('cls')  # On Windows System
clear()

# required inputs for map 
Mam_Dimensions = (650, 1000)
    
# creating the environment
gfx = Graphics(Mam_Dimensions, "Robot Model.png", "Map.png")
    
# required input for the robot
start = (200, 200)
robot = Robot(start, 0.01*3779.52)
    
# required input for ultrasonic sensor
sensor_range = 250, math.radians(40)
ultra_sonic = Ultrasonic(sensor_range, gfx.map)

# keep track of the time lapse beteen loop iterations
time_step = 0
last_time = pygame.time.get_ticks()
    
running = True
    
# simulation loop
while running: 
        
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            # if clicked quit teriminate the simulation
            running = False
        
    time_step = (pygame.time.get_ticks() - last_time)/1000
    last_time = pygame.time.get_ticks()
    
    # draw the map
    gfx.map.blit(gfx.map_img, (0, 0))

    # move the robot using the kinematics defined    
    robot.kinematics(time_step)   
    gfx.draw_robot(robot.x, robot.y, robot.heading)
    # finding obstacle    
    point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)
    # avoiding collision with obstacle    
    robot.avoid_obstacles(point_cloud, time_step)
    gfx.draw_sensor_data(point_cloud)
    #update the screen    
    pygame.display.update()