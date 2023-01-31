# import pygame module
import  pygame
  
import os
import math
import numpy as np
import pygame
from Robot import Graphics, Robot, Ultrasonic


clear = lambda: os.system('cls')  # On Windows System
clear()

pygame.init()
  
# width
width = 1400
  
# height
height = 700
  
#store he screen size
z = [width,height]
  
# store the color
white = (255, 255, 255)
screen_display = pygame.display
  
# Set caption of screen
screen_display.set_caption('GEEKSFORGEEKS')
  
# setting the size of the window
surface = screen_display.set_mode(z)
  
# set the image which to be displayed on screen
python = pygame.image.load('Map.png')
  
# set window true
window = True
while window:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            window = False
              
            # display white on screen other than image
    surface.fill(white)
      
# draw on image onto another
    surface.blit(python,(50, 50))
    screen_display.update()
  
pygame.quit()