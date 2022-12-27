import copy
import math
import matplotlib.pyplot as plt
import numpy as np
import time

from heapq import heappop, heappush, heapify
from random import randint
from IPython.display import HTML
from PIL import Image, ImageDraw, ImageOps
from IPython.display import Image as Img
from IPython.display import display
from sys import float_info

EPS = float_info.epsilon


def make_path(goal):
    '''
    Creates a path by tracing parent pointers from the goal node to the start node
    It also returns path's length.
    '''
    length = goal.g
    g = goal.g
    current = goal
    path = []
    while current.parent:
        path.append(current)
        if current.g == g:
            current = current.parent
        g -= 1
    path.append(current)
    return path[::-1], length
    

def draw(grid_map, dyn_obst_traj, path, output_filename = 'animated_trajectories'):
    '''
    Auxiliary function that visualizes the environment.
    
    The function assumes that nodes_opened/nodes_expanded
    are iterable collestions of search nodes
    '''
    
    k = 30
    quality = 6
    height, width = grid_map.get_size()
    h_im = height * k
    w_im = width * k
    pathlen = len(path)
    
    step = 0
    images = []
    agent_color = randint(0, 255), randint(0, 255), randint(0, 255)
              
    while step < pathlen:
        for n in range(0, quality):
            im = Image.new('RGB', (w_im, h_im), color = 'white')
            draw = ImageDraw.Draw(im)
            
            # draw static obstacles
            for i in range(height):
                for j in range(width):
                    if(not grid_map.traversable(i, j)):
                        draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=( 70, 80, 80 ))
                   
            
            #draw agent
            curr_node = path[step]
            next_node = path[min(pathlen - 1, step + min(n, 1))]

            di = n * (next_node.i - curr_node.i) / quality
            dj = n * (next_node.j - curr_node.j) / quality

            draw.ellipse((float(curr_node.j + dj + 0.2) * k, 
                          float(curr_node.i + di + 0.2) * k, 
                          float(curr_node.j + dj + 0.8) * k - 1, 
                          float(curr_node.i + di + 0.8) * k - 1), 
                          fill=agent_color, width=0)
            
            # draw dynamic obstacles 
            for i in range(len(dyn_obst_traj)):
                curr_pos = dyn_obst_traj[i][min(len(dyn_obst_traj[i]) - 1, step)]
                next_pos = dyn_obst_traj[i][min(len(dyn_obst_traj[i]) - 1, step + min(n, 1))]
                
                di = n * (next_pos[0] - curr_pos[0]) / quality
                dj = n * (next_pos[1] - curr_pos[1]) / quality
            
                draw.rounded_rectangle((float(curr_pos[1] + dj + 0.2) * k, 
                              float(curr_pos[0] + di + 0.2) * k, 
                              float(curr_pos[1] + dj + 0.8) * k - 1, 
                              float(curr_pos[0] + di + 0.8) * k - 1), 
                              fill=(50, 50, 50), width=0, radius=k * 0.2)
            im = ImageOps.expand(im, border=2, fill='black')
            images.append(im)
        step += 1
    images[0].save('./'+output_filename+'.png', save_all=True, append_images=images[1:], optimize=False, duration=500/quality, loop=0)
    display(Img(filename = './'+output_filename+'.png'))
    
    
