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
from datetime import datetime
from tqdm import tqdm

from src.grid import Map, SafeMap, manhattan_distance
from src.utils import make_path, draw
from src.algo.astar_timesteps import astar_timesteps, SearchTree as SearchTreeAStarTimesteps
from src.algo.sipp import sipp, SearchTree as SearchTreeSIPP
from src.algo.wsipp_r import wsipp_r, SearchTree as SearchTreeWSIPPR
from src.algo.wsipp_d import wsipp_d, SearchTree as SearchTreeWSIPPD
from src.algo.naive_arsipp import naive_arsipp

EPS = float_info.epsilon


def generate_dynamic_obstacles_confs(count, height, width):    
    delta = np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])
    tasks = []
    
    for num in range(count):
        confs = []
        count_obs = 0
        if count < 10:
            count_obs = 3 * num
        else:
            count_obs = 7 * num
            
        for i_obs in range(count_obs):
            l = np.random.choice(range(3, 13))
            conf = []
            i = np.random.choice(height)
            j = np.random.choice(width)
            conf.append((i, j))
            deltas = delta[np.random.choice(4, size=l-1)]
            for pos in range(1, l):
                conf.append((conf[pos-1][0] + deltas[pos-1][0], conf[pos-1][1] + deltas[pos-1][1])) 

            conf = conf + conf[-2:0:-1]
            conf_res = []
            for _ in range(100):
                conf_res = conf_res + conf
            confs.append(conf_res)
        
        tasks.append(confs)
    
    return tasks


def launch_astar_timesteps(file_name, tasks_count, *args):
    np.random.seed(100)
    from src.algo.astar_timesteps import CATable, Node
        
    map_path = "maps/" + file_name + ".map"    
    grid = Map()
    grid.read_from_file(map_path)
    
    scens_path = "scens/" + file_name + ".map.scen"     
    scens_file = open(scens_path) 
    start_i, start_j, goal_i, goal_j = list(map(lambda i: int(i), scens_file.readline().split()))
        
    stat = dict()
    stat["wasFind"] = []
    stat["lenght"] = []
    stat["steps"] = []
    stat["nodesCreated"] = []
    stat["time"] = []
    
    tasks = generate_dynamic_obstacles_confs(tasks_count, grid.get_size()[0], grid.get_size()[1])
    
    for i, task in tqdm(enumerate(tasks)):
        try:   
            ca_table = CATable(task)
            
            start_time = datetime.now()
            result = astar_timesteps(grid, ca_table, start_i, start_j, goal_i, goal_j, *args)
            runtime = datetime.now() - start_time   
            
            stat["wasFind"].append(result[0])
            stat["steps"].append(result[2])
            stat["nodesCreated"].append(result[3])
            stat["time"].append(runtime)
            
            if result[0]:
                path = make_path(result[1]) 
                stat["lenght"].append(path[1])
                
                print("Path found! Length: " + str(path[1]) +\
                    ". Nodes created: " + str(result[3]) + \
                    ". Number of steps: " + str(result[2]) + \
                    ". Time: " + str(runtime))
                
            else:
                stat["lenght"].append(0.0)
                print("Path not found!")
            
        except Exception as e:
            print("Execution error")
            print(e)
        
    return stat


def launch_sipp(file_name, tasks_count, *args):
    from src.algo.astar_timesteps import CATable, Node
        
    map_path = "maps/" + file_name + ".map"    
    grid = Map()
    grid.read_from_file(map_path)
    
    scens_path = "scens/" + file_name + ".map.scen"     
    scens_file = open(scens_path) 
    start_i, start_j, goal_i, goal_j = list(map(lambda i: int(i), scens_file.readline().split()))
        
    stat = dict()
    stat["wasFind"] = []
    stat["lenght"] = []
    stat["steps"] = []
    stat["nodesCreated"] = []
    stat["time"] = []
    
    tasks = generate_dynamic_obstacles_confs(tasks_count, grid.get_size()[0], grid.get_size()[1])
    
    for i, task in tqdm(enumerate(tasks)):
        try:   
            safe_task_map = SafeMap(grid, task)
                        
            start_time = datetime.now()
            result = sipp(safe_task_map, start_i, start_j, goal_i, goal_j, *args)
            runtime = datetime.now() - start_time   
            
            stat["wasFind"].append(result[0])
            stat["steps"].append(result[2])
            stat["nodesCreated"].append(result[3])
            stat["time"].append(runtime)
            
            if result[0]:
                path = make_path(result[1]) 
                stat["lenght"].append(path[1])
            
                print("Path found! Length: " + str(path[1]) +\
                    ". Nodes created: " + str(result[3]) + \
                    ". Number of steps: " + str(result[2]) + \
                    ". Time: " + str(runtime))
                
            else:
                stat["lenght"].append(0.0)
                print("Path not found!")
            
        except Exception as e:
            print("Execution error")
            print(e)
        
    return stat


def launch_wsipp(file_name, search_fun, tasks_count, w, *args):
    from src.algo.astar_timesteps import CATable, Node
        
    map_path = "maps/" + file_name + ".map"    
    grid = Map()
    grid.read_from_file(map_path)
    
    scens_path = "scens/" + file_name + ".map.scen"     
    scens_file = open(scens_path) 
    start_i, start_j, goal_i, goal_j = list(map(lambda i: int(i), scens_file.readline().split()))
        
    stat = dict()
    stat["wasFind"] = []
    stat["lenght"] = []
    stat["coef"] = []
    stat["steps"] = []
    stat["nodesCreated"] = []
    stat["time"] = []
    
    tasks = generate_dynamic_obstacles_confs(tasks_count, grid.get_size()[0], grid.get_size()[1])
    
    for i, task in tqdm(enumerate(tasks)):
#         try:   
            safe_task_map = SafeMap(grid, task)
            
            expected = sipp(safe_task_map, start_i, start_j, goal_i, goal_j, manhattan_distance, SearchTreeSIPP)
            expected_len = make_path(expected[1])[1]
                
            start_time = datetime.now()
            result = search_fun(safe_task_map, start_i, start_j, goal_i, goal_j, w, *args)
            runtime = datetime.now() - start_time   
            
            stat["wasFind"].append(result[0])
            stat["steps"].append(result[2])
            stat["nodesCreated"].append(result[3])
            stat["time"].append(runtime)
            
            if result[0]:
                path = make_path(result[1]) 
                stat["lenght"].append(path[1])
            
                coef = path[1] / expected_len
                stat["coef"].append(coef)
                
                print("Path found! Length: " + str(path[1]) +\
                    ". Coefficient: " + str(coef) +\
                    ". Nodes created: " + str(result[3]) + \
                    ". Number of steps: " + str(result[2]) + \
                    ". Time: " + str(runtime))
                
            else:
                stat["lenght"].append(0.0)
                print("Path not found!")
            
#         except Exception as e:
#             print("Execution error")
#             print(e)
        
    return stat








