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

from src.grid import Map, SafeMap, manhattan_distance
from src.algo.wsipp import wsipp


EPS = float_info.epsilon


def wsipp_transform(safe_grid_map, 
                    start_i, start_j, 
                    goal_i, goal_j, 
                    w_param = 1,
                    heuristic_func = None,
                    search_tree = None):
    path_found, last_node, iter_steps, nodes_created, opened, closed = wsipp(safe_grid_map, start_i, start_j, goal_i, goal_j, w_param, heuristic_func, search_tree)
    return path_found, last_node, iter_steps


def naive_arsipp(safe_grid_map, start_i, start_j, goal_i, goal_j, start_w = 3.0, step_w = 0.5, heuristic_func = None, search_tree = None):
    '''
    Repeatedly runs weighted A* search algorithm without re-expansion on any domain, 
    decreasing current weight from start_w to 1.0 by step_w.

    Parameters
    ----------
    grid_map : Map
        An additional domain information (such as grid map).
    start_i, start_j : int, int
        The start state of search in useful for your implementation form.
    goal_i, goal_j  : int, int
        The goal state of search in useful for your implementation form.
    start_w : float
        The initial weight of heuristics in F-value computation. Must be greater or equal to 1.0, by default 3.0.
    step_w : float
        The value by which the weight will be reduced, if it is provided by the algorithm, by default 0.5.

    Yields
    -------
    path_found : bool
        Path was found or not.  
    last_node : Node
        The last node in path. None if path was not found.
    steps : int
        The number of search steps
    weight : float
        Weight used at iteration
    '''

    if start_w < 1:
        raise Exception("Weight must be greater or equal to 1")

    weight = float(start_w)
    steps = 0

    while True:
        path_found, last_node, iter_steps = wsipp_transform(safe_grid_map, start_i, start_j, goal_i, goal_j, weight, heuristic_func, search_tree)
        steps += iter_steps
        yield path_found, last_node, steps, weight
        
        if abs(weight - 1.0) < EPS:
            break

        weight = (weight - step_w) if (weight - step_w) >= 1.0 else 1.0


