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


EPS = float_info.epsilon


class Node:
    '''
    Node class represents a search node

    - i, j: coordinates of corresponding grid element
    - g: g-value of the node (also equals time moment when the agent reaches the cell)
    - h: h-value of the node // always 0 for Dijkstra
    - f: f-value of the node // always equal to g-value for Dijkstra
    - parent: pointer to the parent-node 

    '''
    
    def __init__(self, i, j, g = 0, h = 0, w = 1, f = None, parent = None, interval = -1):
        self.i = i
        self.j = j
        self.g = g
        self.h = h
        self.w = w
        self.interval = interval
        if f is None:
            self.f = self.g + self.h * self.w
        else:
            self.f = f        
        self.parent = parent

    
    def __eq__(self, other):
        '''
        Estimating where the two search nodes are the same,
        which is needed to detect dublicates in the search tree.
        '''
        return (self.i == other.i) and (self.j == other.j) and (self.interval == other.interval)
    
    
    def __hash__(self):
        '''
        To implement CLOSED as set of nodes we need Node to be hashable.
        '''
        ijg = self.i, self.j, self.interval
        return hash(ijg)


    def __lt__(self, other): 
        '''
        Comparison between self and other. Returns is self < other (self has higher priority).
        '''
        return self.f < other.f
    
    
from queue import PriorityQueue

class SearchTree: 
    def __init__(self):
        self._open = PriorityQueue()   
        self._open_size = 0
        self._closed = set()
        self._enc_open_dublicates = 0
      
    
    def __len__(self):
        return self._open_size + len(self._closed)
    
    
    def open_is_empty(self):
        return self._open_size == 0
    
    
    def add_to_open(self, item):        
        self._open.put(item) 
        self._open_size += 1
        return 
    
    
    def get_best_node_from_open(self):
        while not self.open_is_empty():
            best = self._open.get()
            self._open_size -= 1
            
            if not best in self._closed:
                return best
            
        return None
    
    
    def add_to_closed(self, item):
        self._closed.add(item)

        
    def was_expanded(self, item):
        return item in self._closed

    
    @property
    def OPEN(self):
        to_return = []
    
        while not self.open_is_empty():
            best = self._open.get()
            self._open_size -= 1
            to_return.append(best)
            
        return to_return
    
    
    @property
    def CLOSED(self):
        return self._closed



def sipp(safe_grid_map, 
         start_i, start_j, 
         goal_i, goal_j, 
         heuristic_func = None, 
         search_tree = None):
    
    '''
    Runs A* search algorithm without re-expansion on dynamic obstacles domain.

    Parameters
    ----------
    grid_map : Map
        An additional domain information (such as grid map).
    start_i, start_j : int, int
        Start cell
    goal_i, goal_j  : int, int
        Goal cell
    heuristic_func : function
        Heuristic function
    search_tree : type 
        Search tree data structure

    Returns
    -------
    path_found : bool
        Path was found or not.  
    last_node : Node
        The last node in path. None if path was not found.
    steps : int
        The number of search steps
    noodes_created : int
        The number of nodes, which were created and stored during the search process (size of the resultant search tree)
    open : iterable object
        Iterable collection of OPEN nodes
    expanded : iterable object
        Iterable collection of the expanded nodes
    '''

    ast = search_tree()
    steps = 0
    nodes_created = 0

    if not safe_grid_map.traversable(start_i, start_j, 0):
        Exception("Bad start:", start_i, start_j)
    
    start_node = Node(start_i, start_j, 
                      g=0, 
                      h=heuristic_func(start_i, start_j, goal_i, goal_j), 
                      interval=0)
    
    ast.add_to_open(start_node)
    nodes_created += 1
    
    while not ast.open_is_empty():
        steps += 1
        node = ast.get_best_node_from_open()
        if node is None:
            return (False, None, steps, nodes_created, ast.OPEN, ast.CLOSED)
        if node.i == goal_i and node.j == goal_j:
            return (True, node, steps, nodes_created, ast.OPEN, ast.CLOSED)
        
        neighbors = safe_grid_map.get_neighbors(node.i, node.j, node.g)
        for neighbor in neighbors:
            neighbor_node = Node(neighbor[0], neighbor[1], neighbor[2],
                                 interval=safe_grid_map.get_interval(neighbor[0], neighbor[1], neighbor[2]),
                                 h=heuristic_func(neighbor[0], neighbor[1], goal_i, goal_j), 
                                 parent = node)
            nodes_created += 1
            if not ast.was_expanded(neighbor_node):
                ast.add_to_open(neighbor_node)
                
        ast.add_to_closed(node)
        
    return False, None, steps, nodes_created, ast.OPEN, ast.CLOSED

    
