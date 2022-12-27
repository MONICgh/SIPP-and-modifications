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

from src.grid import Map, manhattan_distance


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
    
    def __init__(self, i, j, g = 0, h = 0, f = None, parent = None):
        self.i = i
        self.j = j
        self.g = g
        self.h = h
        if f is None:
            self.f = self.g + h
        else:
            self.f = f        
        self.parent = parent

        
    def __eq__(self, other):
        '''
        Estimating where the two search nodes are the same,
        which is needed to detect dublicates in the search tree.
        '''
        return (self.i == other.i) and (self.j == other.j) and (self.g == other.g)
    
    
    def __hash__(self):
        '''
        To implement CLOSED as set of nodes we need Node to be hashable.
        '''
        ijg = self.i, self.j, self.g
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

    
    @property
    def number_of_open_dublicates(self):
        return self._enc_open_dublicates

 
        
def compute_cost(i1, j1, i2, j2):
    '''
    Computes cost of simple moves

    Parameters
    ----------
    i1, j1 : int, int
        Position of first cell on the grid map
    i2, j2 : int, int
        Position of second cell on the grid map

    Returns
    -------
    int
        Cost of the move between two neighbouring cells

    '''

    d = abs(i1 - i2) + abs(j1 - j2)
    if d == 0:  # wait
        return 1
    elif d == 1:  # cardinal move
        return 1
    else:
        raise Exception('Trying to compute the cost of non-supported move!')
        
        
class CATable:
    '''
    Class, which implements collision avoidance table for effective checking collisions with dynamic obstacles
    '''
    def __init__(self, dyn_obst_traj):       
        self.pos_time_table = dict()
        self.max_time_table = dict()
        
        for obst_id, obstacle in enumerate(dyn_obst_traj):
            for t, (i, j) in enumerate(obstacle):
                self.pos_time_table[(i, j, t)] = obst_id
            
            self.max_time_table[obstacle[-1]] = len(obstacle) - 1 
            
            
    def __check_pos_at_time(self, i, j, t):
        '''
        Checks, that cell (i, j) is occupied at moment t
        
        Parameters
        ----------
        i, j: int, int
            Position of cell on the grid map
        t : int
             Time moment to check
            
        Returns
        -------
        bool
            False, if cell is occupied at time moment t
            True, if not occupied at time moment t
        '''
        return not ((i, j, t) in self.pos_time_table)
           
        
    def __check_rev_move(self, i1, j1, i2, j2, t_start):
        '''
        Checks, that the given move does not result in edge collision
        
        Parameters
        ----------
        i1, j1 : int, int
            Start cell of the move
        i2, j2 : int, int
            Target cell of the move
        t_start : int
             Time when the move starts
            
        Returns
        -------
        bool        
            True if the given move does not result in the edge collision
            False if the given move does results in the edge collision
        '''
        return not ((i2, j2, t_start) in self.pos_time_table and (i1, j1, t_start + 1) in self.pos_time_table)

    
    def check_move(self, i1, j1, i2, j2, t_start):
        '''
        Checks if the move between (i1, j1) and (i2, j2) 
        at moment (t_start -> t_start+1) leads 
        to the collision with a dynamic obstacle.

        Parameters
        ----------
        i1, j1 : int, int
            Start cell of the move
        i2, j2 : int, int
            Target cell of the move
        t_start : int
             Time step when the move starts
            
        Returns
        -------
        bool
            Is the move valid (true) or will it lead to a collision (false)
        '''
        
        return self.__check_rev_move(i1, j1, i2, j2, t_start) and self.__check_pos_at_time(i2, j2, t_start + 1)
    
    
def get_neighbors_wrt_time(i, j, t, grid_map, ca_table):
    '''
    Returns a list of neighbouring cells as (i, j) tuples. 
    
    Should return such neighbours, that result
    from non-colliding actions (cardinal moves and wait) w.r.t.
    the given time step.
    
    I.e. the succesor cell should be free at the next time step,
    as well at the move should not result in the edge collision.
    
    Parameters
    ----------
    i, j : int
        Cell coordinates
    grid_map : Map
        An additional domain information (such as grid map).
    ca_table : CATable
        Collision avoidance table

    Returns
    -------
    neighbours : list[tuple[int, int]]
        List of neighbours grid map (i, j) coordinates
    '''

    neighbors = grid_map.get_neighbors(i, j)
    neighbors.append((i, j))
    result = []

    for (i_n, j_n) in neighbors:
        if ca_table.check_move(i, j, i_n, j_n, t):
            result.append((i_n, j_n))

    return result


def astar_timesteps(grid_map, ca_table, start_i, start_j, goal_i, goal_j, heuristic_func = None, search_tree = None):
    '''
    Runs A* search algorithm without re-expansion on dynamic obstacles domain.

    Parameters
    ----------
    grid_map : Map
        An additional domain information (such as grid map).
    ca_table : CATable
        Collision avoidance table
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

    start_node = Node(start_i, start_j, g=0, h=heuristic_func(start_i, start_j, goal_i, goal_j))

    ast.add_to_open(start_node)
    nodes_created += 1
        
    while not ast.open_is_empty():
        steps += 1
        node = ast.get_best_node_from_open()
                
        if node is None:
            return (False, None, steps, nodes_created, ast.OPEN, ast.CLOSED)
        if node.i == goal_i and node.j == goal_j:
            return (True, node, steps, nodes_created, ast.OPEN, ast.CLOSED)
        
        successors = list(map(
            lambda neighbor: Node(
                i=neighbor[0], j=neighbor[1], 
                g=node.g + compute_cost(node.i, node.j, neighbor[0], neighbor[1]),
                h=heuristic_func(neighbor[0], neighbor[1], goal_i, goal_j),
                parent=node
            ),
            get_neighbors_wrt_time(node.i, node.j, node.g, grid_map, ca_table)
        ))
    
        for _, successor in enumerate(successors):
            nodes_created += 1
            
            if not ast.was_expanded(successor):
                ast.add_to_open(successor)
                
        ast.add_to_closed(node)
    
    return (False, None, steps, nodes_created, ast.OPEN, ast.CLOSED)
 
