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


class Map:
    '''
    Square grid map class represents the environment for our moving agent
        - width -- the number of columns in the grid
        - height -- the number of rows in the grid
        - cells -- the binary matrix, that represents the grid. 0 - cell is traversable, 1 - cell is blocked
    '''

    def __init__(self):
        '''
        Default constructor
        '''
        self._width = 0
        self._height = 0
        self._cells = []
    

    def read_from_string(self, cell_str, width, height):
        '''
        Converting a string (with '#' representing obstacles and '.' representing free cells) to a grid

        Parameters
        ----------
        cell_str : str
            String which contains map data
        width : int
            Number of grid columns
        height : int
            Number of grid rows
        '''
        self._width = width
        self._height = height
        self._cells = [[0 for _ in range(width)] for _ in range(height)]
        cell_lines = cell_str.split("\n")
        i = 0
        j = 0
        for l in cell_lines:
            if len(l) != 0:
                j = 0
                for c in l:
                    if c == '.':
                        self._cells[i][j] = 0
                    elif c == '#' or c == 'T' or c == '@':
                        self._cells[i][j] = 1
                    else:
                        continue
                    j += 1
                if j != width:
                    raise Exception("Size Error. Map width = ", j, ", but must be", width )
                
                i += 1

        if i != height:
            raise Exception("Size Error. Map height = ", i, ", but must be", height )
    
    
    def read_from_file(self, path):
        '''
        Read file with grid (with '@', 'T', '#' representing obstacles and '.' representing free cells)
        '''
        map_file = open(path)
    
        map_file.readline()
        height = int(map_file.readline().split()[1])
        width = int(map_file.readline().split()[1])
        map_file.readline()
        
        if height > 70:
        	self._height = 70
        else:
        	self._height = height
        	
        if width > 70:
        	self._width = 70
        else:
        	self._width = width		
        
        self._cells = [[0 for _ in range(width)] for _ in range(height)]

        i = 0
        j = 0

        for l in map_file:
            j = 0
            for c in l:
                if c == '.':
                    self._cells[i][j] = 0
                elif c == '@' or c == 'T' or c == '#':
                    self._cells[i][j] = 1
                else:
                    continue
                j += 1
                
            if j != width:
                raise Exception("Size Error. Map width = ", j, ", but must be", width )

            i += 1

        if i != height:
            raise Exception("Size Error. Map height = ", i, ", but must be", height )
    

    def set_grid_cells(self, width, height, grid_cells):
        '''
        Initialization of map by list of cells.

        Parameters
         ----------
        width : int
            Number of grid columns
        height : int
            Number of grid rows
        grid_cells : list[list[int]]
            Map matrix consisting of values of two types: 0 (traversable cells) and 1 (obstacles)
        '''

        self._width = width
        self._height = height
        self._cells = grid_cells


    def in_bounds(self, i, j):
        '''
        Check if the cell is on a grid.

        Parameters
        ----------
        i : int
            The number of the column in which the cell is located
        j : int
            The number of the row in which the cell is located

        Returns
        -------
        bool
            Is the cell inside map bounds
        '''
        return (0 <= j < self._width) and (0 <= i < self._height)
    
    
    def traversable(self, i, j):
        '''
        Check if the cell is not an obstacle.

        Parameters
        ----------
        i : int
            The number of the column in which the cell is located
        j : int
            The number of the row in which the cell is located

        Returns
        -------
        bool
            Is the cell traversable (true) or obstacle (false)
        '''
        return not self._cells[i][j]


    def get_neighbors(self, i, j):
        '''
        Returns a list of neighbouring cells as (i, j) tuples. 
        Fucntions should returns such neighbours, that allows only cardinal moves, 
        but dissalows cutting corners and squezzing. 

        Parameters
        ----------
        i : int
            The number of the column in which the cell is located
        j : int
            The number of the row in which the cell is located

        Returns
        -------
        neighbours : list[tuple[int, int]]
            List of neighbours grid map (i, j) coordinates
        '''

        neighbors = []
        delta = [[0, 1], [1, 0], [0, -1], [-1, 0]]

        for d in delta:
            if self.in_bounds(i + d[0], j + d[1]) and self.traversable(i + d[0], j + d[1]):
                neighbors.append((i + d[0], j + d[1]))
                
        return neighbors

    
    def get_size(self):
        '''
        Returns the size of the map in cells

        Returns
        -------
        tuple[int, int]
            Size of the map in cells (height, width)
        '''
        return (self._height, self._width)



class SafeMap: # Map, but with safe intervals.
    
    def __init__(self, grid_map, dyn_obst_traj):       
        pos_time_table = dict()
        max_time_table = dict()
        
        for obst_id, obstacle in enumerate(dyn_obst_traj):
            for t, (i, j) in enumerate(obstacle):
                if not (i, j) in pos_time_table:
                    pos_time_table[(i, j)] = []
                d_i = 0 if t == len(obstacle) - 1 else obstacle[t + 1][0] - i 
                d_j = 0 if t == len(obstacle) - 1 else obstacle[t + 1][1] - j 
                pos_time_table[(i, j)].append((t, d_i, d_j))

            max_time_table[obstacle[-1]] = len(obstacle) - 1
        
        size = grid_map.get_size();
        self._height = size[0]
        self._width = size[1]
        self.intervals = [[[] for j in range(size[1])] for i in range(size[0])]
        for i in range(size[0]):
            for j in range(size[1]):
                if not grid_map.traversable(i, j):
                    continue
                old_t = -1
                out_moves = set()
                if (i, j) not in pos_time_table:
                    self.intervals[i][j].append((old_t, math.inf, out_moves))
                    continue
                pos_time_table[(i, j)].sort()
                for (t, d_i, d_j) in pos_time_table[(i, j)]:
                    if (i, j) in max_time_table and t > max_time_table[(i, j)]:
                        break
                    if t - old_t > 1:
                        self.intervals[i][j].append((old_t, t, out_moves))
                    if t != old_t:
                        out_moves = set()
                    old_t = t
                    if d_i != 0 or d_j != 0:
                        out_moves.add((d_i, d_j))
                
                if not (i, j) in max_time_table:
                    self.intervals[i][j].append((old_t, math.inf, out_moves))
        
        
    # Check if the cell is on a grid.    
    def in_bounds(self, i, j): 
        return (0 <= j < self._width) and (0 <= i < self._height)

    
    def get_interval(self, i, j, t):
        if not self.in_bounds(i, j) or len(self.intervals[i][j]) == 0 or self.intervals[i][j][-1][1] <= t:
            return -1
        
        l = -1
        r = len(self.intervals[i][j])
        while r - l > 1:
            m = (l + r) // 2
            if self.intervals[i][j][m][1] <= t:
                l = m
            else:
                r = m
          
        # if self.intervals[i][j][r][0] < t < self.intervals[i][j][r][1]:
        return r
    
    
    def traversable(self, i, j, t): # Check if the cell is not an obstacle.
        interval = self.get_interval(i, j, t)
        if interval == -1 or not self.intervals[i][j][interval][0] < t < self.intervals[i][j][interval][1]:
            return False
        return True

    
    def get_neighbors(self, i, j, t):
        '''
        Returns a list of neighbouring cells as (i, j) tuples. 
        Fucntions should returns such neighbours, that allows only cardinal moves, 
        but dissalows cutting corners and squezzing. 
        '''

        interval = self.get_interval(i, j, t)
        if interval == -1 or not self.intervals[i][j][interval][0] < t < self.intervals[i][j][interval][1]:
            raise Exception("How did you even get there:", i, j, t)
        
        t += 1
        f = self.intervals[i][j][interval][1]
        
        neighbors = []
        delta = [[0, 1], [1, 0], [0, -1], [-1, 0]]

        for d in delta:
            di = i + d[0]
            dj = j + d[1]
            if not self.in_bounds(di, dj):
                continue
                
            t_interval = self.get_interval(di, dj, t)
            if t_interval == -1:
                continue
            
            f_interval = self.get_interval(di, dj, f)
            if f_interval == -1:
                f_interval = len(self.intervals[di][dj]) - 1
            
            for interval in range(t_interval, f_interval + 1):
                t_in = max(t, self.intervals[di][dj][interval][0] + 1)
                if t_in == self.intervals[di][dj][interval][0] + 1 and t_in == f:
                    in_interval = self.get_interval(i, j, t_in)
                    if (-d[0], -d[1]) in self.intervals[di][dj][interval][2]:
                        t_in += 1
                if t_in > f or t_in >= self.intervals[di][dj][interval][1]:
                    continue
                
                neighbors.append((di, dj, t_in))
                
        return neighbors
    

    def get_size(self): # Returns the size of the map in cells
        return (self._height, self._width)
    


def manhattan_distance(i1, j1, i2, j2):
    '''
    Returns a manhattan distance between two cells

    Parameters
    ----------
    i1, j1 : int, int
        Position of first cell on the grid map
    i2, j2 : int, int
        Position of second cell on the grid map

    Returns
    -------
    int
        Manhattan distance between two cells

    '''
    return abs(i1 - i2) + abs(j1 - j2)
    

