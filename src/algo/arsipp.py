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
    
    

class SearchTree:

    def __init__(self):
        self._open = []         # prioritized queue for the OPEN nodes
        self._closed= set()     # set for the expanded (at the current invocation of the main search loop) nodes = CLOSED
        self._incons = set()    # set of the inconsistnet nodes
        self._visited = dict()  # Dict of nodes (e.g. key = node, value = node) 
                                # this is needed to 'save' the search tree in between the search invocations
                                # this is also needed to obtain a g/f value of an arbitrary node in the search tree
            
            
    def __len__(self):
        return len(self._open) + len(self._closed) + len(self._incons)
         
        
    def open_is_empty(self):
        return len(self._open) == 0
 

    def add_to_open(self, item):
        '''
        Adding a node to the search-tree (i.e. to OPEN).
        As we can obtain a g-value of an arbitrary node, it is reasonable to check wheter the added node has
        a better g-value. If it is, then we add to OPEN. If it is not, then no need to add.
        Please node, that in such an approach on its own still does not guarantee that OPEN has no dublicates
        (i.e. nodes with the same id, but different g-values/parents/etc.) because if there is a node in OPEN 
        with the worse g-value we can opt to add a new node and to keep the old one and discard the latter lazily.

        Parameters
        ----------
        item : Node
            Node which should be added to OPEN
        '''
        heappush(self._open, item)
        self._visited[(item.i, item.j)] = item
    

    def get_best_node_from_open(self):
        '''
        Extracting the best node (i.e. the one with the minimal key) from OPEN.
        This node will be expanded further on in the main loop of the search.
        '''
        while not self.open_is_empty():
            best = heappop(self._open)
            if not best in self._closed:
                return best
            
        return None

    
    def add_to_closed(self, item):
        self._closed.add(item)

    
    def was_expanded(self, item):
        return item in self._closed

    
    def add_to_incons(self, item):
        '''
        This is needed only for ARA*.
        If we generated a node that already was expanded BUT the generated node has a better g-value
        we need not to add this generated node to INCONS. This node will not be used to continue
        growing the search tree at the current iteration of the search main loop but will be used as the
        frontier node at the next iteration of the search main loop in ARA*.
        
        Parameters
        ----------
        item : Node
            Node to insert
        '''
        self._incons.add(item)


    def unite_update_open_incons(self, new_weight):
        '''
        This is needed only for ARA*.
        Unites OPEN and INCONS sets and updates f-values of nodes using new suboptimality bound.

        Parameters
        ----------
        new_weight : float
            New weigth for updating f-values of nodes
        '''
        for n in self._open:
            n.f = n.g + n.h * new_weight
                    
        for n in self._incons:
            n.f = n.g + n.h * new_weight

        self._open = self._open + list(self._incons)
        heapify(self._open)

        
    def clear_closed(self):
        '''
        This is needed only for ARA*.
        Closed should be freshed before each termination of the search main loop.
        The 'closed part' of the search tree will still be saved in memory in between the search main loops
        in the VISITED container.
        '''   
        self._closed.clear()

        
    def clear_incons(self):
        '''
        This is needed only for ARA*.
        Incons should be freshed before each termination of the search main loop.
        ''' 
        self._incons.clear()
    
    
    def get_node(self, i, j):
        '''
        Checks if the node was generated (belongs to the search tree). If it is, then we return this node,
        more precisely -- the version of this node (as we might have dublicates) with the best g-value.
        If it is not, then a 'bulk' node with the 'infinite' g-value is returned.
        ''' 
        if (i, j) in self._visited:
            return self._visited[(i, j)]
        else:
            return Node(i, j, math.inf)

        
    @property
    def OPEN(self):
        return self._open
    
    
    @property
    def CLOSED(self):
        return self._closed 


