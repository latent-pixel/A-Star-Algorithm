# -*- coding: utf-8 -*-
"""
Created on Mon Mar 21 19:27:44 2022

@author: Bhargav Kumar
"""

class Node():
    def __init__(self, state, parent_node, action, cost):
        self.state = state
        self.parent_node = parent_node
        self.action = action
        self.cost = cost
    
    def get_state(self):
        return self.state
    
    def get_parent(self):
        return self.parent_node
    
    def get_action(self):
        return self.action
    
    def get_cost(self):
        return self.cost
    
    def get_path(self):
        crnt_node = self
        action_set = list()
        path_nodes = list()
        while crnt_node.get_action() is not None:
            action_set.append(crnt_node.get_action())
            path_nodes.append(crnt_node)
            crnt_node = crnt_node.get_parent()
        
        action_set.reverse()
        path_nodes.reverse()
        return action_set, path_nodes
    
    # Rich comaparision, required for priority queue sorting
    def __lt__(self, other):
        return self.get_state() < other.get_state()