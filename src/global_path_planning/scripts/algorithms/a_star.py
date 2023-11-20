import numpy as np
import rospy

class Node:

    def __init__(self, pos):
        
        self.pos = pos
        #move cost
        self.g_cost   = 0
        #heuristic cost
        self.h_cost   = 0
        #total cost
        self.t_cost   = 0
        #parent
        self.parent = None

    def compute_h(self, g_pos):

        c_pos = np.array(self.pos, copy = False)
        g_pos = np.array(g_pos, copy = False)
        self.h = np.linalg.norm(g_pos - c_pos)
        self.update_t()
        
    def update_t(self):

        self.t_cost = self.g_cost + self.h_cost

class Astar():
    
    def __init__(self, static_map, map_h, map_w):
        
        #initialise open and closed list
        self.o_list = []
        self.c_list = []

        self.in_queue_check = {}
        
        #initialise static map        
        static_map_cpy = np.array(static_map, copy = True)
        static_map_cpy = np.reshape(static_map_cpy, (map_h, map_w))
        self.static_map = static_map_cpy

        #initialise cost map
        self.t_map = {}

        self.map_h = map_h
        self.map_w = map_w

        #initialise step cost
        self.step_cost = 1
        self.diag_step_cost = np.math.sqrt(2)

    def convert_to_1d_index(self, pos):
        '''
        args:
            pos: 2d index (col index, row index)
        '''
        
        return pos[1]*self.map_w + pos[0]
    
    def get_valid_neighbors(self, c_node, g_pos, move, step_cost):
        
        '''
        args:
            c_node: current node
            g_pos: goal position (x, y)
            move: defined movement
            step_cost: cost of 1 step
            
        return:
            neighbors: valid neigbors
        '''
        
        neighbors = []
        ostacle_th = 150

        for dx, dy in move:
            
            #convert back to tuple to ensure it is hashable
            n_pos = tuple(c_node.pos + np.array([dx, dy]))

            if 0 <= n_pos[0] < self.map_w and 0 <= n_pos[1] < self.map_h: 
                if self.static_map[n_pos[1], n_pos[0]] < ostacle_th:
                    n = Node(n_pos)
                    
                    #compute cost
                    next_move_cost = step_cost + self.static_map[n_pos[1], n_pos[0]]/255
                    n.g_cost  = c_node.g_cost + next_move_cost
                    n.compute_h(g_pos)
                        
                    neighbors.append(n)
                
        return neighbors
    
    def find_neighbors(self, c_node, g_pos):

        '''
        args:
            c_node: current node
            cost_map: 2d - array
            
        returns:
            neighbors: valid neighbors
        '''

        neighbors = []

        #handle up. down, right, left step
        move = ((1, 0), (-1, 0), (0, 1), (0, -1))
        neighbors += self.get_valid_neighbors(c_node, g_pos, move, step_cost = 1)
        
        #handle diagonal step
        move = ((1, 1), (-1, 1), (1, -1), (-1, -1))
        neighbors += self.get_valid_neighbors(c_node, g_pos, move, step_cost = np.math.sqrt(2))

        return neighbors

    def backtrack(self):
        
        '''
        returns
            best_path: shortest path to the goal
        '''
        
        n = self.g_node
        
        best_path = []
        while n is not None:
            best_path.append(self.convert_to_1d_index(n.pos))
            n = n.parent
        return best_path[::-1]
        
    def search_path(self, 
                    s_pos, 
                    g_pos, 
                    map_resol = None, 
                    grid_viz = None):

        '''
        args:
            s_pos: (x, y)
            g_pos: (x, y)

        returns:
            best_path: shortest path to the goal    
        '''    

        best_path = []

        is_path_found = False
	
        #initialise start node
        self.s_node = Node(pos = s_pos)
        self.s_node.compute_h(g_pos = g_pos)
        
        self.t_map[self.s_node.pos] = self.s_node.t_cost
        
        #put start node in queue
        self.o_list.append(self.s_node)
        self.in_queue_check[self.s_node.pos] = True
        #initialise goal node
        g_node = Node(pos = g_pos)
        
        while self.o_list:
            #sort queue
            self.o_list.sort(key = lambda x:x.t_cost)
            #get greedy best node
            c_node = self.o_list.pop(0)
            del self.in_queue_check[c_node.pos]
	    
            #add current node to closed list
            self.c_list.append(self.convert_to_1d_index(c_node.pos))
            #check if goal is reached 
            if c_node.pos[0] == g_pos[0] and c_node.pos[1] == g_pos[1]:
                self.g_node = c_node
                is_path_found = True
                break
            #find neighbors
            neigbors = self.find_neighbors(c_node, g_pos)
            for n in neigbors:
                
                #if explore already
                if self.convert_to_1d_index(n.pos) in self.c_list:
                    continue
                #check if the node is in the queue
                is_in_queue = False
                for ind, o_node in enumerate(self.o_list):
                    if n.pos[0] == o_node.pos[0] and n.pos[1] == o_node.pos[1]:
                        is_in_queue = True
                        break

                if is_in_queue:
                    if n.t_cost < self.t_map[n.pos]:
                        self.t_map[n.pos] = n.t_cost
                        n.parent = c_node

                        self.o_list[ind] = n
                else:
                    self.t_map[n.pos] = n.t_cost
                    n.parent = c_node
                    self.o_list.append(n)
                    self.in_queue_check[n.pos] = True

        if not is_path_found:
            best_path = []
            #rospy.loginfo("path not found")
        else:
            best_path = self.backtrack()    
            #rospy.loginfo("path found")
        
        return best_path
