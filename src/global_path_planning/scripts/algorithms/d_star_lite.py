import rospy
import numpy as np

class Node:
    
    def __init__(self, pos):
        
        self.pos = pos
        #move cost
        self.g_cost = np.inf
        #parent
        self.parent = None
        #key
        self.key = None
        

class d_star_lite():
    
    def __init__(self):
        pass
        
    def reset(self, s_pos, g_pos, c_static_map, map_w, map_h):
        
        #initialise s_pos and g_pos
        self.s_pos = s_pos
        self.g_pos = g_pos
        
        #initialise open and closed list
        self.o_list = []
        
        #initalise km variable (help differentiate keys of multiple paths)
        self.km = 0
        
        #initialise static cost map (indicate if it is empty of have any obstacle)
        self.c_static_map = c_static_map
        self.p_static_map = None
        
        self.map_w = map_w
        self.map_h = map_h
        
        #move cost history map (cost of start_node => current node)
        self.g_map = {}
        
        #updated move cost map
        self.rhs_map = {}
        
        #set the history map at start and goal position to np.inf
        #used for trigger update
        self.g_map[g_pos] = np.inf
        self.g_map[s_pos] = np.inf
        
        #set the updated map at start and goal position to np.inf and 0
        #used for trigger update
        self.rhs_map[g_pos] = 0
        self.rhs_map[s_pos] = np.inf 
        
        #append goal node in the queue
        g_node = Node(self.g_pos)
        g_node.key = self.compute_key(self.g_pos)
        self.o_list.append(g_node)
        
    def compute_dist(self, s_pos, g_pos):
        s = np.array(s_pos, copy = True)
        g = np.array(g_pos, copy = True)
        
        d = np.linalg.norm(g - s)
        
        return d
    
    def compute_key(self, c_pos):
        
        #k1 = min(cost of current node => goal)
        k1 = min(self.g_map[c_pos], self.rhs_map[c_pos])
        #k2 = k1 + dist of start node => current node
        k2 = k1 + self.compute_dist(self.s_pos, c_pos)

        return (k2, k1)
    
    def update_queue(self, c_node):
        
        #mark unknown node
        if c_node.pos not in self.g_map:
            self.g_map[c_node.pos] = np.inf
            
        #mark unknown node
        if c_node.pos not in self.rhs_map:
            self.rhs_map[c_node.pos] = np.inf
        
        #check if the current node is in queue
        ind = None
        is_in_queue = False
        for i, n in enumerate(self.o_list):
            if n.pos[0] == c_node.pos[0] and n.pos[1] == c_node.pos[1]:
                is_in_queue = True
                ind = i
                break
        
        if is_in_queue:
            if self.g_map[c_node.pos] != self.rhs_map[c_node.pos]:
                #in queue but this node has not been updated 
                self.o_list[ind].key = self.compute_key(c_node.pos)
            else:
                #in queue and is the most updated => no need to explore again
                self.o_list.pop(ind)
        elif not is_in_queue and self.g_map[c_node.pos] != self.rhs_map[c_node.pos]:
            
            #not in queue but has not been updated
            c_node.key = self.compute_key(c_node.pos)
            self.o_list.append(c_node)
    
    def get_valid_neighbors(self, c_node, move, step_cost):
        
        '''
        args:
            c_node: current node
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
                #generate new node
                n = Node(n_pos)
                
                #compute next move cost
                if  self.c_static_map[n_pos[1], n_pos[0]] < ostacle_th:
                    next_move_cost = step_cost + self.c_static_map[n_pos[1], n_pos[0]]/255
                else:
                    next_move_cost = np.inf
                
                '''
                c_node.g_cost = cost of current node to goal 
                next_move_cost = cost of neighbor node to current node
                '''
                n.g_cost = self.g_map[c_node.pos] + next_move_cost
                neighbors.append(n)
                    
        return neighbors
    
    def find_neighbors(self, c_node):

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
        neighbors += self.get_valid_neighbors(c_node, move, step_cost = 1)
        
        #handle diagonal step
        move = ((1, 1), (-1, 1), (1, -1), (-1, -1))
        neighbors += self.get_valid_neighbors(c_node, move, step_cost = np.math.sqrt(2))

        return neighbors
    
    def update_cost_map(self):
        
        self.o_list.sort(key = lambda x:x.key[0])
        
        '''
            self.rhs_map[self.s_pos] != self.g_map[self.s_pos] 
                => update process is incomplete
            self.o_list[0].key < self.compute_key(self.s_pos)  
                => more efficient path is found
        '''
        while self.o_list[0].key < self.compute_key(self.s_pos) or self.rhs_map[self.s_pos] != self.g_map[self.s_pos]:
            
            c_node  = self.o_list.pop(0)
            c_pos   = c_node.pos
            old_key = c_node.key
            new_key = self.compute_key(c_pos)
            
            if c_pos not in self.g_map:
                self.g_map[c_pos] = np.inf
                
            if c_pos not in self.rhs_map:
                self.rhs_map[c_pos] = np.inf
            
            if old_key < new_key:
                '''
                    old_key < new_key
                        => old cost < new cost
                        => the map structure has changed and cause k1 or k2 becomes larger
                        => larger k1 or k2 => larger moving distance
                        => need to explore 
                        => no pop
                '''
                c_node.key = new_key
                self.o_list.push(c_node)
            elif self.g_map[c_pos] > self.rhs_map[c_pos]:
                '''
                    self.g_map[c_pos] > self.rhs_map[c_pos]
                        => updated cost map have a more efficient path (current node => target)
                        => need to update current node's neighbors
                '''
                self.g_map[c_pos] = self.rhs_map[c_pos]
                neighbors = self.find_neighbors(c_node)
                
                for n in neighbors:
                    if n.pos[0] != self.g_pos[0] and n.pos[1] != self.g_pos[1]:
                        #update rhs_map
                        if n.pos in self.rhs_map:
                            self.rhs_map[n.pos] = min(self.rhs_map[n.pos], n.g_cost)
                        else:
                            self.rhs_map[n.pos] = min(np.inf, n.g_cost)
                    #check if the key value of queue has changed             
                    self.update_queue(n)
            else:
                '''
                    not(old_key < new_key) and not(self.g_map[c_pos] > self.rhs_map[c_pos])
                        => old cost >= new cost
                        => but self.g_map[c_pos] <= self.rhs_map[c_pos] = old(k1) <= new(k1)
                        => so, old cost(start node => current node) > new cost(start node => current node)
                        => need to update the cost of start to current node
                '''
                self.g_map[c_pos] = np.inf
                self.update_queue(c_node)
                
                neighbors = self.find_neighbors(c_node)
                for n in neighbors:
                    self.update_queue(n)
                
            #sort queue
            self.o_list.sort(key = lambda x:x.key[0])
        
    def convert_to_1d_index(self, pos):
        '''
        args:
            pos: 2d index (col index, row index)
        '''
        
        return pos[1]*self.map_w + pos[0] 
        
    def get_shortest_path(self):
        
        self.path = []
        c_pos = self.s_pos

        move1 = [(1, 0), (-1, 0), (0,  1), ( 0, -1)]
        move2 = [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        move = move1 + move2
        
        while c_pos[0] != self.g_pos[0] or c_pos[1] != self.g_pos[1]:
            self.path.append(self.convert_to_1d_index(c_pos))
            min_cost = np.inf
            next_pos = None
            for dx, dy in move:
                new_pos = (c_pos[0] + dx, c_pos[1] + dy)
                if self.convert_to_1d_index(new_pos) not in self.path and new_pos in self.rhs_map:
                    if self.rhs_map[new_pos] < min_cost:
                        next_pos = new_pos
                        min_cost = self.rhs_map[new_pos]
                
            c_pos = next_pos
        
        self.path.append(self.convert_to_1d_index(self.g_pos))
                    
    def save_hist(self):
        
        return {
        'o_list': self.o_list,
        'g_map': self.g_map,
        'rhs_map': self.rhs_map,
        'static_map': self.c_static_map,
        'path': self.path,
        's_pos': self.s_pos
        }
    
    def search_path(self, s_pos, g_pos, map_w, map_h, c_static_map, hist = None):
        
        if len(np.array(c_static_map).shape) == 1:
            #reshape 1d map to 2d map
            c_static_map_cpy = np.array(c_static_map, copy = True)
            c_static_map_cpy = np.reshape(c_static_map_cpy, (map_h, map_w))
        
        if hist is None: 
            self.reset(s_pos, g_pos, c_static_map_cpy, map_w, map_h)
        else:    
            self.c_static_map = c_static_map_cpy
            
            self.o_list  = hist['o_list']
            self.g_map   = hist['g_map']
            self.rhs_map = hist['rhs_map']
            self.p_static_map = hist['static_map']
            path = hist['path']

            self.s_pos = hist['s_pos']
            self.g_pos = g_pos
            
            for i, c_ind in enumerate(path):
                c_pos = (c_ind % map_w, int(c_ind / map_w))
                if self.c_static_map[c_pos[1], c_pos[0]] != self.p_static_map:
                    self.update_queue(c_pos)
                    
        self.update_cost_map()
        
        self.get_shortest_path()
        
        hist = self.save_hist()
        
        return self.path, hist

