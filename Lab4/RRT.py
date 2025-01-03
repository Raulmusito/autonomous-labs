import numpy as np
import matplotlib.pyplot as plt
import random
from Node import Node
import math 
import copy
from tqdm import tqdm

class RRT:
    def __init__(self, start, goal,p,goal_threshold, map, max_iter=1000, step_size=1, search_radius=100):
        self.start = start
        self.goal = goal
        self.map = copy.copy(map) 
        self.max_iter = max_iter
        self.step_size = step_size
        self.search_radius = search_radius
        self.goal_threshold=goal_threshold
        self.p=p
        self.path_length=None
        self.pth_found_after=None
        self.tree = [self.start]
        self.Cost = np.full(map.shape, np.inf)

        self.expand_obstacle()
    
    def expand_obstacle(self):
        # Expand the obstacles in the map
        grid = copy.deepcopy(self.map.grid) # make copy not ot everwrite 
        # For each pixel in the map
        for i in range(self.map.grid.shape[0]): 
            for j in range(self.map.grid.shape[1]):
                # if pixel is obstacle
                if self.map.grid[i,j]==1:
                    # for each neighbor of the pixel
                    for k in range(-1,2):
                        for l in range(-1,2):
                            # check neighbor is in the map
                            if i+k>=0 and i+k<self.map.grid.shape[0] and j+l>=0 and j+l<self.map.grid.shape[1]:
                                # make naighbor an 
                                grid[i+k,j+l]=1
        print("Obstacle expanded")
        #re assign the expanded grid to the map
        self.map.grid = grid

    def distance(self, node1, node2):
        return math.sqrt((node2.coord[0] - node1.coord[0])**2 + (node2.coord[1] - node1.coord[1])**2)

    def get_pixels_between_points(self, point1, point2):  # Accepts coords
        collision = False
        num_steps = max(abs(point2[0] - point1[0]), abs(point2[1] - point1[1])) + 1

        # Interpolate x and y coordinates
        x_values = np.linspace(point1[0], point2[0], num_steps)
        y_values = np.linspace(point1[1], point2[1], num_steps)

        # Combine x and y into pixel coordinates
        pixels = list(zip(np.round(x_values).astype(int), np.round(y_values).astype(int)))
        q_nodes = []

        for i, p in enumerate(pixels):
            if 0 <= p[0] < self.map.grid.shape[0] and 0 <= p[1] < self.map.grid.shape[1]:
                if self.map.grid[p] != 0:
                    collision = True
                    del pixels[i:]
                    return collision, q_nodes
            else:
                collision = True  # Out-of-bounds considered as collision
                del pixels[i:]
                return collision, q_nodes

            if (i + 1) % self.step_size == 0:
                q_nodes.append(p)

        return collision, q_nodes

    def get_random_node(self,p):
        if random.random() < p:
            return self.goal
        # Get all free cells (where grid is 0)
        free_positions = np.argwhere(self.map.grid == 0)
        # Randomly choose one of the free cells 
        random_index = np.random.choice(len(free_positions))
        node = Node(free_positions[random_index][0],free_positions[random_index][1])

        return node

    def nearest_node(self, random_node):
        return min(self.tree, key=lambda node: self.distance(node, random_node))
    
    def find_nearby_nodes(self, node):
        return [n for n in self.tree if self.distance(n, node) < self.search_radius]
     
    def min_paths(self, tree):
        goal_idx = [i+1 for i,n in enumerate(tree[1:]) if n.coord == self.goal.coord and n.parent.coord !=n.coord]
        paths = []
        # for g in goal_idx:
        node = tree[goal_idx[-1]]
        path = [node]
        while node.parent:
            node=node.parent
            path.append(node)
        paths.append(path)
        
        return paths

    
    def cost_optim(self):
        
        self.Cost[self.start.coord] = 0   
        self.start.g = 0  
        for i in tqdm(range(self.max_iter)):
            # Sample a random node
            random_node = self.get_random_node(self.p)

            # Find the nearest node in the tree
            nearest = self.nearest_node(random_node)

            # Steer towards the random node and check for collision
            qnew = self.new_config(nearest, random_node, self.step_size)
            collision, _ = self.get_pixels_between_points(qnew.coord, nearest.coord)
            if collision:
                continue  # Skip if the path is not collision-free

            # Initialize the cost of the new node
            qnew.g = nearest.g + self.distance(nearest, qnew)
            qnew.parent = nearest

            # check for a better parent
            neighbours = self.find_nearby_nodes(qnew)
            for node in neighbours:
                if node is not qnew.parent and self.distance(node, qnew) <= self.step_size:
                    collision, _ = self.get_pixels_between_points(node.coord, qnew.coord)
                    if not collision and node.g + self.distance(node, qnew) < qnew.g:
                        qnew.g = node.g + self.distance(node, qnew)
                        qnew.parent = node

            # Add the new node to the tree
            self.tree.append(qnew)

            # Check if the goal is reached
            if self.distance(qnew, self.goal) <= self.goal_threshold:
                print("Goal reached")
                if self.pth_found_after == None: 
                    self.pth_found_after=i 
                return self.tree, self.get_edges(self.tree)

        raise AssertionError("Path not found\n")

    def rewire(self, max_run=False):
        self.Cost[self.start.coord] = 0  # Set cost of the start node to 0
        self.start.g = 0  # Initialize g value for the start node

        for i in tqdm(range(self.max_iter)):
            # Sample a random node
            random_node = self.get_random_node(0.2)

            # Find the nearest node in the tree
            nearest = self.nearest_node(random_node)

            # Steer towards the random node and check for collision
            qnew = self.new_config(nearest, random_node, self.step_size)
            
            collision, _ = self.get_pixels_between_points(qnew.coord, nearest.coord)
            if collision:
                continue  # Skip if the path is not collision-free

            # Initialize the cost of the new node
            qnew.g = nearest.g + self.distance(nearest, qnew)

            if nearest.coord != qnew.coord:  # Prevent self-parenting
                qnew.parent = nearest
            else:
                continue  # Skip adding qnew if it is the same as nearest to avoid self-parenting

            # Check for a better parent 
            neighbours = self.find_nearby_nodes(qnew)
            for node in neighbours:
                if node is not qnew.parent and self.distance(node, qnew) <= self.step_size:
                    collision, _ = self.get_pixels_between_points(node.coord, qnew.coord)
                    if not collision and node.g + self.distance(node, qnew) < qnew.g:
                        if node.coord != qnew.coord:  # Prevent self-parenting
                            qnew.g = node.g + self.distance(node, qnew)
                            qnew.parent = node

            # Add the new node to the tree
            self.tree.append(qnew)

            # Rewire the tree to ensure optimality
            for node in neighbours:
                if node is not qnew and self.distance(node, qnew) <= self.step_size:
                    collision, _ = self.get_pixels_between_points(qnew.coord, node.coord)
                    if not collision and qnew.g + self.distance(qnew, node) < node.g:
                        if qnew.coord != node.coord:  # Prevent self-parenting
                            node.g = qnew.g + self.distance(qnew, node)
                            node.parent = qnew

            # Check if the goal is reached
            if self.distance(qnew, self.goal) <= self.goal_threshold:
                if not max_run:
                    print("Goal reached with rewire")
                    self.pth_found_after=i 
                    return None,None,self.tree, self.get_edges(self.tree)
                else:
                    if self.pth_found_after == None:
                        print("Goal reached with rewire")
                        self.pth_found_after=i 

        if max_run:
            if self.goal in self.tree:
                paths = self.min_paths(self.tree) #find shortest patt 
                if not paths :
                    raise AssertionError("Path not found\n")
                short_path = min(paths, key=len) 
                return short_path,self.get_edges(short_path), self.tree,self.get_edges(self.tree)
        if self.goal not in self.tree: 
            raise AssertionError("Path not found\n")


    def rrt_star(self,cost_optim=True,rewire=False,max_run=False):
        self.Cost[self.start.coord] = 0  # Initialize cost of start node
        
        if cost_optim:
            node,edges= self.cost_optim()
            self.path_length = len(node)
            if edges:
                return None,None,node,edges
        if rewire:
            fill_pth,fill_edge,node,edges= self.rewire(max_run=max_run)
            if fill_pth:self.path_length = len(fill_pth)
            if edges:
                fill_pth = fill_pth[::-1] if max_run else fill_pth
                return fill_pth,fill_edge,node,edges
          
        raise AssertionError("Path not found\n")

    
    def get_edges(self,nodes):
        edges = []
        for node in nodes:
            if node.parent:
                edges.append((nodes.index(node.parent), nodes.index(node)))
        return edges

    def new_config(self, nearest, random_node, step_size):
        distance = self.distance(nearest, random_node)
        if distance <= step_size:
            return random_node
        else:
            theta = math.atan2(random_node.coord[1] - nearest.coord[1], random_node.coord[0] - nearest.coord[0])
            return Node(int(nearest.coord[0] + step_size * math.cos(theta)), int(nearest.coord[1] + step_size * math.sin(theta)))

    def collision_free(self, point1, point2):
        num_steps = max(abs(point2[0] - point1[0]), abs(point2[1] - point1[1])) + 1
        # Interpolate x and y coordinates
        x_values = np.linspace(point1[0], point2[0], num_steps)
        y_values = np.linspace(point1[1], point2[1], num_steps)

        # Combine x and y into pixel coordinates
        pixels = list(zip((np.floor(x_values)).astype(int), (np.floor(y_values)).astype(int)))
        q_nodes = []
        for i,p in enumerate(pixels):
            if 0 <= p[0] < self.map.grid.shape[0] and 0 <= p[1] < self.map.grid.shape[1]:
                if self.map.grid[p]!=0:
                    return False
        return True

    def rrt(self,smooth_path=True):
        for k in tqdm(range(self.max_iter)):
            # 1. Sample a random node
            random_node = self.get_random_node(0.2) 

            # 2. Find the nearest node in the tree
            nearest = self.nearest_node(random_node)
            
            # 3. New configuration
            qnew = self.new_config(nearest, random_node, self.step_size)


            if self.collision_free((nearest.x,nearest.y), (qnew.x,qnew.y)):
                qnew.parent = nearest
                # 5. Check if goal is reached
                self.tree.append(qnew)
                
                if self.distance(qnew, self.goal) <= self.goal_threshold:
                    print(f"Goal reached in {k} iterations")
                    ns_nodes = copy.deepcopy(self.tree)
                    ns_edges = copy.deepcopy(self.get_edges(self.tree))
                    if self.pth_found_after == None: 
                        self.pth_found_after=k
                    if smooth_path:
                        # # Extract path from start to goal
                        path = []
                        current = qnew
                        while current is not None:
                            path.append(current)
                            current = current.parent
                        
                        # Smooth the path
                        smooth_pth = self.smooth(path)
                            
                        # Return both the smoothed path and its edges
                        return smooth_pth, self.get_edges(smooth_pth), ns_nodes, ns_edges
                    return self.tree, self.get_edges(self.tree)
        raise AssertionError("Path not found\n")
    
    def smooth(self, nodes):
        nodes = nodes[::-1]
        smooth_path = []
        target = nodes[-1]  
        i = 0
        while target is not nodes[0]:
            node = nodes[i]
            i += 1
            if self.collision_free((node.x, node.y), (target.x, target.y)):
                i = 0
                smooth_path.append(target)
                smooth_path[-1].parent = node
                target = node  
        smooth_path.append(nodes[0])

        return smooth_path [::-1]

            