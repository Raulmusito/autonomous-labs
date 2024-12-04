import numpy as np
import matplotlib.pyplot as plt
import random
from Node import Node
import math 

class RRT:
    def __init__(self, start, goal,goal_threshold, map, max_iter=1000, step_size=1, search_radius=100):
        self.start = start
        self.goal = goal
        self.map = map 
        self.max_iter = max_iter
        self.step_size = step_size
        self.search_radius = search_radius
        self.goal_threshold=goal_threshold
        self.tree = [self.start]
        self.Cost = np.full(map.shape, np.inf)

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
    
    def is_collision(self, node1, node2):
        collision, _ = self.get_pixels_between_points(node1, node2)
        return collision
    
    def node_exp(self,path,random_node):
        for n in path:
            new_node = Node(n[0],n[1]) 
            nearest = self.nearest_node(random_node)
            new_node.parent = nearest
            self.tree.append(new_node)
            if self.distance(new_node, self.goal) <= self.goal_threshold:
                print("Goal reached")
                # self.map.plot(states=self.tree,edges=self.get_edges(self.tree),path=[],goal=self.goal.coord,goal_threshold=self.goal_threshold)
                return self.tree, self.get_edges(self.tree)
        return None, None
    
    def cost_optim(self):
        
        self.Cost[self.start.coord] = 0   
        self.start.g = 0  
        for _ in range(self.max_iter):
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
            qnew.parent = nearest

            # Check for a better parent (cost optimization)
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
                return self.tree, self.get_edges(self.tree)

        raise AssertionError("Path not found\n")

    def rewire(self):
        self.Cost[self.start.coord] = 0  # Set cost of the start node to 0
        self.start.g = 0  # Initialize g value for the start node

        for _ in range(self.max_iter):
            # 1. Sample a random node
            random_node = self.get_random_node(0.2)

            # 2. Find the nearest node in the tree
            nearest = self.nearest_node(random_node)

            # 3. Steer towards the random node and check for collision
            qnew = self.new_config(nearest, random_node, self.step_size)
            collision, _ = self.get_pixels_between_points(qnew.coord, nearest.coord)
            if collision:
                continue  # Skip if the path is not collision-free

            # 4. Initialize the cost of the new node
            qnew.g = nearest.g + self.distance(nearest, qnew)
            qnew.parent = nearest

            # 5. Check for a better parent (cost optimization)
            neighbours = self.find_nearby_nodes(qnew)
            for node in neighbours:
                if node is not qnew.parent and self.distance(node, qnew) <= self.step_size:
                    collision, _ = self.get_pixels_between_points(node.coord, qnew.coord)
                    if not collision and node.g + self.distance(node, qnew) < qnew.g:
                        qnew.g = node.g + self.distance(node, qnew)
                        qnew.parent = node

            # # 6. Add the new node to the tree
            self.tree.append(qnew)

            # 7. Rewire the tree to ensure optimality
            
            for node in neighbours:
                if node is not qnew and self.distance(node, qnew) <= self.step_size:
                    collision, _ = self.get_pixels_between_points(qnew.coord, node.coord)
                    if not collision and qnew.g +self.distance(qnew, node) < node.g:
                        node.g = qnew.g  + self.distance(qnew, node)
                        node.parent = qnew
                        # self.tree[self.tree.index(node)] = node
                        
            # Remove nodes whose parents are not in the tree
            # self.tree.append(qnew)
            
            # 8. Check if the goal is reached
            if self.distance(qnew, self.goal) <= self.goal_threshold:
                # without_par = [node for node in self.tree[1::] if node.parent not in self.tree]    
                print("Goal reached with wire")
                return self.tree, self.get_edges(self.tree)

        raise AssertionError("Path not found\n")

    def rrt_star(self,node_expansion=False,cost_optim=False,rewire=False):
        self.Cost[self.start.coord] = 0  # Initialize cost of start node
        
        if cost_optim:
            node,edges= self.cost_optim()
            if edges:
                return node,edges
        if rewire:
            node,edges= self.rewire()
            if edges:
                return node,edges
            
        for _ in range(self.max_iter):
            # Sample a random node
            random_node = self.get_random_node(0.2) 

            #Find the nearest node in the tree
            nearest = self.nearest_node(random_node)
            
            # Steer towards the random node and check for collision
            _,path = self.get_pixels_between_points(nearest.coord, random_node.coord)
            
            if node_expansion:
                node,edges= self.node_exp(path,random_node)
                if edges:
                    return node,edges
          
        raise AssertionError("Path not found\n")


    def select_parent(self, neighbors, nearest, new_node):
        parent = nearest
        min_cost = nearest.cost + self.distance(nearest, new_node.position)
        for n in neighbors:
            if not self.is_collision(n, new_node):
                cost = n.cost + self.distance(n, new_node.position)
                if cost < min_cost:
                    parent = n
                    min_cost = cost
        return parent, min_cost
    
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
        pixels = list(zip(np.round(x_values).astype(int), np.round(y_values).astype(int)))
        q_nodes = []
        for i,p in enumerate(pixels):
            if 0 <= p[0] < self.map.grid.shape[0] and 0 <= p[1] < self.map.grid.shape[1]:
                if self.map.grid[p]!=0:
                    return False
        return True

    def rrt(self,smooth_path=True):
        for k in range(self.max_iter):
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
                        return smooth_pth, self.get_edges(smooth_pth)
                    return self.tree, self.get_edges(self.tree)
            
    def smooth(self, nodes):
        smooth_path = [nodes[-1]]  
        current_node = nodes[-1]  
        for i in range(len(nodes) - 2, -1, -1): #reversed traversal
            node = nodes[i]
            if not self.collision_free((node.x, node.y), (current_node.x, current_node.y)):
                nodes[i + 1].parent = smooth_path[-1]
                smooth_path.append(nodes[i + 1])
                current_node = nodes[i + 1]  # update current_node to the last valid node
        nodes[0].parent=smooth_path[-1]
        smooth_path.append(nodes[0])
        return smooth_path

            