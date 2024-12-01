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

    def get_pixels_between_points(self,point1, point2): ##accepts coords

        collision = False
        num_steps = max(abs(point2[0] - point1[0]), abs(point2[1] - point1[1])) + 1

        # Interpolate x and y coordinates
        x_values = np.linspace(point1[0], point2[0], num_steps)
        y_values = np.linspace(point1[1], point2[1], num_steps)

        # Combine x and y into pixel coordinates
        pixels = list(zip(np.round(x_values).astype(int), np.round(y_values).astype(int)))
        q_nodes = []
        for i,p in enumerate(pixels):
            if self.map.grid[p]!=0:
                collision= True
                del pixels[i:]
                return collision,q_nodes
            if(i+1)% self.step_size==0:
                q_nodes.append(p)

        return collision,q_nodes
    
    def draw_path(self,path,random_node,color='b',show=False):
        path = list(zip(*path))
        plt.matshow(self.map.grid, origin='upper')
        plt.scatter(path[1], path[0], s=50, marker=".", c=color)
        if show:
            plt.scatter(random_node[1], random_node[0], c="w",s=30, marker="o")
            plt.scatter(self.goal.y, self.goal.x, c="r", marker=(5, 1))
            plt.scatter(self.start.y, self.start.x, s=50, marker="^", c="y")
            plt.colorbar()
            plt.show()
        

    def get_random_node(self,p):
        if random.random() < p:
            return self.goal
        # Get all free cells (where grid is 0)
        free_positions = np.argwhere(self.map.grid == 0)
        # Randomly choose one of the free cells
        random_index = np.random.choice(len(free_positions))
        return Node(free_positions[random_index][0],free_positions[random_index][1])

    def nearest_node(self, random_node):
        return min(self.tree, key=lambda node: self.distance(node, random_node))
    
    def find_nearby_nodes(self, node):
        return [n for n in self.tree if self.distance(n, node) < self.search_radius]
    
    def is_collision(self, node1, node2):
        collision, _ = self.get_pixels_between_points(node1, node2)
        return collision
    

    def rrt_star(self,smooth_path=False):
        self.Cost[self.start.coord] = 0  # Initialize cost of start node
        self.start.g = 0
        nodes = []
        
        for _ in range(self.max_iter):
            # 1. Sample a random node
            random_node = self.get_random_node(0.2) 

            # 2. Find the nearest node in the tree
            nearest = self.nearest_node(random_node)
            
            # 3. Steer towards the random node and check for collision
            _,path = self.get_pixels_between_points(nearest.coord, random_node.coord)
            
            for n in path:
                new_node = Node(n[0],n[1])
                new_node.parent = nearest

                self.Cost[new_node.coord] = self.Cost[new_node.parent.coord] + self.step_size
                new_cost = self.Cost[nearest.coord] + self.distance(nearest, new_node)
            
                if new_node.coord not in self.Cost or new_cost < self.Cost[new_node.coord]:
                    self.Cost[new_node.coord] = new_cost

                    # Update nearest to the newly added node
                    nearest = new_node
                    self.tree.append(new_node)
                    nodes.append(new_node.coord) 

                # 5. Check if goal is reached
                if self.distance(new_node, self.goal) <= self.goal_threshold:
                    print("Goal reached")
                    return self.tree, self.get_edges(self.tree)
         
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

    def rewire(self, neighbors, new_node):
        for n in neighbors:
            if not self.is_collision(new_node, n):
                cost = new_node.cost + self.distance(new_node, n.position)
                if cost < n.cost:
                    n.parent = new_node
                    n.cost = cost

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
            theta = math.atan2(random_node.coord[0] - nearest.coord[0], random_node.coord[1] - nearest.coord[1])
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
            if self.map.grid[p]!=0:
                return False
        return True

    def rrt(self):
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
                    return self.tree, self.get_edges(self.tree)
            
            """print("Goal reached")
                    last = qnew
                    while last != self.goal:
                        nodes.append(last)
                        last = last.parent
                    nodes.append(last) """