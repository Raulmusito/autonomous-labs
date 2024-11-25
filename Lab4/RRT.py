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

    def get_pixels_between_points(self,point1, point2):

        num_steps = max(abs(point2[0] - point1[0]), abs(point2[1] - point1[1])) + 1

        # Interpolate x and y coordinates
        x_values = np.linspace(point1[0], point2[0], num_steps)
        y_values = np.linspace(point1[1], point2[1], num_steps)

        # Combine x and y into pixel coordinates
        pixels = list(zip(np.round(x_values).astype(int), np.round(y_values).astype(int)))
        q_nodes = []
        for i,p in enumerate(pixels):
            if self.map.grid[p]!=0:
                del pixels[i:]
                return q_nodes
            if(i+1)% self.step_size==0:
                q_nodes.append(p)
        if q_nodes:
            # self.draw_path(q_nodes,point2,color='b',show=True)
            print(q_nodes)
        return q_nodes
    
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
    
    def rrt_star(self):
        self.Cost[self.start.coord] = 0  # Initialize cost of start node
        self.start.g = 0
        nodes = []
        
        for _ in range(self.max_iter):
            # 1. Sample a random node
            random_node = self.get_random_node(0.2)
            print('Random node:', random_node)
            
            # 2. Find the nearest node in the tree
            nearest = self.nearest_node(random_node)
            
            # 3. Steer towards the random node and check for collision
            path = self.get_pixels_between_points(nearest.coord, random_node.coord)
            
            for n in path:
                new_node = Node(n[0],n[1])
                new_node.parent = nearest
                self.Cost[new_node.coord] = self.Cost[new_node.parent.coord] + self.step_size
                # print('Streak:', new_node.coord, len(path) - 1)
                 
                new_cost = self.Cost[nearest.coord] + self.distance(nearest, new_node)
            
                if new_node.coord not in self.Cost or new_cost < self.Cost[new_node.coord]:
                    self.Cost[new_node.coord] = new_cost
                    print('Node added:', new_node.coord, 'Cost:', self.Cost[new_node.coord])
                    
                    # Update nearest to the newly added node
                    nearest = new_node
                    self.tree.append(new_node)
                    nodes.append(new_node.coord) 
                print("Dist b/w curr node and goal:",self.distance(new_node, self.goal) )
                # 5. Check if goal is reached
                if self.distance(new_node, self.goal) <= self.goal_threshold:
                    print("Goal reached")
                    return self.tree, self.get_edges()
                
        return self.tree, self.get_edges()


     

    def find_nearby_nodes(self, node):
        return [n for n in self.tree if self.distance(n, node) < self.search_radius]

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

    def get_edges(self):
        edges = []
        for node in self.tree:
            if node.parent:
                edges.append((self.tree.index(node.parent), self.tree.index(node)))
        return edges


    def rrt(self):
        self.start.g = 0
        for k in range(self.max_iter):
            random_node = self.get_random_node(0.2)
            nearest = self.nearest_node(random_node)
            new_node = self.steer(nearest, random_node)

            if  self.is_collision2(nearest, new_node) == False:
                neighbors = self.find_nearby_nodes(new_node)
                parent, cost = self.select_parent(neighbors, nearest, new_node)
                new_node.parent = parent
                new_node.cost = cost
                self.rewire(neighbors, new_node)
                self.tree.append(new_node)
                if self.distance(new_node, self.goal.position) <= self.goal_threshold:
                    return self.fill_path()