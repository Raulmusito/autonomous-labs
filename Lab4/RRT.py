import numpy as np
import matplotlib.pyplot as plt
import random
from Node import Node

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
        return np.linalg.norm(node1.numpy() - node2.numpy())


    def check_line_collision(self, start, end):  ##BresenhamLine
        pixels = []
        collision = False
        x1, y1 = start
        x2, y2 = end
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = self.step_size if x1 < x2 else -self.step_size
        sy = self.step_size if y1 < y2 else -self.step_size
        err = dx - dy

        while True:
            # Ensure coordinates are within map bounds
            if 0 <= x1 <= np.shape(self.map)[1] and 0 <= y1 <= np.shape(self.map)[0]:
                if self.map[x1,y1] == 0:
                    pixels.append((x1, y1))
                    if x1 == x2 and y1 == y2:
                        break
                    e2 = 2 * err
                    if e2 > -dy:
                        err -= dy
                        x1 += sx
                    if e2 < dx:
                        err += dx
                        y1 += sy
                else:
                    collision = True
                    return collision, pixels

            else:
                collision = True
                break
            
        return collision, pixels
    
    def draw_path(self,path,random_node,color='b',show=False):
        path = list(zip(*path))
        plt.matshow(self.map, origin='upper')
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
        free_positions = np.argwhere(self.map == 0)
        # Randomly choose one of the free cells
        random_index = np.random.choice(len(free_positions))
        return Node(free_positions[random_index])

    def nearest_node(self, random_node):
        return min(self.tree, key=lambda node: self.distance(node, random_node))
    
    def is_collision(self, node1, node2):
        collision, _ = self.check_line_collision(node1, node2.position)
        return collision
    
    def rrt_star(self):
        #self.Cost[self.start.position] = 0  # Initialize cost of start node
        self.start.g = 0
        nodes = []
        
        for _ in range(self.max_iter):
            # 1. Sample a random node
            random_node = self.get_random_node(0.2)
            print('Random node:', random_node)
            
            # 2. Find the nearest node in the tree
            nearest = self.nearest_node(random_node)
            
            # 3. Steer towards the random node and check for collision
            collision, path = self.check_line_collision(nearest.position, random_node)
            # if collision:
            #     continue  # Skip this iteration if there's an obstacle
            # self.draw_path(path, random_node,color='r', show=True)
            # 4. Add new nodes from the valid path
            for n in path[1:]:
                new_node = Node(n)
                new_node.parent = nearest
                self.Cost[new_node.position] = self.Cost[new_node.parent.position] + self.step_size
                print('Streak:', new_node.position, len(path) - 1)
                
                # Update nearest to the newly added node
                nearest = new_node
                # self.tree.append(new_node)
                nodes.append(n)
                new_cost = self.Cost[nearest.position] + self.distance(nearest, new_node.position)
            
                if new_node.position not in self.Cost or new_cost < self.Cost[new_node.position]:
                    self.Cost[new_node.position] = new_cost
                    print('Node added:', new_node.position, 'Cost:', self.Cost[new_node.position])
                    
                    # Update nearest to the newly added node
                    nearest = new_node
                    self.tree.append(new_node)
                    nodes.append(n)
                # 5. Check if goal is reached
                if self.distance(new_node, self.goal.position) <= self.goal_threshold:
                    print("Goal reached")
                    return self.tree, self.get_edges()
                    # self.draw_path(nodes,random_node, color='g', show=True)
                
        return self.tree, self.get_edges()  # Return the final tree and edges 


    def steer(self, from_node, to_node):
        vector = to_node.numpy() - from_node.numpy()
        length = np.linalg.norm(vector)
        vector = vector / length * min(self.step_size, length)
        new_position = np.array(from_node) + vector
        return Node(new_position)

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