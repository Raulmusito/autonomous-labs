import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import random
from Point import Point
from Plot import Plot
class Node:
    def __init__(self, position):
        self.position = position
        self.point= Point(position[0],position[1])
        self.parent = None
        self.cost = 0

class RRT:
    def __init__(self, start, goal,goal_threshold, map, max_iter=1000, step_size=1, search_radius=100):
        self.start = Node(start)
        self.goal = Node(goal)
        self.map = map 
        self.max_iter = max_iter
        self.step_size = step_size
        self.search_radius = search_radius
        self.goal_threshold=goal_threshold
        self.tree = [self.start]
        self.Cost = np.full(map.shape, np.inf)

    def distance(self, node1, node2):
        return np.linalg.norm(np.array(node1.position) - np.array(node2))


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
            plt.scatter(self.goal.position[1], self.goal.position[0], c="r", marker=(5, 1))
            plt.scatter(self.start.position[1], self.start.position[0], s=50, marker="^", c="y")
            plt.colorbar()
            plt.show()
        

    def get_random_node(self,p):
        if random.random() < p:
            return goal
        # Get all free cells (where grid is 0)
        free_positions = np.argwhere(grid == 0)
        # Randomly choose one of the free cells
        random_index = np.random.choice(len(free_positions))
        return tuple(free_positions[random_index])

    def nearest_node(self, random_node):
        return min(self.tree, key=lambda node: self.distance(node, random_node))
    
    def is_collision(self, node1, node2):
        collision, _ = self.check_line_collision(node1, node2.position)
        return collision
    
    def rrt_star(self):
        self.Cost[self.start.position] = 0  # Initialize cost of start node
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
        vector = np.array(to_node) - np.array(from_node)
        length = np.linalg.norm(vector)
        vector = vector / length * min(self.step_size, length)
        new_position = np.array(from_node) + vector
        return tuple(self.map(new_position))

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



import numpy as np
import math
from matplotlib import pyplot as plt
from PIL import Image

from Point import Point

# Load grid map
image = Image.open("map0.png").convert("L")
grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1]) / 255
# binarize the image
grid_map[grid_map > 0.5] = 1
grid_map[grid_map <= 0.5] = 0
# Invert colors to make 0 -> free and 1 -> occupied
grid_map = (grid_map * -1) + 1
# Show grid map
plt.matshow(grid_map)
plt.colorbar()
plt.show()




def load_and_process_map(map_number):
    """
    Loads and processes the map image for a given map number.
    
    Args:
        map_number (int): The number of the map file to load (e.g., 'map0.png').
    
    Returns:
        np.ndarray: A processed grid map where 0 represents free cells and 1 represents obstacles.
        None: If the map file is not found.
    """
    try:
        # Load the image
        image = Image.open(f'map{map_number}.png').convert('L')
    except FileNotFoundError:
        print(f"Map file 'map{map_number}.png' not found.")
        return None

    # Convert image to numpy array and normalize
    grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1]) / 255

    # Binarize the grid map
    grid_map[grid_map > 0.5] = 1
    grid_map[grid_map <= 0.5] = 0

    # Invert the grid to make 0 -> free and 1 -> occupied
    grid = (grid_map * -1) + 1

    return grid

if __name__ == "__main__":
    start = (10, 10)
    goal = (90, 70)
    map_number = 0
    # map_dims = (100, 100)
    # obstacle_list = [(20, 20), (30, 30), (40, 40)]  # Example obstacles
    grid = load_and_process_map(map_number)
    # Plot the map
    # plt.matshow(grid, origin='upper')
    # plt.colorbar()
    # plt.show()
    rrt = RRT(start=start, goal=goal,goal_threshold=2,map=grid,max_iter=1000,step_size=10, search_radius=5)
    nodes,edges = rrt.rrt_star()
    Plot.plot(grid, nodes, edges, [])
