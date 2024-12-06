import numpy as np
from matplotlib import pyplot as plt
from PIL import Image


class Map():
    def __init__(self, map_number=0):
        print ("Map number: ", map_number)
        self.map_number = map_number
        self.grid = self.load_and_process_map()
        self.shape = self.grid.shape
        

    def load_and_process_map(self):
        """
        Loads and processes the map image for a given map number.
        
        Args:
            map_number (int): The number of the map file to load (e.g., 'map0.png').
        
        Returns:
            np.ndarray: A processed grid map where 0 represents free cells and 1 represents obstacles.
            None: If the map file is not found.
        """
        try:
            # Load the image maps Lab4/maps/map3.png
            image = Image.open(f"maps/map{self.map_number}.png").convert('L')
        except FileNotFoundError:
            print(f"Map file 'map{self.map_number}.png' not found.")
            return None

        # Convert image to numpy array and normalize
        grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1]) / 255

        # Binarize the grid map
        grid_map[grid_map > 0.5] = 1
        grid_map[grid_map <= 0.5] = 0

        # Invert the grid to make 0 -> free and 1 -> occupied
        grid = (grid_map * -1) + 1

        return grid
    
    def plot(self,step_size,search_radius,goal,goal_threshold,states=[], edges=[], path=[]):
        plt.figure(figsize=(10, 10))
        plt.matshow(self.grid, fignum=0)

        if len(states) > 0:
            for i,v in enumerate(states):
                
                plt.plot(v.coord[1], v.coord[0], "+w")
                # plt.text(v.coord[1], v.coord[0], i, fontsize=14, color="w")

            for e in edges:
                plt.plot(
                    [states[e[0]].coord[1], states[e[1]].coord[1]],
                    [states[e[0]].coord[0], states[e[1]].coord[0]],
                    "--g",
                )

            for i in range(1, len(path)):
                plt.plot(
                    [path[i - 1][1], path[i][1]],
                    [path[i - 1][0], path[i][0]],
                    "r",
                )
            # Start
            plt.plot(states[0].coord[1], states[0].coord[0], marker="^", c="y")
            # Goal
            if not path: plt.plot(states[-1].coord[1], states[-1].coord[0], c="r", marker=(5, 1))
            else: plt.plot(path[-1][1], path[-1][0], c="r", marker=(5, 1))
            plt.plot(goal[1], goal[0], c="r", marker='o', markersize=2*3.14*1.7*goal_threshold , alpha=0.3)
            plt.title(f"Step Size: {step_size}, Search Radius: {search_radius}", fontsize=14, color='black')

        path = []

        plt.savefig(f"rrt*_results/rrt.png")
        plt.show()

    def plot_with_smoothing(self,step_size,search_radius,goal,goal_threshold,states=[], edges=[], path=[], path2=[]):
        plt.figure(figsize=(10, 10))
        plt.matshow(self.grid, fignum=0)

        if len(states) > 0:
            for i,v in enumerate(states):
                
                plt.plot(v.coord[1], v.coord[0], "+w")
                # plt.text(v.coord[1], v.coord[0], i, fontsize=14, color="w")

            for e in edges:
                plt.plot(
                    [states[e[0]].coord[1], states[e[1]].coord[1]],
                    [states[e[0]].coord[0], states[e[1]].coord[0]],
                    "--g",
                )

            for i in range(1, len(path)):
                plt.plot(
                    [path[i - 1][1], path[i][1]],
                    [path[i - 1][0], path[i][0]],
                    "r",
                )
            for i in range(1, len(path2)):
                plt.plot(
                    [path2[i - 1][1], path2[i][1]],
                    [path2[i - 1][0], path2[i][0]],
                    "--b",
                )
            # Start
            plt.plot(states[0].coord[1], states[0].coord[0], marker="^", c="y")
            # Goal
            plt.plot(states[-1].coord[1], states[-1].coord[0], c="r", marker=(5, 1))
            plt.plot(goal[1], goal[0], c="r", marker='o', markersize=2*3.14*1.7*goal_threshold , alpha=0.3)
            plt.title(f"Step Size: {step_size}, Search Radius: {search_radius}", fontsize=14, color='black')

        path = []
        plt.savefig(f"rrt_results/smoothing.png")
        plt.show()

    def fill_path(self,vertices, edges):
        path = []
        node = vertices[-1]
        while node.parent is not None:
            path.append(node.coord)
            node = node.parent
        path.append(vertices[0].coord)
        path.reverse()
        return vertices, edges, path