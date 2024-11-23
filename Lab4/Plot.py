from Point import Point
import numpy as np
import math
from matplotlib import pyplot as plt
from PIL import Image


class Plot:
    def _init_(self,map):
        self.grid_map = map

    def plot(grid_map, states, edges, path):
        plt.figure(figsize=(10, 10))
        plt.matshow(grid_map, fignum=0)
        for i,v in enumerate(states):
            
            plt.plot(v.position[1], v.position[0], "+w")
            plt.text(v.position[1], v.position[0], i, fontsize=14, color="w")

        for e in edges:
            plt.plot(
                [states[e[0]].position[1], states[e[1]].position[1]],
                [states[e[0]].position[0], states[e[1]].position[0]],
                "--g",
            )

        for i in range(1, len(path)):
            plt.plot(
                [states[path[i - 1]].y, states[path[i]].y],
                [states[path[i - 1]].x, states[path[i]].x],
                "r",
            )
        # Start
        plt.plot(states[0].position[1], states[0].position[0], marker="^", c="y")
        # Goal
        plt.plot(states[-1].position[1], states[-1].position[0], c="r", marker=(5, 1))

        path = []
        plt.show()

    def fill_path(vertices, edges):
        edges.reverse()
        path = [edges[0][1]]
        next_v = edges[0][0]
        i = 1
        while next_v != 0:
            while edges[i][1] != next_v:
                i += 1
            path.append(edges[i][1])
            next_v = edges[i][0]
        path.append(0)
        edges.reverse()
        path.reverse()
        return vertices, edges, path
