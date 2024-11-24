from Map import Map
from Node import Node
from RRT import RRT


if __name__ == "__main__":
    start = Node(10, 10)
    goal = Node(90, 70)
    map_number = 0
    # map_dims = (100, 100)
    # obstacle_list = [(20, 20), (30, 30), (40, 40)]  # Example obstacles
    grid = Map(0)
    # Plot the map
    #grid.plot()
    rrt = RRT(start=start, goal=goal,goal_threshold=2,map=grid,max_iter=1000,step_size=10, search_radius=5)
    nodes,edges = rrt.rrt_star()
