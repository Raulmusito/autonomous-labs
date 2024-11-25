from Map import Map
from Node import Node
from RRT import RRT


if __name__ == "__main__":
    

    # Choose a map 0-3
    map_number = 3

    # Assign goal and start coordinates for each map
    if map_number == 0:
        #goal = (50, 70)
        goal = Node(90, 70)
        start = Node(10, 10)
    elif map_number == 1:
        goal = Node(90, 60)
        start = Node(60, 60)
    elif map_number == 2:
        goal = Node(139, 38)
        start = Node(8, 31)
    elif map_number == 3:
        goal = Node(375, 375)
        start = Node(50, 90)
     
    grid = Map(map_number)

    goal_threshold = 10
    rrt = RRT(start=start, goal=goal,map=grid,max_iter=100000,step_size=15,goal_threshold=goal_threshold,search_radius=20)
    nodes,edges = rrt.rrt_star()
    grid.plot(states=nodes,edges=edges,goal=goal.coord,goal_threshold=goal_threshold)
