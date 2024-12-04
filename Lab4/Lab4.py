from Map import Map
from Node import Node
from RRT import RRT


if __name__ == "__main__":
    

    # Choose a map 0-3
    map_number = 0
    goal_threshold = 5
    max_iter=100000
    step_size=10
    search_radius=10

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
        step_size=40           #PERFORM WELL AT STEPSIZE 40
    
    grid = Map(map_number)

    rrt = RRT(start=start, goal=goal,map=grid,goal_threshold=goal_threshold,
              max_iter=max_iter,step_size=step_size,search_radius=search_radius)
    try:
        #nodes,edges = rrt.rrt_star(node_expansion=False,cost_optim=False,rewire=True)
        # if no smoothing the function will return 2 values
        nodes,edges, ns_nodes, ns_edges = rrt.rrt(smooth_path=True) 
        _,_,path=grid.fill_path(nodes, edges)
        grid.plot(states=nodes,edges=edges,path=path,goal=goal.coord,goal_threshold=goal_threshold)
        _,_,ns_path=grid.fill_path(ns_nodes, ns_edges)
        grid.plot_with_smoothing(states=ns_nodes,edges=ns_edges,path=ns_path,goal=goal.coord,goal_threshold=goal_threshold, path2=path)
    except AssertionError as e:
        print(e)