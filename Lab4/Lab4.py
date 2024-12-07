from Map import Map
from Node import Node
from RRT import RRT


if __name__ == "__main__":
    
    rrt_star = True  # If true run RRT* else RRT 

    max_run_rewire = True  # Run for max iteration
    rewire = True   # Run rewire with first found path
    cost_optim = False #Run only Cost Optimization   

    for i in [0]:#[0,1,3]:
        # Choose a map 0-3
        map_number = i
        goal_threshold = 0  
        max_iter=10000
        step_size=10
        search_radius=20
        p_bias=0.2
        caption = {
            "type":"rrt*" if rrt_star else "rrt",
            "map_number":map_number,
            "goal_threshold":goal_threshold,
            "search_radius":search_radius,
            "step_size":step_size,
            "max_iterations":max_iter,
            "path_length":None,
            "first_pth_found_after":None,
            "p":p_bias,
            "max_run":max_run_rewire
            
        }

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

        rrt = RRT(p=p_bias,start=start, goal=goal,map=grid,goal_threshold=goal_threshold,
                max_iter=max_iter,step_size=step_size,search_radius=search_radius)
        try:
            if not rrt_star:
                #if no smoothing, the function will return 2 values
                nodes,edges, ns_nodes, ns_edges = rrt.rrt(smooth_path=True) 
                _,_,ns_path=grid.fill_path(ns_nodes, ns_edges)
                _,_,path=grid.fill_path(nodes, edges)
                grid.plot_with_smoothing(states=ns_nodes,edges=ns_edges,path=ns_path,goal=goal.coord,
                                        goal_threshold=goal_threshold, path2=path,caption=caption)   
            elif (rewire or cost_optim)==True:
                fill_pth,fill_edges,nodes,edges = rrt.rrt_star(cost_optim=cost_optim,rewire=rewire,max_run=max_run_rewire)
                caption["first_pth_found_after"] = rrt.pth_found_after
                caption["path_length"] = rrt.path_length
                print(caption)
                _,_,path=grid.fill_path(fill_pth, fill_edges) if fill_pth else grid.fill_path(nodes,edges) 
                grid.plot(states=nodes,edges=edges,path=path,goal=goal.coord,
                        caption=caption
                        )
            else:
                print("Please Specify option:\n 1: Cost_Optimization\n 2: Rewire")
        except AssertionError as e:
            print(e)