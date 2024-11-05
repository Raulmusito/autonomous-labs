import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
import copy
import queue

def  find_neighbours(position, map, neighbours_num = 4):
    #   position: 1x2 tuple (x,y) from the cell where the neighbours 
    #   map: 2D n x m np.array from the map (used to now if the nieghbour is inside de map)
    #   neighbour_num = int 4 or 8. Assign connectivity

    # return a list with the x,y coordinates of the neighbours from "position" inside "map"

    """ 
    8D connectivity:            4D conectivity:

    | 1   1    1 |              | 0    1    0 |
    | 1  goal  1 |              | 1  goal   1 |
    | 1   1    1 |              | 0    1    0 |
    """

    height, width = map.shape   # get dimensions of the map to define if a neighbour is valid or not
    y,x = position              # depack the psotion
    neighbours = []             # define empty list of neighbours

    if neighbours_num == 4 or neighbours_num == 8:


        if neighbours_num == 4:                             # for connectivity 4
            if x-1 >= 0 :
                neighbours.append((position[0],position[1]-1))      # left
                
            if y-1 >= 0:
                neighbours.append((position[0]-1,position[1]))      # up

            if x+1 < width:
                neighbours.append((position[0],position[1]+1))      # right
                
            if y+1 < height:
                neighbours.append((position[0]+1,position[1]))      # down
        
        elif neighbours_num == 8:                           # for connectivity 8
            if x-1 >= 0 :
                neighbours.append((position[0],position[1]-1))      # left
                
            if y-1 >= 0:
                neighbours.append((position[0]-1,position[1]))      # up

            if x+1 < width:
                neighbours.append((position[0],position[1]+1))      # right
                
            if y+1 < height:
                neighbours.append((position[0]+1,position[1]))      # down

            if x-1 >= 0 and y-1 >= 0:
                neighbours.append((position[0]-1,position[1]-1))    # left up
                
            if y-1 >= 0 and x+1 < width:
                neighbours.append((position[0]-1,position[1]+1))    # right up

            if x+1 < width and y+1 < height:
                neighbours.append((position[0]+1,position[1]+1))    # right down 
                
            if y+1 < height and x-1 >= 0:
                neighbours.append((position[0]+1,position[1]-1))    # left down
            pass
                
        return neighbours 
    else: 
        print (f"Number of neighbours must be 4 or 8. Not {neighbours_num}")
        return 0

def bushfire (map, neighbours_num):
    #   map: 2D n x m np.array from the map
    #   neighbour_num = int 4 or 8. Assign connectivity

    # return a 2d np.array with the distance to the closest obstacle for each cell

    L = list(np.argwhere(map == 1)) # get a list of all the elements in the map == 1 (obstacles)

    while len(L) != 0:  # while there still neighbours to visit

        actual = L.pop() # pop the LAST neightbour in the list
        neighbours = find_neighbours(actual, map, neighbours_num=neighbours_num)
        for neighbour in neighbours: # for each neigbhour
            y,x = neighbour # get coordinates of the neighbour
            yact, xact = actual # get coordinates of the current cell
            if map[y][x] == 0: # if the neighbour has not already a closer obstacle
                map[y][x] = map[yact][xact] + 1 # add the value of the actual cell + 1 to the neighbour
                L =  [(y,x)]   + L # add the neighbour to BEGINNING the list
    return map

def remove_edges (map, edge_size = 3):
    #   map: 2D n x m np.array from the map
    #   edge_size = int 3 as default

    # return a 2d n x m np.array with the first and last edge_size columns and rows = 0

    height, width = map.shape

    map[0:edge_size, :] = 0                 # first rows 
    map[width-edge_size:width,:] = 0        # first columns
    map[:, 0:edge_size] = 0                 # last columns
    map[:,height -edge_size:height] = 0     # last rows

    return map

def get_attraction_function(map, goal, z = 1, distance = "d2", apply_scaling = True, scale_factor = 10):
    #   map: 2D n x m np.array from the map (to get the size of the output)
    #   z = int 1 as default attracction function multiplier
    #   distance = str type of function to calulate the atraction   
    #           manhatan: "mht"      quadratic euclidian: d2    
    #   apply_scaling = bool Trye as default should the distance be normalized?

    # return a 2d n x m np.array with the attraction for each cell

    height, width = map.shape # get the dimensions of the output map
    ygoal, xgoal = goal # get coordinates of the goal
    attraction_grid = np.zeros((height, width)) # create map with zeros on it

    if distance == "d2":
        """ to calculate the attraction a quadratic potential function is used:
                        U_att (q) = 1/2 Z d^2 (q, q_goal)"""
        for i in range(height):
            for j in range( width):
                attraction_grid[i][j] = .5*z*((j - xgoal)**2 + (i - ygoal)**2)

    if distance == "mht":
        """to calculate the attraction a quadratic potential function is used:
                    U_att (q) = abs (delta in y) + abs (delta in x)"""
        for i in range(height):
            for j in range( width):
                attraction_grid[i][j] = abs((j - xgoal)) + abs((i - ygoal))
    
    if apply_scaling == True: # normalize and multiply by the scale factor
        attraction_grid = attraction_grid/attraction_grid.max()*scale_factor

    return attraction_grid

def get_repulsive_function (map, Q = 4, eta = 15):
    #   map: 2D n x m np.array from the map with the distance to the closest obstacle 
    #        (to get the obstacles and dimensions of the return map)
    #   Q = int 4 as default distance of cell after which the repulsion will be 0
    #   eta = int 15 ad default multiplier of the repulsive function.  

    # return a 2d n x m np.array with the repulsion for each cell

    height, width = map.shape # get dimensions fo the map

    repuslsive_grid = np.zeros((height, width)) # create output map
    
    for i in range(height):
        for j in range( width):
            if map[i,j] > Q: # set as 0 if outside the Q range
                repuslsive_grid [i,j] = 0 
            else:            # set the actual repulsive value
                repuslsive_grid [i,j] = .5 * eta * ( (1/map[i,j]) - (1/Q) )**2
    return repuslsive_grid

def get_gradient_descent(map, q_start, neighbours_num = 4):
    #   map: 2D n x m np.array from the map with the repulsive force of each cell
    #   q_start = int coordinate to start the gradient descent
    #   neighbours_num = int 4 as default number of neighbours to use  

    # return a 2 1D list with the x and y coordinates of the gradient descent path

    E = .001 # breaking value
    y,x = q_start   # coordinates of the starting point
    #gradient_map = copy.deepcopy(map)

    # List to store the path
    gradient_listx = []
    gradient_listy = []

    dq = 100 # inicial gradient value
    c = 0 # counter to break after n iterations
    
    while abs(dq)> E:
        neighbours = find_neighbours((y,x), map, neighbours_num) # get neighbours for the actual xy
        temp_neig_dq = [] 

        for i in neighbours: # compute the gradient for each neighbour
            dq = map[i[0],i[1]] - map[y,x] 
            temp_neig_dq.append(dq) # store for the current neighbours

        dq = min(temp_neig_dq) # get the min gradient of the current neighbours
        y,x = neighbours[temp_neig_dq.index(dq)] #  get the index of that min neighbour, and use that index to get the position
        
        # update path list
        gradient_listx.append(x)
        gradient_listy.append(y)

        # Break if stuck 
        if c > 500:
            break

        c += 1
    
    return gradient_listx, gradient_listy

def wave_front(q_goal,apply_scaling=True,n_neighbours=4):
    #   map: 2D n x m np.array from the map with the repulsive force of each cell
    #   q_goal: int coordinate goal
    #   neighbours_num: int 4 as default number of neighbours to use  

    # return a map with the wave front value (distance to the goal for each cell)

    wave_map = copy.deepcopy(map).astype(float)
    value = 2
    wave_map[q_goal[0],q_goal[1]] = value
    que = queue.SimpleQueue()
    que.put([q_goal])

    while not que.empty():
        value+=1
        all_neigh = que.get() #all neighbours to be updated
        valid_neigh = []
        
        for p in all_neigh:
            neigh = [list(t) for t in find_neighbours([*p], wave_map,n_neighbours)] #convert list of tuples to nested list
            for n in neigh:
                if list(n)!=q_goal and wave_map[n[0],n[1]]==0:
                    wave_map[n[0],n[1]] = value
                    valid_neigh.append(n)
        
        if len(valid_neigh)!=0:
            que.put(valid_neigh)
    wave_map[np.where(wave_map == 1)]=wave_map.max() + 1  #setting obs manually to max
    
    # if apply_scaling:
    #     wave_map =wave_map/wave_map.max()*200  # scalling
    return wave_map

def find_path(wave_map,q_start,q_goal,neighbours_num):
    #   wave_map: 2D n x m np.array from the map with the wave front 
    #   q_start: tuple (x,y) coordinate start
    #   q_goal: tuple (x,y) coordinate goal
    #   neighbours_num: int 4 as default number of neighbours to use  

    # return a 2 1D list with the x and y coordinates of the gradient descent path
    pathx = []
    pathy = []
    while q_start!=q_goal:
        neigh =find_neighbours(q_start,wave_map,neighbours_num)
        q_start = neigh[np.argmin([wave_map[n] for n in neigh])]
        pathx.append(q_start[1])
        pathy.append(q_start[0])

    return pathx,pathy

def smiley_map():
    map = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]])

    return map

# Choose a map 0-4
map_number = 3

# Assign goal and start coordinates for each map
if map_number == 0:
    goal = (90, 70)
    #goal = (110, 40)
    start = (10, 10)
    # goal = (5,4) #for 10x10 grid
elif map_number == 1:
    goal = (90, 60)
    start = (60, 60)
elif map_number == 2:
    goal = (139, 38)
    start = (8, 31)
elif map_number == 3:
    goal = (375, 375)
    start = (50, 90)
else:
    goal = (30, 30)
    start = (1, 1)

# Load grid map 
if map_number <= 3:
    image = Image.open('Lab2/data/map'+str(map_number)+'.png').convert('L')
    grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255
    # binarize the image
    grid_map[grid_map > 0.5] = 1
    grid_map[grid_map <= 0.5] = 0
    # Invert colors to make 0 -> free and 1 -> occupied
    map = (grid_map * -1) + 1   
    #map = remove_edges(map)

else: 
    map = smiley_map()

# Plot map
plt.matshow(map)
plt.colorbar()
plt.show()


dist_to_obstacle = bushfire(copy.deepcopy(map), 4)
# Plot Distance to obstacle
plt.matshow(dist_to_obstacle)
plt.colorbar()
plt.show()


repulsive_grid = get_repulsive_function(dist_to_obstacle, Q = 5, eta = 15)
# Plot Repulsive grid
plt.matshow(repulsive_grid)
plt.colorbar()
plt.show()


attraction_grid = get_attraction_function(map, goal,distance = "d2")
# Plot Atractiong rid
plt.matshow(attraction_grid)
plt.colorbar()
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.show()


potential = attraction_grid + repulsive_grid
# Plot Full Potential grid
plt.matshow(potential)
plt.colorbar()
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.show()


gradient_listx, gradient_listy = get_gradient_descent(potential, start, neighbours_num=4)
# Plot potential map with the path of the gradient
plt.matshow(potential)
plt.colorbar()
plt.scatter(gradient_listx, gradient_listy, s=2, marker = "*", c="b")
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.show()


wave_map = wave_front(goal,True)
# Plot the wave map
plt.matshow(wave_map)
plt.colorbar()
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
#plt.savefig(f"wave_front{map_number}.png")
plt.show()

pathx,pathy = find_path(wave_map, start,goal, neighbours_num=4)
# Plot Wave map with the path from the start to the goal 
plt.matshow(wave_map)
plt.colorbar()
plt.scatter(pathx, pathy, s=1, marker = "*", c="b")
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
#plt.savefig(f"find_path_wave_front{map_number}.png")
plt.show()
