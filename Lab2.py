import os
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
import copy
import queue

def  find_neighbors(position, map, neighbours_num = 4):
    height, width = map.shape
    y,x = position
    neighbors = []

    if neighbours_num == 4:
        if x-1 >= 0 :
            neighbors.append((position[0],position[1]-1)) # left
            
        if y-1 >= 0:
            neighbors.append((position[0]-1,position[1])) # up

        if x+1 < width:
            neighbors.append((position[0],position[1]+1)) # right
            
        if y+1 < height:
            neighbors.append((position[0]+1,position[1])) # down
    
    elif neighbours_num == 8:
        if x-1 >= 0 :
            neighbors.append((position[0],position[1]-1)) # left
            
        if y-1 >= 0:
            neighbors.append((position[0]-1,position[1])) # up

        if x+1 < width:
            neighbors.append((position[0],position[1]+1)) # right
            
        if y+1 < height:
            neighbors.append((position[0]+1,position[1])) # down

        if x-1 >= 0 and y-1 >= 0:
            neighbors.append((position[0]-1,position[1]-1)) # left up
            
        if y-1 >= 0 and x+1 < width:
            neighbors.append((position[0]-1,position[1]+1)) # right up

        if x+1 < width and y+1 < height:
            neighbors.append((position[0]+1,position[1]+1)) # right down 
            
        if y+1 < height and x-1 >= 0:
            neighbors.append((position[0]+1,position[1]-1)) # left down
        pass
            
    return neighbors 

def bushfire (map, neighbours_num):

    L = list(np.argwhere(map == 1)) # get a list of all the elements in the map == 1

    while len(L) != 0:  # while there still neighbors to visit

        actual = L.pop() # pop the last neightbor in the list
        #neighbors = find_neighbors(actual,map)
        neighbors = find_neighbors(actual, map, neighbours_num=neighbours_num)
        for neighbor in neighbors:
            y,x = neighbor
            yact, xact = actual
            if map[y][x] == 0:
                map[y][x] = map[yact][xact] + 1
                L =  [(y,x)]   + L 

    return map

def remove_edges (map, edge_size = 3):
    
    height, width = map.shape

    map[0:edge_size, :] = 0
    map[width-edge_size:width,:] = 0
    map[:, 0:edge_size] = 0
    map[:,height -edge_size:height] = 0

    return map

def get_attraction_function(map, goal, z = 1, distance = "mht", apply_scaling = True, scale_factor = 10):
    
    """ 
    8D connectivity:            4D conectivity:

    | 1   1    1 |              | 0    1    0 |
    | 1  goal  1 |              | 1  goal   1 |
    | 1   1    1 |              | 0    1    0 |
    """

    height, width = map.shape
    ygoal, xgoal = goal
    attraction_grid = np.zeros((height, width))

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
    
    if apply_scaling == True:
        attraction_grid = attraction_grid/attraction_grid.max()*scale_factor

    return attraction_grid

def get_repulsive_function (map, Q = 4, eta = 15):
    height, width = map.shape

    repuslsive_grid = np.zeros((height, width))
    
    for i in range(height):
        for j in range( width):
            if map[i,j] > Q:
                repuslsive_grid [i,j] = 0
            else: 
                repuslsive_grid [i,j] = .5 * eta * ( (1/map[i,j]) - (1/Q) )**2
    return repuslsive_grid

def get_gradient_descent(map, q_start, neighbours_num = 8):

    E = .001
    y,x = q_start
    #gradient_map = copy.deepcopy(map)
    gradient_listx = []
    gradient_listy = []

    dq = 100
    c = 0
    
    while abs(dq)> E:
        neighbours = find_neighbors((y,x), map, neighbours_num)
        temp_neig_dq = []

        for i in neighbours:
            dq = map[i[0],i[1]] - map[y,x]
            temp_neig_dq.append(dq)  
            #gradient_map[i[0],i[1]] = dq

        dq = min(temp_neig_dq)
        y,x = neighbours[temp_neig_dq.index(dq)] # get the min dq on the neighbourhood, get the index of that neighbour, and use that index to get the position
        gradient_listx.append(x)
        gradient_listy.append(y)
        if c > 500:
            break

        c += 1
    
    return gradient_listx, gradient_listy

def wave_front(q_goal,n_neighbours,apply_scaling=True):
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
            neigh = [list(t) for t in find_neighbors([*p], wave_map,n_neighbours)] #convert list of tuples to nested list
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
    
    pathx = []
    pathy = []
    while q_start!=q_goal:
        neigh =find_neighbors(q_start,wave_map,neighbours_num)
        q_start = neigh[np.argmin([wave_map[n] for n in neigh])]
        pathx.append(q_start[1])
        pathy.append(q_start[0])

    return pathx,pathy

map_number = 0
num_neigbours_used = 8
Q_used = 50
save_path = f"FINAL_RESULTS/map{map_number}"

 
if map_number == 0:
    # goal = (90, 70)
    goal = (110, 40)
    start = (10, 10)
    # goal = (5,4) #for 10x10 grid
    # start = (0,0)
elif map_number == 1:
    goal = (90, 60)
    start = (60, 60)
elif map_number == 2:
    goal = (139, 38)
    start = (8, 31)
elif map_number == 3:
    goal = (375, 375)
    start = (50, 90)

# Load grid map 
image = Image.open('data/map'+str(map_number)+'.png').convert('L')
grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255
# binarize the image
grid_map[grid_map > 0.5] = 1
grid_map[grid_map <= 0.5] = 0

# Invert colors to make 0 -> free and 1 -> occupied
map = (grid_map * -1) + 1
map = remove_edges(map)
""""
plt.matshow(map)
plt.colorbar()
plt.show()
"""
######################### PART 1 #################################
dist_to_obstacle = bushfire(copy.deepcopy(map), neighbours_num=num_neigbours_used)
"""
plt.matshow(dist_to_obstacle)
plt.title(f"Bush-Fire Map {map_number}")
plt.colorbar()
plt.savefig(os.path.join(save_path,f"Bush-Fire_{map_number}_n_{num_neigbours_used}.png"))
plt.show()
"""

repulsive_grid = get_repulsive_function(dist_to_obstacle, Q = Q_used, eta = 15)
"""
plt.matshow(repulsive_grid)
plt.title(f"Repulsive Grid {map_number}")
plt.colorbar()
plt.savefig(os.path.join(save_path,f"repulsive_grid{map_number}_n_{num_neigbours_used}.png"))
plt.show()
"""

atraction_grid = get_attraction_function(map, goal,distance = "d2")
"""
plt.matshow(atraction_grid)
plt.title(f"Atraction Grid {map_number}")
plt.colorbar()
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.savefig(os.path.join(save_path,f"atraction_grid_{map_number}_n_{num_neigbours_used}.png"))
plt.show()
"""

potential = atraction_grid + repulsive_grid
"""
plt.matshow(potential)
plt.title(f"Total Potential {map_number}")
plt.colorbar()
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.savefig(os.path.join(save_path,f"potential_{map_number}_n_{num_neigbours_used}.png"))
plt.show()
"""
######################### PART 2 #################################

gradient_listx, gradient_listy = get_gradient_descent(potential, start, neighbours_num=num_neigbours_used)
"""
plt.matshow(potential)
plt.title(f"Gradient Descent {map_number}")
plt.colorbar()
plt.scatter(gradient_listx, gradient_listy, s=2, marker = "*", c="b")
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.savefig(os.path.join(save_path,f"gradient_{map_number}_n_{num_neigbours_used}_goal2.png"))
plt.show()
"""
######################### PART 3 #################################
wave_map = wave_front(goal,n_neighbours=num_neigbours_used,apply_scaling=True)
"""
plt.matshow(wave_map)
plt.title(f"Wave Front {map_number}")
plt.colorbar()
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.savefig(os.path.join(save_path,f"wave_map_{map_number}_n_{num_neigbours_used}.png"))
plt.show()
"""
pathx,pathy = find_path(wave_map, start,goal, neighbours_num=num_neigbours_used)
"""
plt.matshow(wave_map)
plt.title(f"Path Finder {map_number}")
plt.colorbar()
plt.scatter(pathx, pathy, s=1, marker = "*", c="b")
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.savefig(os.path.join(save_path,f"path_finder_{map_number}_n{num_neigbours_used}.png"))
plt.show()
"""
