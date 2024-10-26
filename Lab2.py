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

"""
def find_neighbors(targ_coord,n, map):
    map_size = map.shape
    neighbours = []
    
    for x, y in [targ_coord]:
        # Check each of the four neighbors (up, down, left, right) and add valid ones to the list
        if n ==4: potential_neighbors = [
                        (x, y-1),  # Left
                        (x-1, y),  # Up
                        (x, y+1),  # Right
                        (x+1, y)  # Down
                    ]
        else: potential_neighbors = [
                        (x-1, y-1),  # Diagonal left Up
                        (x-1, y),  # Up
                        (x-1, y+1),  # Diagonal Right up
                        (x, y-1),  # Left
                        (x, y+1),  # Right
                        (x+1, y-1),  # Diag left Down
                        (x+1, y),  # Down
                        (x+1, y+1),  # Down rihght down
                    ]

        # Loop through each neighbor and check if it's within bounds and matches the value 'i'
        for nx, ny in potential_neighbors:
            if 0 <= nx < map_size[0] and 0 <= ny < map_size[1]:  # Boundary check
                if map[nx, ny] == 1:  # Check if the neighbor has the value 'i'
                    neighbours.append([nx, ny])
    
    return np.array(neighbours)"""


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
###############################################


def wave_front(q_goal,apply_scaling=True,n_neighbours=4):
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
    wave_map[np.where(wave_map == 1)]=900  #setting obs manually to max
    
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

####################################################

map_number = 3
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

# map = np.array([[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,1,1,0,0,0,0],[0,0,0,0,1,1,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0]])
# map = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
#                 [0, 0, 0, 0, 0, 0, 0,  0, 0, 0,],
#                 [0, 0,  1, 1,  1,  1,  1,  0,  0, 0,],
#                 [0, 0,  1,  1,  1,  1,  1,  0,  0,  0,],
#                 [0, 0,  1,  0,  0,  0,  0,  0,  0,  0,],
#                 [0, 0,  1,  0,  0,  0,  0,  0,  0,  0,],
#                 [0, 0,  1,  0,  0,  0,  0,  0,  0,  0,],
#                 [0, 0,  1,  0,  0,  0,  0,  0,  0,  0,],
#                 [0, 0,  1,  0,  0,  0,  0,  0,  0, 0,],
#                 [0,  0,  0,  0,  0,  0,  0,  0, 0, 0,]])

dist_to_obstacle = bushfire(copy.deepcopy(map), 8)
"""
plt.matshow(dist_to_obstacle)
plt.colorbar()
plt.show()
"""

repulsive_grid = get_repulsive_function(dist_to_obstacle, Q = 5, eta = 15)
"""
plt.matshow(repulsive_grid)
plt.colorbar()
plt.show()
"""

atraction_grid = get_attraction_function(map, goal,distance = "d2")
"""
plt.matshow(atraction_grid)
plt.colorbar()
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.show()
"""

potential = atraction_grid + repulsive_grid
"""
plt.matshow(potential)
plt.colorbar()
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.show()
"""

gradient_listx, gradient_listy = get_gradient_descent(potential, start, neighbours_num=4)
"""
plt.matshow(potential)
plt.colorbar()
plt.scatter(gradient_listx, gradient_listy, s=2, marker = "*", c="b")
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.show()
"""

wave_map = wave_front(goal,True)
plt.matshow(wave_map)
plt.colorbar()
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.savefig(f"wave_front{map_number}.png")
plt.show()

pathx,pathy = find_path(wave_map, start,goal, neighbours_num=4)
plt.matshow(wave_map)
plt.colorbar()
plt.scatter(pathx, pathy, s=1, marker = "*", c="b")
plt.scatter(goal[1], goal[0], c="r",  marker=(5, 1))
plt.savefig(f"find_path_wave_front{map_number}.png")
plt.show()
