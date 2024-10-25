import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
import copy

def  find_neighbors(position, map, neighbors_num = 4):
    height, width = map.shape
    y,x = position
    neighbors = []

    if neighbors_num == 4:
        if x-1 >= 0 and map[y][x-1] == 0:
            neighbors.append((position[0],position[1]-1))
            
        if y-1 >= 0 and map[y-1][x] == 0:
            neighbors.append((position[0]-1,position[1]))

        if x+1 < width and map[y][x+1] == 0:
            neighbors.append((position[0],position[1]+1))
            
        if y+1 < height and map[y+1][x] == 0:
            neighbors.append((position[0]+1,position[1]))
    
    elif neighbors_num == 8:
        pass

    return neighbors 


def  find_neighbors2(position, map, neighbors_num = 4):
    height, width = map.shape
    y,x = position
    neighbors = []

    if neighbors_num == 4:
        if x-1 >= 0 :
            neighbors.append((position[0],position[1]-1))
            
        if y-1 >= 0:
            neighbors.append((position[0]-1,position[1]))

        if x+1 < width:
            neighbors.append((position[0],position[1]+1))
            
        if y+1 < height:
            neighbors.append((position[0]+1,position[1]))
    
    elif neighbors_num == 8:
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


def bushfire (map):

    L = list(np.argwhere(map == 1)) # get a list of all the elements in the map == 1

    while len(L) != 0:  # while there still neighbors to visit

        actual = L.pop() # pop the last neightbor in the list
        #neighbors = find_neighbors(actual,map)
        neighbors = find_neighbors(actual, map)
        for neighbor in neighbors:
            y,x = neighbor
            yact, xact = actual
            if map[y][x] == 0:
                map[y][x] = map[yact][xact] + 1
                L =  [(y,x)]   + L 
    
    return map

def remove_edges (map):
    # working on this    ##########################################################
    i = 0
    while map[i,i] == 1:
        i += 1
    
    map[0:i, :] = 0
    map[-i:-1,:] = 0
    map[:, 0:i] = 0
    map[:,-i:-1]

    return map

def get_attraction_function(map, goal, z = 1, distance = "mht", points = 4):
    
    """ 
    8D connectivity:            4D conectivity:

    | 1   1    1 |              | 0    1    0 |
    | 1  goal  1 |              | 1  goal   1 |
    | 1   1    1 |              | 0    1    0 |
    
    where:
        1 = neighbor
        0 = not neighboor

    """

    height, width = map.shape
    ygoal, xgoal = goal

    attraction_grid = np.zeros((height, width))

    if points == 4:

        if distance == "d2":
            """
            to calculate the attraction a quadratic potential function is used:

                           U_att (q) = 1/2 Z d^2 (q, q_goal)
            """

            for i in range(height):
                for j in range( width):
                    attraction_grid[i][j] = .5*z*((j - xgoal)**2 + (i - ygoal)**2)


        if distance == "mht":
            """
            to calculate the attraction a quadratic potential function is used:

                           U_att (q) = abs (delta in y) + abs (delta in x)
            """

            for i in range(height):
                for j in range( width):
                    attraction_grid[i][j] = abs((j - xgoal)) + abs((i - ygoal))

    if points == 8:
        
        pass


    return attraction_grid


def get_repulsive_function (map, Q, eta = 20000):
    height, width = map.shape

    repuslsive_grid = np.zeros((height, width))
    
    for i in range(height):
        for j in range( width):
            #if map[i,j] == 1:
                #pass
            if map[i,j] > Q:
                repuslsive_grid [i,j] = 0
            else: 
                repuslsive_grid [i,j] = .5 * eta * ((1/map[i,j]) - 1/Q)**2
    return repuslsive_grid

def get_gradient_descent(map, q_start):

    E = -5
    y,x = q_start
    gradient_map = copy.deepcopy(map)

    dq = 0

    while abs(dq)> E:
        neighbours = find_neighbors2(q_start, map, 4)
        temp_neig_dq = []

        for i in neighbours:
            dq = map[i[0],i[1]] - map[q_start[0],q_start[1]]
            temp_neig_dq.append(dq) 
            
            gradient_map[i[0],i[1]] = dq

        dq = min(temp_neig_dq)
        q_start = neighbours[temp_neig_dq.index(dq)] # get the min dq on the neighbourhood, get the index of that neighbour, and use that index to get the position

    return gradient_map



# Load grid map 
image = Image.open('data/map0.png').convert('L')
grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255

# binarize the image
grid_map[grid_map > 0.5] = 1
grid_map[grid_map <= 0.5] = 0

# Invert colors to make 0 -> free and 1 -> occupied
inv_grid_map = (grid_map * -1) + 1

####################################### testing the brush fire #######################################
"""
map = np.array([[0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,1,1,1,0,0,0,0],
                [0,0,0,1,1,1,0,0,0,0],
                [0,0,0,1,1,1,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0]])""" 


map = inv_grid_map
#map = remove_edges(map)

dist_to_obstacle = bushfire(map)
plt.matshow(dist_to_obstacle)
plt.colorbar()
plt.show()

repulsive_grid = get_repulsive_function(dist_to_obstacle, 4)
plt.matshow(repulsive_grid)
plt.colorbar()
plt.show()


####################################### testing the atraction #######################################

#map = np.zeros((100,100))
goal = (110, 40)
start = (10, 10)
atraction_grid = get_attraction_function(map, goal, distance = "d2")

#atraction_grid = ((atraction_grid * -1) + np.max(atraction_grid))/np.max(atraction_grid)

plt.matshow(atraction_grid)
plt.colorbar()
plt.show()


potential = atraction_grid + repulsive_grid
plt.matshow(potential)
plt.colorbar()
plt.show()

gradient_map = get_gradient_descent(map, start)

plt.matshow(gradient_map)
plt.colorbar()
plt.show()
