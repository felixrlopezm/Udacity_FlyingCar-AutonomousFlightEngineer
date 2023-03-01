from enum import Enum
from queue import PriorityQueue
from bresenham import bresenham
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import numpy.linalg as LA
import utm


def global_to_local(global_position, global_home):
    """
    Convert a global position (lon, lat, up) to a local position (north, east, down) relative to the home position.

    Returns:
        numpy array of the local position [north, east, down]
    """
    (east_home, north_home, _, _) = utm.from_latlon(global_home[1], global_home[0])
    (east, north, _, _) = utm.from_latlon(global_position[1], global_position[0])

    local_position = np.array([north - north_home, east - east_home, -global_position[2]])
    return local_position


def local_to_global(local_position, global_home):
    """
    Convert a local position (north, east, down) relative to the home position to a global position (lon, lat, up)

    Returns:
        numpy array of the global position [longitude, latitude, altitude]
    """
    (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(global_home[1], global_home[0])
    (lat, lon) = utm.to_latlon(east_home + local_position[1], north_home + local_position[0], zone_number, zone_letter)

    lla = np.array([lon, lat, -local_position[2]])
    return lla


def extract_map_center(file_name):
    ''' This function extracts map center coordinates (latitude, longitude)
        from first line of 2.5D map 'colliders.csv'
    '''
    with open(file_name) as f:             
        first_line = f.readline().rstrip()       # reading first line without trailing characters
    lat0, lon0 = first_line.split(', ')          # split using ', ' as delimiter
    lat0 = float(lat0.lstrip('lat0 '))           # extracting latitude as float
    lon0 = float(lon0.lstrip('lon0 '))           # extracting latitude as float
    print('Extracted latitude-longitude coordinates of the center of the map: ({},{})'.format(lat0,lon0))

    return lat0, lon0
    

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.

    Data contains obstacle data of a map arranged in six columns x, y, z 
    and 𝛿𝑥, 𝛿𝑦, 𝛿𝑧 such that:
        x -> NORTH
        y -> EAST
        z -> ALTITUDE (positive up, note the difference with NED coords)
    Each (x,y,z) coordinate is the center of an obstacle and (𝛿𝑥,𝛿𝑦,𝛿𝑧) is
    the half widths of the obstacle. 
    For example, an obstacle with (x=37,y=12,z=8) and (𝛿𝑥=5,𝛿𝑦=5,𝛿𝑧=8) is
    a 10 x 10 m obstacle that is 16 m high and is centered at (x,y)=(37,12)
    at a height of 8 m.

    Given a map like this, the free space in the (x,y) plane is a function
    of altitude.

    The function creates a 2D grid map at 1 metre resolution of the
    configuration space for a particular altitude, where each value is 
    assigned either a 0 or 1 representing feasible or infeasible 
    (obstacle) spaces respectively.

    Note that the function extends each obstacle by a safety margin to
    create the equivalent of a 3 dimensional configuration space.

    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))     #np.floor = rounding to lower integer (-2.3 --> -3)
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))      #np.ceil = rounding to upper integer (-2.3 --> -2)

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if drone_altitude <= (alt + d_alt + safety_distance):
            obstacle = [int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                        int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                        int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                        int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1))]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
    print('Grid created from 2.5D map')

    return grid, int(north_min), int(east_min)


class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid map and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or is an obstacle
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions


def a_star(grid, h, start, goal):
    ''' A* algorithm for grid maps
    '''

    print('Planner calculating path from start to goal....')

    path = []
    path_cost = 0
    queue = PriorityQueue()   # Priority queue for prioritizing expanding cells with lower cost
    queue.put((0, start))     # (cost, cell); cost value is use for priority
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        # Get and remove the first element from the queue
        item = queue.get()
        current_node = item[1]

        # Assignation of current cost
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        # If the current cell is at goal state, stop the search  
        if current_node == goal:        
            print('**********************')
            print('*** Found a path!! ***')
            print('**********************')
            found = True
            break
        else:
            # Iterate through each valid action at current cell:
            for action in valid_actions(grid, current_node):
                da = action.delta      # get delta-movement of performing the action
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                
                # Branch cost evaluation (action.cost + g)
                branch_cost = action.cost + current_cost
                # Heuristics distance evaluation
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost



def Euclidean_d(p1, p2):
    ''' Euclidean distance; Numpy version
    '''
    return np.linalg.norm(np.array(p1) - np.array(p2))


def collinearity_pruning(path):
    '''Prune the path using collinearity method
       If the 3 points are in a line, remove the 2nd point.
       The 3rd point now becomes and 2nd point and the check
       is redone with a new third point on the next iteration.'''

    def point(p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_check(p1, p2, p3, epsilon=1e-5):   
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    pruned_path = [p for p in path]   # Path to a list
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    print('Path pruned from {} points to {} points'.format(len(path), len(pruned_path)))
    return pruned_path
    
def bresenham_pruning(path, grid):
    '''Prune the path using Brensenham mehtod
       If the Brensenham line between a node and the next to the following one (n+2) in the path
       does not hit any obstacle, then the middle node (n+1) is removed from the path.
       '''    
    pruned_path = [p for p in path]   # Path to a list
    i = 0
    
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i + 2]
        
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for cell in cells:
            # Check if we're in collision
            if grid[cell[0], cell[1]] == 1:
                hit = True
                break

        # If the edge does not hit an obstacle, remove the middle point
        if not hit:
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    print('Path pruned from {} points to {} points'.format(len(path), len(pruned_path)))        
    return pruned_path


def plotting(grid, grid_start, grid_goal, path, pruned_path):
    ''' Plotting grid and path
    '''
    print('Saving the path in a .png file...')

    fig = plt.figure(figsize=(15,15))
    
    # Grid plotting
    plt.imshow(grid, cmap='Greys', origin='lower')

    # Start and Goal plotting
    plt.plot(grid_start[1], grid_start[0], 's', color='green')
    plt.plot(grid_goal[1], grid_goal[0], 's', color='red')
    # Original Path plotting
    if path is not None:
        pp = np.array(path)
        plt.plot(pp[:, 1], pp[:, 0], 'orange', label = 'Path')
    # Pruned Path plotting
    if pruned_path is not None:
        pp = np.array(pruned_path)
        plt.plot(pp[:, 1], pp[:, 0], 'blue', label = 'Pruned Path')
        plt.scatter(pp[:, 1], pp[:, 0])
    
    # Format
    plt.xlabel('EAST', fontsize=20)
    plt.ylabel('NORTH', fontsize=20)
    plt.suptitle('City Map planning from Grid', fontsize=20, x=0.5, y=0.93)
    plt.title('A* and final pruning with Bresenham method', fontsize=16, x=0.5, y=1.0)
    plt.legend(loc='upper right', fontsize=20)

    # Saving plot
    fig.savefig('Flight_planning_grid.png')
    print('Saved Flight planning as "Flight_planning_grid.png')

    return


