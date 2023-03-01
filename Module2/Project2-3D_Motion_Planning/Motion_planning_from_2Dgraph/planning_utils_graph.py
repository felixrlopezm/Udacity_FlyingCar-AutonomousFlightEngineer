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
    
def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """
    print('Creating Grid....')
    
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Initialize an empty list for Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                        int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                        int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                        int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1))]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    print('Grid created from 2.5D map')
    print('Creating Edges....')

    # Create a voronoi graph based on location of obstacle centres
    graph = Voronoi(points)

    # Check each edge from graph.ridge_vertices for collision with Bresenham method
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    print('Edges created using Voronoi method')

    return grid, edges, int(north_min), int(east_min)


def closest_point(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    closest_point = None
    dist = 10000000
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        if d < dist:
            closest_point = p
            dist = d
    return closest_point

def create_graph(edges, start, goal):
    # Create the graph with the weight of the edges set to the Euclidean distance between the points
    graph = nx.Graph()     # Graph creation

    for edge in edges:  # for every edge in the Voronoi graph
        # extract the ends of the edge
        p1 = edge[0]
        p2 = edge[1]
    
        # calculate their euclidean distance between ends
        dist = LA.norm(np.array(p2) - np.array(p1))
    
        # add the edge to the NetworkX graph with a weight given by its euclidean distance
        graph.add_edge(p1, p2, weight=dist)

    start_closest = closest_point(graph, start)
    #dist = LA.norm(np.array(start_closest) - np.array(start))
    graph.add_edge(start, start_closest, weight=0)

    goal_closest = closest_point(graph, goal)
    #dist = LA.norm(np.array(goal_closest) - np.array(goal))
    graph.add_edge(goal, goal_closest, weight=0)

    print('Graph created')

    return graph

def a_star_graph(graph, h, start, goal):
    ''' A* implementation for dealing with graphs
    '''
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
        
        # If the current cell corresponds to the goal state, stop the search
        if current_node == goal:        
            print('**********************')
            print('*** Found a path!! ***')
            print('**********************')
            found = True
            break
        else:
            for next_node in graph[current_node]:               # For every node connected to the current node
                cost = graph.edges[current_node, next_node]['weight']   # extract cost(=weitgh value) to get there
                
                # Branch cost evaluation (action.cost + g)
                branch_cost = current_cost + cost
                
                # Heuristics distance evaluation
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node)
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
    print('Path pruned from {} points to {} points using collinearity method'.format(len(path), len(pruned_path)))
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
    print('Path pruned from {} points to {} points using Bresenham method'.format(len(path), len(pruned_path)))        
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
    plt.suptitle('City Map planning from Graph', fontsize=20, x=0.5, y=0.93)
    plt.title('A* and final pruning with Bresenham method', fontsize=16, x=0.5, y=1.0)
    plt.legend(loc='upper right', fontsize=20)

    # Saving plot
    fig.savefig('Flight_planning_graph.png')
    print('Saved Flight planning as "Flight_planning_graph.png')

    return


