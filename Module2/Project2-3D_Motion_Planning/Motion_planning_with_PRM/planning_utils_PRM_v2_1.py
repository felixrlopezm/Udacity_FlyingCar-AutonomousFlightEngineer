from enum import Enum
from queue import PriorityQueue
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
    The coordinates are given relative to the center of the map. Its actual
    location is given in the first row of the file.

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
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))  #np.floor = rounding to lower integer (-2.3 --> -3)
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))   #np.ceil = rounding to upper integer (-2.3 --> -2)

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


class Poly:
    def __init__(self, coords, height):
        self._polygon = Polygon(coords)
        self._height = height

    @property
    def height(self):
        return self._height

    @property
    def coords(self):
        return list(self._polygon.exterior.coords)[:-1]
    
    @property
    def area(self):
        return self._polygon.area

    @property
    def center(self):
        return (self._polygon.centroid.x, self._polygon.centroid.y)

    def contains(self, point):
        point = Point(point)
        return self._polygon.contains(point)

    def crosses(self, other):
        return self._polygon.crosses(other)


def extract_polygons(data, s_d):
    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        # Safety distance added to obstacle definition
        obstacle = [north - d_north - s_d, north + d_north + s_d, east - d_east - s_d, east + d_east + s_d]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]),
                   (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]
        
        # Height of the polygon
        height = alt + d_alt + s_d

        p = Poly(corners, height)
        polygons.append(p)

    return polygons


class Sampler:
    def __init__(self, data, safety_distance, min_flight_alt, max_flight_alt):
        self._polygons = extract_polygons(data, safety_distance)
        self._xmin = np.min(data[:, 0] - data[:, 3])
        self._xmax = np.max(data[:, 0] + data[:, 3])

        self._ymin = np.min(data[:, 1] - data[:, 4])
        self._ymax = np.max(data[:, 1] + data[:, 4])

        # limit z-axis
        self._zmin = min_flight_alt   # minimum flight altitude to avoid ground
        self._zmax = max_flight_alt
        # Record maximum polygon dimension in the xy plane
        # multiply by 2 since given sizes are half widths
        # This is still rather clunky but will allow us to 
        # cut down the number of polygons we compare with by a lot.
        self._max_poly_xy = 2 * np.max((data[:, 3], data[:, 4]))
        centers = np.array([p.center for p in self._polygons])
        self._tree = KDTree(centers, metric='euclidean')

    def sample(self, nodes_to_sample):
        print('Sampling {} nodes'.format(nodes_to_sample))
        """Implemented with a k-d tree for efficiency."""
        xvals = np.random.uniform(self._xmin, self._xmax, nodes_to_sample)
        yvals = np.random.uniform(self._ymin, self._ymax, nodes_to_sample)
        zvals = np.random.uniform(self._zmin, self._zmax, nodes_to_sample)
        samples = list(zip(xvals, yvals, zvals))

        pts = []
        for s in samples:
            in_collision = False
            idxs = list(self._tree.query_radius(np.array([s[0], s[1]]).reshape(1, -1), r=self._max_poly_xy)[0])
            if len(idxs) > 0:
                for ind in idxs: 
                    p = self._polygons[int(ind)]
                    if p.contains(s) and p.height >= s[2]:
                        in_collision = True
            if not in_collision:
                pts.append(s)
                
        return pts

    @property
    def polygons(self):
        return self._polygons


def can_connect(n1, n2, polygons):
    ''' This function checks whether a connection between two points 
        it is possible or not. It is not possible if the line that
        links the points crosses a polygon
        '''
    line = LineString([n1, n2])   # Creation of lineString object
    for poly in polygons:        
        if poly.crosses(line) and poly.height >= min(n1[2], n2[2]): 
            return False
    return True


def Euclidean_d(position, goal_position):
    ''' Euclidean distance; Numpy version
    '''
    return np.linalg.norm(np.array(position) - np.array(goal_position))


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


def create_probabilistic_roadmap(data, grid, min_flight_alt, max_flight_alt, nodes_to_sample, s_d):

    # Initialize sampler object from Sample class with 'data'
    sampler = Sampler(data, s_d, min_flight_alt, max_flight_alt)

    # Sampling random nodes and removing those conflicting with obstacles using KDtree
    nodes = sampler.sample(nodes_to_sample)

    print('{} of {} nodes do not collide with any obstacle'.format(len(nodes),nodes_to_sample))

    print('Creating graph from not-colliding sampled nodes...')

    # Create polygons from sampler object
    polygons = sampler._polygons

    # 3D graph creation;
    graph = nx.Graph()       # Graph creation
    tree = KDTree(nodes)     # Tree creation
    
    for node in nodes:   # for each node, identify the indexes of the k nearest nodes
        idxs = tree.query([node], k=10, return_distance=False)[0]
        
        for idx in idxs: # for each nearest node
            node_b = nodes[idx]
            if node_b == node:     # Skip node_b if identical
                continue
            if can_connect(node, node_b, polygons):
                graph.add_edge(node, node_b, weight = 1)
                
    print('Graph created with {} nodes joined with {} edges'.format(len(nodes), len(graph.edges)))

    return graph, polygons

def path_pruning(path, polygons):
    '''Prune the path using Brensenham mehtod
       If the line between a node and the next to the following one (n+2) in the path
       does not hit any obstacle, then the middle node (n+1) is removed from the path.
       '''    
    pruned_path = [p for p in path]   # Path to a list
    i = 0
    
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i + 2]

        if can_connect(p1, p2, polygons):
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1

    print('Path pruned from {} nodes to {} nodes'.format(len(path), len(pruned_path)))        
    return pruned_path


# Plotting functions

def plotting_path(data, grid, graph, path, pruned_path, start, goal):
    print('Saving the path in a .png file...')
    # Plotting
    fig = plt.figure(figsize=(15,15))
    
    # Grid plotting
    plt.imshow(grid, cmap='Greys', origin='lower')

    # Offset values for plotting
    nmin = np.floor(np.min(data[:, 0] - data[:, 3]))
    emin = np.floor(np.min(data[:, 1] - data[:, 4]))

    # Draw nodes
    #for n1 in graph.nodes:
    #    plt.scatter(n1[1] - emin, n1[0] - nmin, c='grey')
        
    # Draw edges
    #for (n1, n2) in graph.edges:
    #    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'grey', label = 'Edges')
        
    # Draw path
    path_pairs = zip(path[:-1], path[1:])
    for (n1, n2) in path_pairs:
        plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'orange', marker='o', label = 'Path')

    # Draw pruned path
    path_pairs = zip(pruned_path[:-1], pruned_path[1:])
    for (n1, n2) in path_pairs:
        plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'blue', label = 'Pruned Path')

    # Start and Goal plotting
    plt.plot(start[1] - emin, start[0] - nmin, 's', color='green')
    plt.plot(goal[1] - emin, goal[0] - nmin, 's', color='red')

    # Format
    plt.xlabel('EAST', fontsize=20)
    plt.ylabel('NORTH', fontsize=20)
    plt.suptitle('City Map planning using Probabilistic RoadMap', fontsize=20, x=0.5, y=0.93)
    plt.title('(City Map at ground level)', fontsize=16, x=0.5, y=1.0)
    plt.legend(loc='lower right', fontsize=20)

    # Saving plot
    fig.savefig('Flight_planning_PRM.png')
    print('Saved Flight planning as "Flight_planning_PRM.png"')



