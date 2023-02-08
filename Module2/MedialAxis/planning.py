from enum import Enum
from queue import PriorityQueue
import numpy as np


class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
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
    Returns a list of valid actions given a grid and current node.
    """
    valid_act = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_act.remove(Action.UP)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_act.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_act.remove(Action.LEFT)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_act.remove(Action.RIGHT)

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_act.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_act.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_act.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_act.remove(Action.SOUTH_EAST)

    return valid_act


def a_star(grid, h, start, goal):
    '''A* algorithm
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
            print('Found a path.')
            found = True
            break
        else:
            # Get the new valid nodes connected to the current node
            valid = valid_actions(grid, current_node)
            # Iterate through each of the valid actions and:
            for action in valid:
                da = action.delta    # get delta-movement of performing the action
                next_node = (current_node[0] + da[0], current_node[1] + da[1])

                # Branch cost evaluation (action.cost + g)
                branch_cost = current_cost + action.cost

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

