import re
from enum import Enum
from queue import PriorityQueue
# from Queue import PriorityQueue
import numpy as np
from udacidrone.frame_utils import local_to_global


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

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

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1.0)
    EAST = (0, 1, 1.0)
    NORTH = (1, 0, 1.0)
    SOUTH = (-1, 0, 1.0)
    # Four diagonal directions
    NORTH_EAST = (1, 1, np.sqrt(2))
    NORTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_WEST = (-1, -1, np.sqrt(2))
    SOUTH_EAST = (-1, 1, np.sqrt(2))

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
    valid_actions = []
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle
    for action in list(Action):
        i, j = x + action.delta[0], y + action.delta[1]
        if 0 <= i < grid.shape[0] and 0 <= j < grid.shape[1] and grid[i, j] == 0:
            valid_actions.append(action)

    return valid_actions


def a_star(grid, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
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


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def get_latlog(fname):
    """
    Read fname to extract lat and log of the map.
    :param fname:
    :return: lat 0 and lon0
    """
    with open(fname) as f:
        line = f.readline()
    matches = re.match(r'lat0 (.*), lon0 (.*)', line)
    assert matches
    return float(matches.group(1)), float(matches.group(2))  # Lat, Lon


def random_free_location_in_grid(grid):
    while True:
        i, j = np.random.randint(grid.shape[0], size=1)[0], np.random.randint(grid.shape[1], size=1)[0]
        if grid[i, j] == 0:
            return (i, j)


def random_free_location_lla(grid, north_offset, east_offset, altitude, global_home):
    i, j = random_free_location_in_grid(grid)
    tmp = (i + north_offset, j + east_offset)
    lla = local_to_global([tmp[0], tmp[1], -altitude], global_home)
    return (lla[0], lla[1], altitude)


def can_connect_segment(grid, start, end):
    """
    Calculate line segment connecting p1 and p2 in the free space.
    Based on: http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm#Python
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    # points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        # TODO(saminda)
        if grid[coord[0], coord[1]] == 1:
            return False
        # points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    # if swapped:
    #    points.reverse()
    # return points
    return True


def smooth_path(grid, path):
    """
    Get smooth path. We are using an divide and conqur algorithm
    Motion Planning using Adaptive Random Walks
    """
    smoothed_path = []
    smooth_path_dq(grid, path, 0, len(path) - 1, smoothed_path)
    return smoothed_path


def smooth_path_dq(grid, path, first, last, S):
    if first == last:
        S.append(path[first])
    elif first == last - 1:
        S.append(path[first])
        S.append(path[last])
    elif can_connect_segment(grid, path[first], path[last]):
        S.append(path[first])
        S.append(path[last])
    else:
        mid = (first + last) // 2
        smooth_path_dq(grid, path, first, mid, S)
        smooth_path_dq(grid, path, mid + 1, last, S)


def greedy_smooth_path(grid, path, k=5):
    """
    Greedy look for connect forward nodes if possible
    """
    if len(path) < 3:
        return path

    pruned_path = []
    i = 0
    while i < len(path) - 1:
        pruned_path.append(path[i])
        j = i + 1
        ii = -1
        while j < min(j + k, len(path) - 1):
            if can_connect_segment(grid, path[i], path[j]):
                ii = j
            j += 1
        if ii != -1:
            i = ii
        else:
            i += 1
    pruned_path.append(path[-1])
    return pruned_path
