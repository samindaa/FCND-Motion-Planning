# planning.py contains my contributions to the project.

import re
import numpy as np
import networkx as nx
from sklearn.neighbors import KDTree
from queue import PriorityQueue


class RRT:
    def __init__(self, x_init):
        # A tree is a special case of a graph with
        # directed edges and only one path to any vertex.
        self.tree = nx.DiGraph()
        self.tree.add_node(tuple(x_init))

    def add_vertex(self, x_new):
        self.tree.add_node(tuple(x_new))

    def add_edge(self, x_near, x_new, u):
        self.tree.add_edge(tuple(x_near), tuple(x_new), orientation=u)

    @property
    def vertices(self):
        return self.tree.nodes()

    @property
    def edges(self):
        return self.tree.edges()


class PathPlanning:
    def __init__(self, sampler, free_samples):
        self._sampler = sampler
        self._free_samples = free_samples

    def select_input(self, x_rand, x_near):
        x = x_rand[0] - x_near[0]
        y = x_rand[1] - x_near[1]
        return np.arctan2(y, x)

    # def simulate(self, state, angle, v, dt):
    #     x = state[0]
    #     y = state[1]
    #     theta = state[2]
    #
    #     nx = x + v * np.cos(theta) * dt
    #     ny = y + v * np.sin(theta) * dt
    #     ntheta = theta + v * np.tan(angle) * dt
    #     return [nx, ny, ntheta]

    # def steer(self, x1, x2):
    #     theta = x1[2]
    #     angle = np.arctan2(x2[1] - x1[1], x2[0] - x1[0]) - theta
    #     angle = np.arctan2(np.sin(angle), np.cos(theta))
    #     # angle = np.random.normal(angle, ANGLE_STDDEV)
    #     # angle = np.clip(angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)
    #     return angle

    def nearest_neighbor(self, x_rand, rrt):
        vertices = list(rrt.vertices)
        tree = KDTree(vertices)
        inds = tree.query([x_rand], k=1, return_distance=False)[0]
        return vertices[inds[0]]

    def new_state(self, x_near, u, v, dt):
        return [x_near[0] + v * np.cos(u) * dt,
                x_near[1] + v * np.sin(u) * dt,
                self._sampler.zmax]

    def step_from_to(self, num_extensions, x_near, x_rand, x_goal, angle, v, dt):
        if self._sampler.norm(x_near, x_rand) < 2.:
            if not self.collision(x_rand):
                return x_rand
            else:
                return None

        states = [x_near]
        for _ in range(num_extensions):
            state = self.new_state(states[-1], angle, v, dt)
            if self.collision(state):
                break
            states.append(state)
            #if self._sampler.norm(state, x_goal) < 1.:
            #    break

        if len(states) > 1:
            return [states[-1][0], states[-1][1], self._sampler.zmax]
        else:
            return None

    def collision(self, s):
        in_collision = False
        idxs = self._sampler.tree.query([[s[0], s[1]]], return_distance=False)[0]
        if len(idxs) > 0:
            p = self._sampler.polygons[idxs[0]]
            if p.contains(s) and p.height >= s[2]:
                in_collision = True
        return in_collision

    def generate_RRT(self, x_init, x_goal, num_vertices, num_extensions, v, dt, epsilon=0.1):
        rrt = RRT(x_init)
        # Nearest node to the goal
        x_goal_near = x_init

        if self._sampler.norm(x_init, x_goal) < 1.:
            print('rrt_goal')
            rrt.add_edge(x_init, x_goal, 0)
            return rrt, x_goal, True

        for i in range(num_vertices):
            # print("i:{}".format(i))
            if np.random.uniform() < epsilon:
                idx = np.random.randint(0, len(self._free_samples), 1)[0]
                # print("\tidx:{}".format(idx))
                x_rand = self._free_samples[idx]
            else:
                x_rand = x_goal
                # print("\tto_goal")

            x_near = self.nearest_neighbor(x_rand, rrt)
            # print("\tx_near:{}".format(x_near))
            u = self.select_input(x_rand, x_near)
            # print("\tu:{}".format(u))
            x_new = self.step_from_to(num_extensions, x_near, x_rand, x_goal, u, v, dt)

            if not x_new is None:
                # the orientation `u` will be added as metadata to
                # the edge
                # rrt.add_edge(x_near, x_new, u)
                rrt.add_edge(x_near, x_new,
                             self._sampler.norm(x_near, x_new))
            # else:
            #    print("\tNone")

            x_goal_near = self.nearest_neighbor(x_goal, rrt)
            # if there is a node in node close to goal, stop searching
            if self._sampler.norm(x_goal_near, x_goal) < 2.:
                rrt.add_edge(x_goal_near, x_goal, 0)
                return rrt, x_goal, True

        return rrt, x_goal_near, False

    def heuristic(self, n1, n2):
        return self._sampler.norm(n2, n1)

    def a_star(self, di_graph, start, goal):
        """Modified A* to work with NetworkX DiGraphs."""

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
                print('Found a path; current_node: {} goal: {}'.format(current_node, goal))
                found = True
                break
            else:
                for next_node in di_graph.succ[current_node]:
                    cost = di_graph.edges[current_node, next_node]['orientation']
                    branch_cost = current_cost + cost
                    queue_cost = branch_cost + self.heuristic(next_node, goal)

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

    def point(self, p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_check(self, p1, p2, p3, epsilon=1e-6):
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    def prune_path(self, path):
        """
        Prune the given path. Based on the class material.

        :param path tuples in grid coordinates:
        :return pruned path:
        """
        pruned_path = [p for p in path]

        i = 0
        while i < len(pruned_path) - 2:
            p1 = self.point(pruned_path[i])
            p2 = self.point(pruned_path[i + 1])
            p3 = self.point(pruned_path[i + 2])

            # If the 3 points are in a line remove
            # the 2nd point.
            # The 3rd point now becomes and 2nd point
            # and the check is redone with a new third point
            # on the next iteration.
            if self.collinearity_check(p1, p2, p3):
                # Something subtle here but we can mutate
                # `pruned_path` freely because the length
                # of the list is check on every iteration.
                pruned_path.remove(pruned_path[i + 1])
            else:
                i += 1
        return pruned_path

    @property
    def sampler(self):
        return self._sampler


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
