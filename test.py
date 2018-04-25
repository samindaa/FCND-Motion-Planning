# testing offline
import planning_utils
import matplotlib.pyplot as plt
import numpy as np


def main():
    print('main')
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
    grid, north_offset, east_offset = planning_utils.create_grid(data, 5, 5)
    print('data: {}'.format(grid.shape))
    #grid_start = (-north_offset, -east_offset)
    # grid_goal   = (-north_offset + 50, -east_offset + 50)
    grid_start = planning_utils.random_free_location_in_grid(grid)
    grid_goal = planning_utils.random_free_location_in_grid(grid)
    print('grid_start: {} grid_goal: {}'.format(grid_start, grid_goal))

    path, path_cost = planning_utils.a_star(grid, planning_utils.heuristic, grid_start, grid_goal)
    print('path: {} path_cost: {}'.format(len(path), path_cost))
    # path = planning_utils.prune_path(path)
    # print("pruned_path: {}".format(len(path)))
    smoothed_path = planning_utils.smooth_path(grid, path)
    print("smoothed_path: {}".format(len(smoothed_path)))

    greedy_path = planning_utils.greedy_smooth_path(grid, smoothed_path)
    print('greedy_path: {}'.format(len(greedy_path)))

    if True:
        plt.imshow(grid, cmap='Greys', origin='lower')

        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

        for i in range(len(smoothed_path)-1):
            p1 = smoothed_path[i]
            p2 = smoothed_path[i+1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'g-')

        for i in range(len(greedy_path)-1):
            p1 = greedy_path[i]
            p2 = greedy_path[i+1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')

        # plot in x and y
        plt.plot(grid_start[1], grid_start[0], 'rx')
        plt.plot(grid_goal[1], grid_goal[0], 'gx')

        # plt.plot([grid_start[1], grid_goal[1]], [grid_start[0], grid_goal[0]], 'g-')

        # zoom up to the area
        # plt.xlim((grid_start[1] - 10, grid_goal[1] + 10))
        # plt.ylim((grid_start[0] - 10, grid_goal[0] + 10))

        plt.xlabel('EAST')
        plt.ylabel('NORTH')
        #plt.show()
        plt.rcParams['figure.figsize'] = 12, 12
        plt.savefig('misc/path2.png')


if __name__ == "__main__":
    main()
