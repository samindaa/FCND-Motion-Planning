# Saminda Abeyruwan create_grid implementation.
import numpy as np


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))
    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
    #north_min_center = np.min(data[:, 0])
    #east_min_center = np.min(data[:, 1])
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        # TODO: Determine which cells contain obstacles
        # and set them to 1.
        #
        # Example:
        #
        #    grid[north_coordinate, east_coordinate] = 1
        if alt + d_alt + safety_distance > drone_altitude:
            north_min_b = north - d_north - safety_distance
            north_max_b = north + d_north + safety_distance
            east_min_b = east - d_east - safety_distance
            east_max_b = east + d_east + safety_distance

            north_min_b -= north_min
            north_max_b -= north_min
            east_min_b -= east_min
            east_max_b -= east_min

            north_min_b = np.ceil(north_min_b * (north_size - 1) / north_size)
            north_max_b = np.ceil(north_max_b * (north_size - 1) / north_size)
            east_min_b = np.ceil(east_min_b * (east_size - 1) / east_size)
            east_max_b = np.ceil(east_max_b * (east_size - 1) / east_size)

            north_min_b = int(np.clip(north_min_b, 0, north_size - 1))
            north_max_b = int(np.clip(north_max_b, 0, north_size - 1))
            east_min_b = int(np.clip(east_min_b, 0, east_size - 1))
            east_max_b = int(np.clip(east_max_b, 0, east_size - 1))

            grid[north_min_b:north_max_b + 1, east_min_b:east_max_b + 1] = 1

    return grid