import numpy as np
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point
from udacidrone.frame_utils import local_to_global


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


class Sampler:

    def __init__(self, data, zmax=10, safe_distance=0):
        self._polygons = self.extract_polygons(data, safe_distance)
        self._xmin = np.min(data[:, 0] - data[:, 3])
        self._xmax = np.max(data[:, 0] + data[:, 3])

        self._ymin = np.min(data[:, 1] - data[:, 4])
        self._ymax = np.max(data[:, 1] + data[:, 4])

        self._zmin = 0
        # limit z-axis
        self._zmax = zmax
        # Record maximum polygon dimension in the xy plane
        # multiply by 2 since given sizes are half widths
        # This is still rather clunky but will allow us to 
        # cut down the number of polygons we compare with by a lot.
        self._max_poly_xy = 2 * np.max((data[:, 3], data[:, 4]))
        centers = np.array([p.center for p in self._polygons])
        self._tree = KDTree(centers, metric='euclidean')

    def extract_polygons(self, data, safe_distance):

        polygons = []
        for i in range(data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = data[i, :]

            obstacle = [north - d_north - safe_distance,
                        north + d_north + safe_distance,
                        east - d_east - safe_distance,
                        east + d_east + safe_distance]
            corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]),
                       (obstacle[1], obstacle[2])]

            # TODO: Compute the height of the polygon
            height = alt + d_alt

            p = Poly(corners, height)
            polygons.append(p)

        return polygons

    # def sample(self, num_samples):
    #     """Implemented with a k-d tree for efficiency."""
    #     xvals = np.random.uniform(self._xmin, self._xmax, num_samples)
    #     yvals = np.random.uniform(self._ymin, self._ymax, num_samples)
    #     zvals = np.random.uniform(self._zmin, self._zmax, num_samples)
    #     samples = list(zip(xvals, yvals, zvals))
    #
    #     pts = []
    #     for s in samples:
    #         in_collision = False
    #         idxs = list(self._tree.query_radius(np.array([s[0], s[1]]).reshape(1, -1), r=self._max_poly_xy)[0])
    #         if len(idxs) > 0:
    #             for ind in idxs:
    #                 p = self._polygons[int(ind)]
    #                 if p.contains(s) and p.height >= s[2]:
    #                     in_collision = True
    #         if not in_collision:
    #             pts.append(s)
    #
    #     return pts

    def norm(self, x1, x2):
        return np.linalg.norm(np.array([x1[0], x1[1]]) - np.array([x2[0], x2[1]]))

    def samples(self, num_samples):
        pts = []
        while len(pts) < num_samples:
            in_free_space = False
            while not in_free_space:
                s = self.sample()
                idxs = self._tree.query([[s[0], s[1]]], k=3, return_distance=False)[0]
                if len(idxs) > 0:
                    in_collision = False
                    for ind in idxs:
                        p = self._polygons[int(ind)]
                        if p.contains(s) and p.height >= s[2]:
                            in_collision = True
                    if not in_collision:
                        pts.append(s)
                        in_free_space = True
        return pts

    def sample(self):
        xval = np.random.uniform(self._xmin, self._xmax, 1)[0]
        yval = np.random.uniform(self._ymin, self._ymax, 1)[0]
        return [xval, yval, self._zmax]

    def sample_lonlat(self, global_home):
        tmp = self.sample()
        lla = local_to_global([tmp[0], tmp[1], -tmp[2]], global_home)
        return (lla[0], lla[1], self._zmax)

    @property
    def zmax(self):
        return self._zmax

    @property
    def nmin(self):
        return self._xmin

    @property
    def emin(self):
        return self._ymin

    @property
    def nmax(self):
        return self._xmax

    @property
    def emax(self):
        return self._ymax

    @property
    def tree(self):
        return self._tree

    @property
    def polygons(self):
        return self._polygons
