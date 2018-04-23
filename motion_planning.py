import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
#from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from udacidrone.frame_utils import local_to_global

import planning
import visdom
import sampling


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()
    RE_PLANNING = auto()
    RE_PLANNING_SKIP = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        # plotting
        self.v = visdom.Visdom()
        assert self.v.check_connection()
        self.v_t = 0
        self.v_ne_plot = None
        self.v_d_plot = None

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_POSITION, self.update_ne_plot)
        self.register_callback(MsgID.LOCAL_POSITION, self.update_d_plot)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        self.TARGET_ALTITUDE = 5
        self.SAFETY_DISTANCE = 5
        self.colliders_csv = 'colliders.csv'
        self.goal_ne = None  # Goal location
        self.path_planning = self.init_path_planning()

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        target = np.array([self.goal_ne[0], self.goal_ne[1]])
                        if np.linalg.norm(self.local_position[0:2] - target) > 2.:
                            print("Searching for a path ...")
                            self.flight_state = States.RE_PLANNING
                        else:
                            self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()
            elif self.flight_state == States.RE_PLANNING:
                self.path_replanning()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        if len(self.waypoints) > 0:
            self.target_position = self.waypoints.pop(0)
            print('target position', self.target_position)
            self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                              self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self, waypoints):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(waypoints)
        self.connection._master.write(data)

    # plot helpers
    def update_ne_plot(self):
        if self.v_ne_plot is not None:
            curr_local_position = global_to_local(self.global_position, self.global_home)
            self.v.line(np.array([curr_local_position[0]]),
                        X=np.array([curr_local_position[1]]),
                        win=self.v_ne_plot, update='append')

    def update_d_plot(self):
        if self.v_d_plot is not None:
            curr_local_position = global_to_local(self.global_position, self.global_home)
            d = np.array([curr_local_position[2]])
            self.v_t += 1
            self.v.line(d, X=np.array([self.v_t]), win=self.v_d_plot, update='append')

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        self.target_position[2] = self.TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = planning.get_latlog(self.colliders_csv)
        print('lat0: {} lon0: {}'.format(lat0, lon0))

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0.0)

        # TODO: retrieve current global position
        curr_global_position = self.global_position  # [lon, lat, alt]

        # TODO: convert to current local position using global_to_local()
        curr_local_position = global_to_local(curr_global_position, self.global_home)
        print('curr_local_position: {}'.format(curr_local_position))  # [north, east, down]

        # Plot D
        self.v_ne_plot = self.v.line(np.array([curr_local_position[0]]),
                                     X=np.array([curr_local_position[1]]),
                                     opts=dict(
                                         title="Local position (north, east)",
                                         xlabel='North',
                                         ylabel='East'
                                     ))
        self.v_t = 0
        self.v_d_plot = self.v.line(np.array([curr_local_position[2]]),
                                    X=np.array([self.v_t]),
                                    opts=dict(
                                        title="Altitude (meters)",
                                        xlabel='Timestep',
                                        ylabel='Down'
                                    ))
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        # data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        # grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        # print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)

        # TODO: convert start position to current position rather than map center
        start_position = (curr_local_position[0],
                          curr_local_position[1])  # N, E (x', y')

        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert
        #goal_lonlat = (-122.39687845, 37.79292773, self.TARGET_ALTITUDE)  # lon, lat, alt
        # pick random free location
        goal_lonlat = self.path_planning.sampler.sample_lonlat(self.global_home)
        goal_position = global_to_local(goal_lonlat, self.global_home)
        self.goal_ne = (goal_position[0], goal_position[1])
        print('goal_position: {}'.format(goal_position))

        # TDOO: send start and goal
        #test = [[int(start_position[0]), int(start_position[1]), self.TARGET_ALTITUDE, 0],
        #        [int(goal_position[0]),  int(goal_position[1]), self.TARGET_ALTITUDE, 0]]
        #self.send_waypoints(test)

        # Saminda Abeyruwan implementation requires height
        start_position = start_position + (self.TARGET_ALTITUDE,)
        goal_position = self.goal_ne + (self.TARGET_ALTITUDE,)

        self.path_plan_block(start_position, goal_position)

    def path_plan_block(self, start_position, goal_position):
        print('start_position: {}, goal_position: {}'.format(start_position, goal_position))

        rrt, x_goal, x_goal_state = self.path_planning.generate_RRT(start_position,
                                                                    goal_position,
                                                                    200, 10, 5., 0.1, 0.3)
        print('v: {} e: {} x: {}, x_state: {}'.format(len(rrt.vertices),
                                                      len(rrt.edges),
                                                      x_goal, x_goal_state))

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        # print('Local Start and Goal: ', grid_start, grid_goal)
        # path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        path, cost = self.path_planning.a_star(rrt.tree, start_position, x_goal)
        print('path: {} cost: {}'.format(len(path), cost))

        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        path = self.path_planning.prune_path(path)
        print('prune_path: {}'.format(len(path)))

        waypoints = [[p[0], p[1], self.TARGET_ALTITUDE, 0] for p in path]
        # printing needs int looks like
        print_waypoints = [[int(wp[0]), int(wp[1]), wp[2], wp[3]] for wp in waypoints]

        # Convert path to waypoints
        # waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints(print_waypoints)

    def path_replanning(self):
        if self.flight_state == States.RE_PLANNING_SKIP:
            return

        self.flight_state = States.RE_PLANNING_SKIP
        print("Path replanning ...")
        curr_local_position = global_to_local(self.global_position, self.global_home)
        print('curr_local_position: {}'.format(curr_local_position))  # [north, east, down]
        print('goal_ne: {}'.format(self.goal_ne))

        start_position = (curr_local_position[0],
                          curr_local_position[1],
                          self.TARGET_ALTITUDE)  # N, E (x', y')
        goal_position = self.goal_ne + (self.TARGET_ALTITUDE,)
        self.path_plan_block(start_position, goal_position)
        self.waypoint_transition()

    def init_path_planning(self):
        # Read in obstacle map
        data = np.loadtxt(self.colliders_csv, delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        sampler = sampling.Sampler(data,
                                   zmax=self.TARGET_ALTITUDE,
                                   safe_distance=self.SAFETY_DISTANCE)
        free_samples = sampler.samples(1000)

        print("North offset = {0}, east offset = {1}".format(sampler.nmin, sampler.emin))
        print("free_space_samples: {}".format(len(free_samples)))

        path_planning = planning.PathPlanning(sampler=sampler,
                                              free_samples=free_samples)
        return path_planning

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
