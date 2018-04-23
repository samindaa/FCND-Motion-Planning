import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
from planning_utils import a_star, heuristic, create_grid
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
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5
        colliders_csv = 'colliders.csv'

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = planning.get_latlog(colliders_csv)
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
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(curr_local_position[0]) - north_offset,
                      int(curr_local_position[1]) - east_offset) # N, E (x', y')

        # Set goal as some arbitrary position on the grid
        #grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_lonlat = (-122.39687845, 37.79292773, TARGET_ALTITUDE) # lon, lat, alt
        goal_ned = global_to_local(goal_lonlat, self.global_home)
        print('goal_ned: {}'.format(goal_ned))

        grid_goal = (int(goal_ned[0]) - north_offset,
                     int(goal_ned[1]) - east_offset)


        # Saminda Abeyruwan implementation based on RRT
        sampler = sampling.Sampler(data,
                                   zmax=TARGET_ALTITUDE,
                                   safe_distance=SAFETY_DISTANCE)
        free_samples = sampler.samples(1000)

        print("free_space_samples: {}".format(len(free_samples)))

        path_planning = planning.PathPlanning(sampler=sampler,
                                              free_samples=free_samples)
        # TODO(saminda): param?
        dt = 0.1
        v = 5.
        # my implementation requires height
        grid_start_rrt = grid_start + (TARGET_ALTITUDE,)
        grid_goal_rrt = grid_goal + (TARGET_ALTITUDE,)
        rrt, x_grid, x_grid_state = path_planning.generate_RRT(grid_start_rrt,
                                                               grid_goal_rrt,
                                                               200, 10, v, dt, 0.3)
        print('v: {} e: {} x: {}, x_state: {}'.format(len(rrt.vertices),
                                                      len(rrt.edges),
                                                      x_grid, x_grid_state))

        h_path, cost = path_planning.a_star(rrt.tree, grid_start_rrt, x_grid)
        print('h_path: {} cost: {}'.format(len(h_path), cost))
        h_path = path_planning.prune_path(h_path)
        print('h_prune_path: {}'.format(len(h_path)))

        test_waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in h_path]
        # to print
        test_waypoints = [[int(wp[0]), int(wp[1]), wp[2], wp[3]] for wp in test_waypoints]
        self.send_waypoints(test_waypoints)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        #print('Local Start and Goal: ', grid_start, grid_goal)
        #path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        ## todo:
        waypoints = []
        #waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        #self.send_waypoints(self.waypoints)

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
