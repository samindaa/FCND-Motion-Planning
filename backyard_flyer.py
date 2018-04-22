import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    PRE_LANDING = 4  # Added a new State
    LANDING = 5
    DISARMING = 6


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.idx_waypoint = 0
        self.target_altitude = 5.0
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # Callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        print("local_position_callback")
        print("wpt: {} state: {}".format(self.idx_waypoint, self.flight_state))
        print("lpos: {}".format(self.local_position))
        if self.flight_state == States.TAKEOFF:
            if -self.local_position[-1] > self.target_position[-1] * 0.95:
                self.flight_state = States.WAYPOINT
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.local_position[0:2] - self.target_position[0:2]) < 1.0:
                self.waypoint_transition()
            # Since the project needs to disarm the drone after one cycle, we execute the following code.
            # Uncommenting the following 2-lines will cause the drone to fly forever!
            if self.idx_waypoint == 0:
                self.flight_state = States.PRE_LANDING
        elif self.flight_state == States.LANDING:
            if np.abs(self.global_position[-1] - self.global_home[-1]) < 0.1:
                if np.abs(self.local_position[-1]) < 0.1:
                    self.disarming_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        print("velocity_callback")
        print("wpt: {} state: {}".format(self.idx_waypoint, self.flight_state))
        if self.flight_state == States.PRE_LANDING and np.linalg.norm(self.local_velocity[0:2]) < 0.2:
            self.landing_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        print("state_callback")
        print("wpt: {} state: {}".format(self.idx_waypoint, self.flight_state))
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.calculate_box()
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed and not self.guided:
                self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        self.all_waypoints = [[10, 0, self.target_altitude],
                              [10, 10, self.target_altitude],
                              [0, 10, self.target_altitude],
                              [0, 0, self.target_altitude]]

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming_transition")
        print("wpt: {} state: {} gpos: {}".format(self.idx_waypoint, self.flight_state, self.global_position))

        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff_transition")
        print("wpt: {} state: {}".format(self.idx_waypoint, self.flight_state))
        self.target_position[-1] = self.target_altitude
        self.takeoff(self.target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint_transition")
        print("wpt: {} state: {}".format(self.idx_waypoint, self.flight_state))
        self.target_position = self.all_waypoints[self.idx_waypoint]
        print("wpt: {} target: {} altitude: {}".format(self.idx_waypoint,
                                                       self.target_position,
                                                       self.target_altitude))
        # Next waypoint.
        self.idx_waypoint = (self.idx_waypoint + 1) % len(self.all_waypoints)
        self.cmd_position(self.target_position[0],
                          self.target_position[1],
                          self.target_altitude,
                          0)

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing_transition")
        print("wpt: {} state: {}".format(self.idx_waypoint, self.flight_state))
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm_transition")
        print("wpt: {} state: {}".format(self.idx_waypoint, self.flight_state))
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual_transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL
        print("wpt: {} state: {}".format(self.idx_waypoint, self.flight_state))

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    # conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
