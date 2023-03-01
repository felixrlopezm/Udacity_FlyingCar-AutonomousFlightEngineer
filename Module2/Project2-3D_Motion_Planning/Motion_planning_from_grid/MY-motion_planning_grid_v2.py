from enum import Enum, auto
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import argparse
import time
import msgpack
import numpy as np
import planning_utils_grid as pu


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, data, grid, north_offset, east_offset):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
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
        print("Arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("Takeoff transition")
        print('Takeoff altitude:', self.flight_altitude)
        self.takeoff(self.flight_altitude)

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("Waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('Target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("Landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("Disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("Manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print('Planning transition')
        print("Searching for a path ...")

        # Read in input data from command line input
        FLIGHT_ALTITUDE = args.flight_altitude
        OBSTACLE_MAP = args.obstacle_map
        GOAL_LON = args.goal_longitude
        GOAL_LAT = args.goal_latitude

        # Setting the takeoff altitude
        self.flight_altitude = FLIGHT_ALTITUDE

        # Reading lat0, lon0 from colliders.csv into floating point values
        lat0, lon0 = pu.extract_map_center(OBSTACLE_MAP)
        
        # Set Drone home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
        print('Home position in Global frame    (lon lat alt) [º º m]:', (self.global_home))

        # Retrieve current position in Global frame (lon lat alt) [º º m]
        global_position = self.global_position
        print('Current position in Global frame (lon lat alt) [º º m]:', (global_position))
 
        # Convert to current local position (N, E, D) [m m m]
        local_position = pu.global_to_local(global_position, self.global_home)
        print('Current position in Local frame  (N, E, D)     [m m m]:', local_position)

        #  Define start position as current position in local NED frame
        start = (int(local_position[0]) - north_offset, int(local_position[1]) - east_offset)

        # Define goal position in local NED frame
        global_goal = (GOAL_LON, GOAL_LAT, FLIGHT_ALTITUDE)     # Global coordinates 
        local_goal = pu.global_to_local(global_goal, self.global_home) # Local coordinates
        goal = (int(local_goal[0]) - north_offset, int(local_goal[1]) - east_offset)
        
        print("North offset = {0}, East offset = {1}".format(north_offset, east_offset))
        print('Start and Goal positions on the grid:', start, '/ ', goal)

        # Plan the path from grid with A* algoritum
        path, _ = pu.a_star(grid, pu.Euclidean_d, start, goal)
        
        # Prune path to minimize number of waypoints
        #pruned_path = pu.collinearity_pruning(path)
        pruned_path = pu.bresenham_pruning(path, grid)

        pu.plotting(grid, start, goal, path, pruned_path)

        # Convert pruned_path to waypoints
        self.waypoints = [[p[0] + north_offset, p[1] + east_offset, FLIGHT_ALTITUDE, 0] for p in pruned_path]
        print('Waypoints: ', self.waypoints)
        
        # Send waypoints to sim for visualization
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("Starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

if __name__ == "__main__":
    # Command-line interface
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")

    parser.add_argument('--obstacle_map', type=str, default='../colliders.csv', help='Name of the file containing the 2.5D Map of obstacles')
    parser.add_argument('--flight_altitude', type=int, default=15, help='Flight Altitude in meters [integer] (defaul 5 m)')
    parser.add_argument('--safety_distance', type=int, default=5, help='Safety distance in meteres [integer] (default 5 m)')
    
    # Goal location 1
    parser.add_argument('--goal_longitude', type=float, default=-122.399540, help='Goal Longitude in degrees [float]')
    parser.add_argument('--goal_latitude', type=float, default=37.796983, help='Goal Latitude in degrees [float]')

    # Goal location 2
    #parser.add_argument('--goal_longitude', type=float, default=-122.398112, help='Goal Longitude in degrees [float]')
    #parser.add_argument('--goal_latitude', type=float, default=37.793480, help='Goal Latitude in degrees [float]')

    args = parser.parse_args()

    # Read in obstacle data from 2.5D Map
    data = np.loadtxt(args.obstacle_map, delimiter=',', dtype='Float64', skiprows=2)
        
    # Create a grid at Mean Flight Altitude with defined safety margin around obstacles
    grid, north_offset, east_offset = pu.create_grid(data, args.flight_altitude, args.safety_distance)

    
    # Drone Connection
    print('Starting connection to Drone')
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, data, grid, north_offset, east_offset)
    time.sleep(1)

    drone.start()
