# Backyard Flyer project
#
# FLYING CAR AND AUTONOMOUS FLIGHT ENGINEER nanodegree - Udacity
#
# Félix Ramón López Martínez
# 20/12/2022


import argparse
import time
from enum import Enum

import numpy as np
import matplotlib.pyplot as plt

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        
        # Initializing variables
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.x_position = []
        self.y_position = []
        self.altitude_log = []
        self.velocity_log = []
        self.time_log = []
        self.start_time = time.time()

        # Initial state
        self.flight_state = States.MANUAL

        # Flight parameters
        self.fly_altitude = 8.0    # altitude
        self.length = 10.0            # length of square side

        # Registration of callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        #print('Current local position: ', np.around(self.local_position,2))

        # Data logging
        self.x_position.append(self.local_position[0])
        self.y_position.append(self.local_position[1])
        self.altitude_log.append(-1 * self.local_position[2])
        self.velocity_log.append(np.linalg.norm(self.local_velocity[0:2]))
        self.time_log.append(time.time() - self.start_time)

        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * self.local_position[2]    # Altitude conversion
            
            if altitude > 0.975 * self.fly_altitude:
                self.waypoints = self.calculate_box()
                self.n_waypoints = len(self.waypoints)   # Number of waypoints
                print("Starting Waypoint transition")
                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:
            dist_2_target = np.linalg.norm(self.target_position[0:2] - self.local_position[0:2])
            
            if  dist_2_target < 0.10:             # If drone close to target position
                if len(self.waypoints) > 0:       # and if there are still waypoints
                    self.waypoint_transition()    # then stay in waypoint transition
                
                else:                             # If there are no more waypoints
                    if np.linalg.norm(self.local_velocity[0:2]) < 0.10:   # and velocity low enough
                        print("Starting Landing transition")             # then start landing
                        self.landing_transition()

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:                      # If drone in Landing state
            if self.global_position[2] - self.global_home[2] < 0.05: # and position close to home
                if self.local_position[2] < 0.01:                    # and altitude small enough
                    print("Starting Disarming transition")           # then disarm the drone
                    self.disarming_transition()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                print("Starting Arming transition")
                self.arming_transition()

            elif self.flight_state == States.ARMING:
                if self.armed:
                    print("Starting Take-off transition")
                    self.takeoff_transition()

            elif self.flight_state == States.DISARMING:
                if not self.armed:
                    print("Starting Manual transition")
                    self.manual_transition()


    def calculate_box(self):
        """       
        Return waypoints to fly a box
        """
        print('Setting fly track to a: {}-m square'.format(self.length))
        waypoints = [[self.length, 0, self.fly_altitude],
                    [self.length, self.length, self.fly_altitude],
                    [0, self.length, self.fly_altitude],
                    [0, 0, self.fly_altitude]]
        return waypoints

    def arming_transition(self):
        self.take_control()    # Take control of the drone to accept python commands
        self.arm()             # Arm the drone to be ready for taking-off
        
        # Setting current location as home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        print('Home Position (Global Coordinates) set at: ', np.around(self.global_position, 2))

        self.flight_state = States.ARMING  # Change to Arming state

    def takeoff_transition(self):        
        self.takeoff(self.fly_altitude)     # Take-off up to reach fly altitude
        self.flight_state = States.TAKEOFF  # Change to Take-Off state

    def waypoint_transition(self):
        self.target_position = self.waypoints.pop(0)   # Define next target position taiking first waypoint in the list
        
        target_position_number = self.n_waypoints-len(self.waypoints)
        print('Flying to target position {}:'.format(target_position_number), self.target_position)
        
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)  # Command drone to go to next target position

        self.flight_state = States.WAYPOINT  # Change to WayPoint state
        

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        self.land()                         # Command drone to land
        self.flight_state = States.LANDING  # Change to Landing state

    def disarming_transition(self):
        self.disarm()                         # Disarm the drone
        self.flight_state = States.DISARMING  # Change to Disarming stte

    def manual_transition(self):
        self.release_control()             # Give up the control of the drone (stop accepting python commands)
        print('Drone in Manual mode')
        print('Final position (Global Coordinates):', np.around(self.global_position, 2))
        self.stop()                        # Stop drone connection and telemetry
        self.in_mission = False            # End mission
        self.flight_state = States.MANUAL  # Change to Manual state


    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")   # Open log file

        print("Starting connection")
        self.connection.start()                # Start drone connection

        print("Closing log file")
        self.stop_log()                        # Close log file

        # Plotting Fly Data
        fig = plt.figure(figsize=(15,5))
        # Plot 1: Altitude and Velocity
        plt.subplot(1,2,1)
        plt.plot(self.time_log, self.altitude_log, color='blue', label='altitude [m]')
        plt.plot(self.time_log, self.velocity_log, color='red', label='velocity [m/s]')
        plt.xlabel('time [s]')
        plt.legend()
        # Plot 2: Fly track
        plt.subplot(1,2,2)
        plt.plot(self.y_position, self.x_position, color='grey')
        plt.xlabel('East [m]')
        plt.ylabel('North [m]')
        plt.title('Fly Track')
        # Saving plot
        fig.savefig('Fly_profile.png')
        print('Saved Fly profile as "Fly_profile.png')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)           # Initialize drone class
    time.sleep(4)                         # Wait for 4 seconds before start
    drone.start()                         # Start drone "start" method
