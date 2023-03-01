# Project 2: 3D Motion Planning

## The project

Here you can find my implementation of the second project of the [Flying Car and Autonomous Flight Engineer Learning Nanodegree](https://www.udacity.com/course/flying-car-nanodegree--nd787), called 3D Motion Planning.

This project is about to develop an algorithm that is able to plan a path from a start to a goal location trough an urban environment for a quadcopter.

The algorithm is developed and tested within the synthetic environment provided by the Udacity drone simulator. (you can download the simulator [here](https://github.com/udacity/FCND-Simulator-Releases/releases)). The code is similar to how the drone would be controlled from a ground station computer or an onboard flight computer. Since communication with the drone is done using MAVLink, you will be able to use your code to control an PX4 quadcopter autopilot with very little modification!

The project is solved using A* addapted on thre three different approaches:
+ ON a 2D grid representation of the obstacle map
+ On a 2D graph representation of the obstacle map
+ On a 3D graph representation of the obstacle map built using Probabilistic RoadMap

For more details about how the code is implemented, open `Writeup.ipynb`file in the `Motion_planning_with_PRM`folder.



## Instructions

### Step 1: Download the Simulator
Download the version of the simulator that's appropriate for your operating system [from this repository](https://github.com/udacity/FCND-Simulator-Releases/releases).

### Step 2: Set up your Python Environment
Set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

### Step 3: Clone this Repository

### Step 4: Select with which approach you want to run the planner (and open the corresponding folder)

### Step 5: Open a terminal, inizialize the environment and run "MY-motion_planning_xxx.py" file
