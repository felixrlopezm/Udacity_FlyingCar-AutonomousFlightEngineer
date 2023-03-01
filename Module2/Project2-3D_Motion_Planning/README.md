{
 "cells": [
  {
   "cell_type": "markdown",
   "id": "8c964d85",
   "metadata": {},
   "source": [
    "# Project 2: 3D Motion Planning\n",
    "\n",
    "## The project\n",
    "\n",
    "Here you can find my implementation of the second project of the [Flying Car and Autonomous Flight Engineer Learning Nanodegree](https://www.udacity.com/course/flying-car-nanodegree--nd787), called 3D Motion Planning.\n",
    "\n",
    "This project is about to develop an algorithm that is able to plan a path from a start to a goal location trough an urban environment for a quadcopter.\n",
    "\n",
    "The algorithm is developed and tested within the synthetic environment provided by the Udacity drone simulator. (you can download the simulator [here](https://github.com/udacity/FCND-Simulator-Releases/releases)). The code is similar to how the drone would be controlled from a ground station computer or an onboard flight computer. Since communication with the drone is done using MAVLink, you will be able to use your code to control an PX4 quadcopter autopilot with very little modification!\n",
    "\n",
    "The project is solved using A* addapted on thre three different approaches:\n",
    "+ ON a 2D grid representation of the obstacle map\n",
    "+ On a 2D graph representation of the obstacle map\n",
    "+ On a 3D graph representation of the obstacle map built using Probabilistic RoadMap\n",
    "\n",
    "For more details about how the code is implemented, open `d`file in the\n",
    "\n",
    "\n",
    "\n",
    "## Instructions\n",
    "\n",
    "### Step 1: Download the Simulator\n",
    "Download the version of the simulator that's appropriate for your operating system [from this repository](https://github.com/udacity/FCND-Simulator-Releases/releases).\n",
    "\n",
    "### Step 2: Set up your Python Environment\n",
    "Set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)\n",
    "\n",
    "### Step 3: Clone this Repository\n",
    "\n",
    "### Step 4: Select with which approach you want to run the planner (and open the corresponding folder)\n",
    "\n",
    "### Step 5: Open a terminal, inizialize the environment and run \"MY-motion_planning_xxx.py\" file"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "id": "27c5822a",
   "metadata": {},
   "outputs": [],
   "source": []
  }
 ],
 "metadata": {
  "kernelspec": {
   "display_name": "Python 3",
   "language": "python",
   "name": "python3"
  },
  "language_info": {
   "codemirror_mode": {
    "name": "ipython",
    "version": 3
   },
   "file_extension": ".py",
   "mimetype": "text/x-python",
   "name": "python",
   "nbconvert_exporter": "python",
   "pygments_lexer": "ipython3",
   "version": "3.6.3"
  }
 },
 "nbformat": 4,
 "nbformat_minor": 5
}
