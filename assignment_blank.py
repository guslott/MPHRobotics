###
# Copyright (c) 2025 Dr. Gus Lott (guslott@gmail.com)
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Blank Robotics Assignment

import numpy as np
from robot_world import RobotWorld
from robot import Robot

def user_setup(robot):
    """
    This function is called only once when the robot "boots up." Use this
    function to initialize your program
    """
    pass

def user_program(robot, dt_sec):
    """
    The user can read from sensors/encoders and give actuator
    commands to move or stop the wheels in this function. This is
    the primary location for a student to write their navigation
    code. This is the driver's seat and is called 10 times per
    second.
    """
    pass




# Define obstacles (square blocks)
obstacles = []

# Obstacle 1
obstacle1 = {
    'center': np.array([-0.6, 0.6]),
    'side_length_m': 0.1,  # 10cm
    'color': 'r'
}
obstacles.append(obstacle1)

# Obstacle 2
obstacle2 = {
    'center': np.array([0.5, 0.5]),
    'side_length_m': 0.2,  # 20cm
    'color': 'r'
}
obstacles.append(obstacle2)

# Create a world with a floor image and wall corners defined above
world = RobotWorld(obstacles)

# Draw the floor pattern
ccenter = np.array([-0.15, -0.1])  # in meters
world.draw_circle(ccenter, 0.6, 0.02, [0, 0, 0])  # Black circle
world.draw_dot(ccenter, 0.1, [0, 255, 0])  # Green dot

# Optionally add dirt (if you implement dirt handling)
# world.add_dirt()

# Put a robot in the world - Give it the setup and user_program functions defined above.
# The setup program is called immediately here when the robot is "constructed"
robot = Robot(world, user_setup, user_program)
world.add_robot(robot)

# Start the world's animation
world.start()
