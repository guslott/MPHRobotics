# MPH Robotics: Python Simulator for Education

!MPH Robotics Simulator

### An educational platform for teaching introductory robotics, Python programming, and systems thinking at Manlius Pebble Hill School.

This project provides a simple, 2D differential-drive robot simulator written in Python using `matplotlib`. It is designed specifically for a high school classroom setting, allowing students to focus on programming logic and robotics concepts without the complexities of physical hardware.

The simulator is the foundation for the "Introduction to Robotics & Society" course at **Manlius Pebble Hill School**, a curriculum that blends hands-on programming with a deep ethical and philosophical exploration of autonomous systems.

## Vision & Educational Philosophy

The goal of this simulator is to provide a platform for teaching robotics from first principles. Students learn the foundational engineering loop of **Sense -> Compute -> Actuate**. They write code to read from virtual sensors (color, range, GPS), compute a response, and actuate the robot's wheels and systems.

This platform is designed to support a curriculum that directly embodies the **MPH Core Values** of **Curiosity, Agency, and Respect** by asking the big questions: What is a robot? What is the nature of autonomy? What are our responsibilities to the complex systems we create? By providing an accessible, hands-on tool, we empower students to explore these questions through direct experience.

## Key Features

* **Simple 2D World:** A clean, easy-to-understand environment powered by `matplotlib`.
* **Differential-Drive Robot:** A simulated robot with two independently controlled wheels, mimicking common educational and consumer robots (like the Roomba).
* **Rich Sensor Suite:** The robot is equipped with:
    * Floor Color Sensor
    * Forward-Facing Range Finder
    * GPS and Compass
    * Gyroscope for angular velocity
    * Wheel Encoders (implicit in speed control)
* **Actuators & Tools:**
    * Independent wheel speed control
    * RGB LED for status indication
    * Floor Marker for drawing paths
    * Vacuum for "cleaning" simulated dirt
* **Customizable Worlds:** Easily define worlds with walls, obstacles, colored lines, and "dirt" for a wide variety of challenges.

## Getting Started

### Prerequisites

You will need Python 3 and a couple of common scientific computing libraries.

```bash
pip install numpy matplotlib
```

### Running a Simulation

1.  Clone or download this repository.
2.  All the assignments are designed to be run directly. To run the first bootcamp assignment, for example, simply execute the file in your terminal:

```bash
python assignment_blank.py
```

A `matplotlib` window will appear showing the robot in its world, and the simulation will begin.

## Project Structure

The simulator is broken into three core files:

* `robot_world.py`: This is the "physics engine." It creates the world, draws the floor and obstacles, and manages the animation loop. Students and teachers generally do not need to edit this file.
* `robot.py`: This file defines the `Robot` class. It's the robot's "body"—containing all its sensors, actuators, and the differential drive kinematics. This is the API (Application Programming Interface) that students will use to control the robot.
* `assignment_blank.py` (and other assignment files): This is the "brain" of the robot. This is where students write their code to make the robot perform tasks.

## Writing a Robot Program

Students only need to focus on two functions within an assignment file:

```python
def user_setup(robot):
    """
    This function is called only once when the robot "boots up."
    Use this to initialize state variables, set targets, or configure
    the robot for its task. All your variables should be stored in the
    `robot.userdata` dictionary.
    """
    robot.userdata['state'] = 'FORWARD'
    robot.userdata['time_elapsed'] = 0

def user_program(robot, dt_sec):
    """
    This function is the robot's main control loop. It is called
    repeatedly (approx. 20 times per second). All sensor reading
    and motor commands happen here.
    """
    # Sense -> Read a sensor
    color = robot.read_floor_color()

    # Compute -> Make a decision
    if sum(color) == 0: # If the robot sees black
        # Actuate -> Give a command
        robot.set_wheel_speed_dps(0, 0) # Stop
```

A full curriculum of scaffolded assignments is available upon request, from a "Robot Bootcamp" to advanced challenges involving waypoint navigation, mapping, and obstacle avoidance.

## For Educators

This curriculum and simulator are designed to be a resource for the broader educational community. If you are a teacher interested in adopting or adapting this material for your own classroom, you are welcome to do so.

For access to solved versions of the assignments for grading purposes, or to collaborate on improving this curriculum, please feel free to reach out to me, Dr. Gus Lott, at **<glott@mphschool.org>**.

## License

This project is open-source and available for any educator or student to use, modify, and share. It is released under the **MIT License**. See the `LICENSE` file for full details. We encourage you to adapt it for your own classroom!