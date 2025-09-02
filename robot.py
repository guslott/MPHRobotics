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
import numpy as np
import matplotlib.pyplot as plt

class Robot:
    def __init__(self, world, setupfcn, navfcn):
        self.world = world
        self.user_setup_function = setupfcn
        self.user_navigation_function = navfcn

        self.userdata = {}

        self.width = 0.10  # meters
        self.len = 0.19  # meters
        self.wheel_radius = 0.02  # meters

        self.left_dps = 0.0  # degrees per second
        self.right_dps = 0.0  # degrees per second
        self.left_angle_deg = 0.0  # Absolute angular position for encoder
        self.right_angle_deg = 0.0
        self.omega_gyro = 0.0  # Initialize gyro value
        self.scanned_points = np.zeros((2, 0))
        self.marker_color = None  # Marker is off by default
        self.time_s = 0.0

        self.body = np.array([
            [0, 0, self.width, self.width, 0],
            [0, self.len, self.len, 0, 0]
        ])
        self.wheel_right = np.array([
            [0.10, 0.10, 0.12, 0.12, 0.10],
            [0.13, 0.17, 0.17, 0.13, 0.13]
        ])
        self.wheel_left = np.array([
            [-0.02, -0.02, 0.0, 0.0, -0.02],
            [0.13, 0.17, 0.17, 0.13, 0.13]
        ])

        self.center = np.mean(self.body[:, :4], axis=1)
        self.left_center = np.mean(self.wheel_left[:, :4], axis=1)
        self.right_center = np.mean(self.wheel_right[:, :4], axis=1)
        self.color_sensor = np.array([self.width / 2, self.len - 0.02])
        self.range_sensor = np.array([self.width / 2, self.len])

        self.led_shape = np.array([
            [-1, -1, 1, 1, -1],
            [-1, 1, 1, -1, -1]
        ]) * 0.015
        self.led_r = self.led_shape + np.array([[self.width / 2], [0.03]])
        self.led_g = self.led_shape + np.array([[self.width / 2], [0.07]])
        self.led_b = self.led_shape + np.array([[self.width / 2], [0.11]])
        self.led_values = [False, False, False]

        self.wheel_base = np.linalg.norm(self.left_center - self.right_center)
        self.center_history = np.zeros((2, 0))
        self.icc = np.array([np.nan, np.nan])

        self.n_dirt = 0
        self.dirt_on_off = False
        self.max_dirt = np.inf

        self.floor_color = np.array([255, 255, 255])

        self.read_color()

        # Draw the robot initially in the axis
        self.track_history = False
        self.draw_icc = False
        self.graphics = {}
        self.graphics['h_history'], = self.world.ax.plot([], [], 'g-', linewidth=1)
        self.graphics['h_icc'], = self.world.ax.plot([], [], 'k+', markersize=20)
        self.graphics['h_body'], = self.world.ax.plot(self.body[0, :], self.body[1, :], 'k-', linewidth=2)
        self.graphics['h_wheel_right'], = self.world.ax.fill(self.wheel_right[0, :], self.wheel_right[1, :], 'r')
        self.graphics['h_wheel_left'], = self.world.ax.fill(self.wheel_left[0, :], self.wheel_left[1, :], 'r')
        self.graphics['h_color_sensor'], = self.world.ax.plot([self.color_sensor[0]], [self.color_sensor[1]], 'k+')
        self.graphics['h_range_sensor'], = self.world.ax.plot([self.range_sensor[0]], [self.range_sensor[1]], 'k*')
        self.graphics['h_led_r'], = self.world.ax.fill(self.led_r[0, :], self.led_r[1, :], 'k')
        self.graphics['h_led_g'], = self.world.ax.fill(self.led_g[0, :], self.led_g[1, :], 'k')
        self.graphics['h_led_b'], = self.world.ax.fill(self.led_b[0, :], self.led_b[1, :], 'k')

        # Call the user's setup function
        self.user_setup_function(self)

    def set_track_history(self, on_off):
        self.track_history = on_off
        if self.track_history:
            self.center_history = np.hstack((self.center_history, np.array([[np.nan], [np.nan]])))

    def set_draw_icc(self, on_off):
        self.draw_icc = on_off

    def set_wheel_speed_dps(self, left_dps, right_dps):
        self.left_dps = left_dps
        self.right_dps = right_dps
        
    def read_wheel_speed_dps(self):
        return self.left_dps, self.right_dps

    def read_floor_color(self):
        """
        Reads the color of the floor under the robot's color sensor.
        Returns a NumPy array of integers [R, G, B].
        """
        x = self.world.floor['x']
        y = self.world.floor['y']
        C = self.world.floor['C']

        xi = (np.abs(x - self.color_sensor[0])).argmin()
        yi = (np.abs(y - self.color_sensor[1])).argmin()

        self.floor_color = C[yi, xi, :].astype(int)
        return self.floor_color

    def read_gyro_dps(self):
        """
        Returns the robot's angular velocity in degrees per second.
        """
        return self.omega_gyro * (180 / np.pi)

    def read_gps_meters(self):
        """
        Returns the robot's position in meters.
        """
        # Return the midpoint of the axle
        return (self.left_center + self.right_center) / 2

    def read_compass_degrees(self):
        """
        Returns the robot's heading in degrees, where 0 degrees is along the positive x-axis.
        """
        # Forward vector: from the center to the range sensor (front of the robot)
        forward_vec = self.range_sensor - self.center
        dir_w = forward_vec / np.linalg.norm(forward_vec)

        # Heading angle in degrees, relative to the positive x-axis
        heading_rad = np.arctan2(dir_w[1], dir_w[0])
        heading_deg = np.degrees(heading_rad)
        if heading_deg < 0:
            heading_deg += 360

        return heading_deg

    def led_state(self, red=None, green=None, blue=None):
        """
        Sets or gets the state of the RGB LEDs.
        If red, green, blue are provided (True/False), sets the LED states.
        If no arguments are provided, returns the current LED states as a tuple.
        """
        if red is not None or green is not None or blue is not None:
            if red is not None:
                self.led_values[0] = red
            if green is not None:
                self.led_values[1] = green
            if blue is not None:
                self.led_values[2] = blue
        return tuple(self.led_values)

    def read_range(self):
        """
        Simulates reading from the robot's range finder.
        Returns the distance to the nearest obstacle or wall in front of the robot.
        """
        # Use a ray casting method to find the distance to the nearest obstacle or wall
        # in the direction the robot is facing.

        # Get robot's position and heading
        pos = self.range_sensor  # Assuming the range sensor is at the front
        theta_deg = self.read_compass_degrees()
        theta_rad = np.deg2rad(theta_deg)

        # Define the ray
        max_range = 5.0  # Maximum range of the sensor in meters
        ray_end = pos + max_range * np.array([np.cos(theta_rad), np.sin(theta_rad)])

        # Check intersections with walls and obstacles
        min_distance = max_range
        obstacles = self.world.obstacles.copy()

        # Include walls as obstacles
        wall_obstacle = {'corners': self.world.wall_corners}
        obstacles.append(wall_obstacle)

        # For each obstacle
        for obstacle in obstacles:
            corners = obstacle['corners']
            num_corners = corners.shape[1] - 1
            for i in range(num_corners):
                p1 = corners[:, i]
                p2 = corners[:, i + 1]
                intersection = self._line_segment_intersection(pos, ray_end, p1, p2)
                if intersection is not None:
                    distance = np.linalg.norm(intersection - pos)
                    if distance < min_distance:
                        min_distance = distance

        return min_distance

    def add_scanned_point(self, point):
        """
        Adds a scanned point to the robot's collection.
        """
        self.scanned_points = np.hstack((self.scanned_points, point.reshape(2, 1)))

    def get_scanned_points(self):
        """
        Returns the scanned points collected by the robot.
        """
        return self.scanned_points

    def marker(self, color=None):
        """
        Sets or gets the marker color.
        If `color` is None, the marker is turned off.
        If `color` is a list or tuple of RGB values, the marker is set to that color.
        Returns the current marker color.
        """
        if color is not None:
            if isinstance(color, (list, tuple)) and len(color) == 3:
                self.marker_color = np.array(color, dtype=np.uint8)
            else:
                raise ValueError("Color must be a list or tuple of three RGB values.")
        else:
            self.marker_color = None
            
        return self.marker_color

    def update(self, dt):
        # Primary time evolution function for the robot
        self.time_s += dt

        # Limit speed
        max_speed = 360 * 3 # degrees per second
        self.left_dps = np.clip(self.left_dps, -max_speed, max_speed)
        self.right_dps = np.clip(self.right_dps, -max_speed, max_speed)

        # Differential drive kinematics
        omega_left = self.left_dps * (np.pi / 180)
        Vl = omega_left * self.wheel_radius  # Linear wheel velocity (left)

        omega_right = self.right_dps * (np.pi / 180)
        Vr = omega_right * self.wheel_radius  # Linear wheel velocity (right)

        omega_rad = (Vr - Vl) / self.wheel_base
        velocity = (Vl + Vr) / 2  # linear velocity component
        axle_center = (self.left_center + self.right_center) / 2

        dx = np.array([0.0, 0.0])
        dtheta = 0.0

        if omega_rad != 0:
            # Some rotation
            t = (self.wheel_base / 2) * (Vl + Vr) / (Vr - Vl)
            # Calculate center of rotation
            axle_dir = self.left_center - self.right_center
            axle_dir = axle_dir / np.linalg.norm(axle_dir)
            self.icc = axle_center + axle_dir * t
            dtheta = omega_rad * dt
        else:
            forward = axle_center - self.center
            forward = forward / np.linalg.norm(forward)  # Unit length vector
            self.icc = np.array([np.nan, np.nan])  # icc is at infinity for pure translation
            dist = dt * velocity
            dx = forward * dist

        # Collision detection with obstacles and walls
        effective_dt = dt
        obstacles = self.world.obstacles.copy()

        # Treat walls as another obstacle
        wall_obstacle = {'corners': self.world.wall_corners}
        obstacles.append(wall_obstacle)

        # Get the robot's body coordinates
        bc = self.body.copy()

        # Compute the next step positions (bcdx)
        if omega_rad != 0:
            # Rotational motion
            R = self.rotation_matrix(dtheta)
            bcdx = R @ (bc - self.icc.reshape(2, 1)) + self.icc.reshape(2, 1)
        else:
            # Translational motion
            bcdx = bc + dx.reshape(2, 1)

        # For each obstacle (includes walls)
        collision = False
        for obstacle in obstacles:
            oc = obstacle['corners']
            # Ensure oc is a numpy array
            oc = np.array(oc)
            # For each edge of the obstacle
            num_oc_corners = oc.shape[1]
            for i in range(num_oc_corners - 1):
                pt0 = oc[:, i]
                pt1 = oc[:, i + 1]
                pt10 = pt1 - pt0
                # For each edge of the robot's body
                num_body_points = bcdx.shape[1]
                for j in range(num_body_points - 1):
                    pb0 = bcdx[:, j]
                    pb1 = bcdx[:, j + 1]
                    pb10 = pb1 - pb0
                    # Solve for intersection
                    A = np.column_stack((pt10, -pb10))
                    b_vec = pb0 - pt0
                    detA = np.linalg.det(A)
                    if abs(detA) < 1e-15:
                        continue  # Lines are parallel
                    sd = np.linalg.solve(A, b_vec)
                    s = sd[0]
                    d = sd[1]
                    if 0 <= s <= 1 and 0 <= d <= 1:
                        # Collision detected
                        dx = np.zeros_like(dx)
                        dtheta = 0.0
                        omega_rad = 0.0
                        effective_dt = 0.0
                        collision = True
                        break  # Break out of body edges loop
                if collision:
                    break  # Break out of obstacle edges loop
            if collision:
                break  # Break out of obstacles loop

        # Apply the collision-corrected movement
        if omega_rad != 0:
            self.translate(-self.icc)
            self.rotate(dtheta)
            self.translate(self.icc)
        else:
            self.translate(dx)

        # Update wheel angles
        self.left_angle_deg += self.left_dps * dt
        self.right_angle_deg += self.right_dps * dt

        if self.track_history:
            self.center_history = np.hstack((self.center_history, self.center.reshape(2, 1)))

        self.read_color()
        # self.calc_range()  # Implement if necessary
        self.omega_gyro = omega_rad

        # Vacuum dirt
        if self.dirt_on_off:
            self.vacuum_dirt()

        # Draw on the floor if the marker is active
        if self.marker_color is not None:
            self.draw_marker()

        # Update graphics
        self.draw_robot()

        # Call the user's navigation program
        self.user_navigation_function(self, dt)

    def vacuum(self, on_off=None):
        """
        Turn on/off the vacuum - on_off=True to turn on, False to turn off.
        If on_off is None, returns the current state.
        """
        if on_off is not None:
            self.dirt_on_off = on_off
        return self.dirt_on_off

    def count_dirt(self):
        """
        Returns the amount of dirt collected by the robot.
        """
        return self.n_dirt

    def find_dump_station(self):
        """
        Returns the location of the dump station.
        """
        return self.world.dump_center

    def dump_dirt(self):
        """
        Empties the robot's dirt collection.
        """
        self.n_dirt = 0

    def set_max_dirt(self, max_dirt=None):
        """
        Sets or gets the maximum amount of dirt the robot can handle.
        """
        if max_dirt is not None:
            self.max_dirt = max_dirt
        return self.max_dirt

    def get_max_dirt(self):
        """
        Returns the maximum dirt capacity.
        """
        return self.max_dirt

    def vacuum_dirt(self):
        """
        Remove any dirt within the body frame from the world.
        """
        from matplotlib.path import Path
        body_path = Path(self.body.T)
        inside = body_path.contains_points(self.world.dirt.T)
        collected_dirt = self.world.dirt[:, inside]

        # Update the world's dirt
        self.world.dirt = self.world.dirt[:, ~inside]
        self.world.h_dirt.set_data(self.world.dirt[0, :], self.world.dirt[1, :])

        # Increase the robot's dirt count, considering max capacity
        new_dirt_count = self.n_dirt + collected_dirt.shape[1]
        if new_dirt_count > self.max_dirt:
            # Put some of the dirt back
            excess = new_dirt_count - self.max_dirt
            self.world.dirt = np.hstack((self.world.dirt, collected_dirt[:, :excess]))
            self.n_dirt = self.max_dirt
        else:
            self.n_dirt = new_dirt_count

    def translate(self, dx):
        # Shift all body points by dx
        self.body += dx.reshape(2, 1)
        self.wheel_left += dx.reshape(2, 1)
        self.wheel_right += dx.reshape(2, 1)
        self.center += dx
        self.left_center += dx
        self.right_center += dx
        self.color_sensor += dx
        self.range_sensor += dx
        self.led_r += dx.reshape(2, 1)
        self.led_g += dx.reshape(2, 1)
        self.led_b += dx.reshape(2, 1)

    def rotate(self, theta_rad):
        # Rotate all points around the origin by theta
        R = self.rotation_matrix(theta_rad)
        self.body = R @ self.body
        self.wheel_left = R @ self.wheel_left
        self.wheel_right = R @ self.wheel_right
        self.center = R @ self.center
        self.left_center = R @ self.left_center
        self.right_center = R @ self.right_center
        self.color_sensor = R @ self.color_sensor
        self.range_sensor = R @ self.range_sensor
        self.led_r = R @ self.led_r
        self.led_g = R @ self.led_g
        self.led_b = R @ self.led_b

    def rotation_matrix(self, theta_rad):
        cos_theta = np.cos(theta_rad)
        sin_theta = np.sin(theta_rad)
        return np.array([
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta]
        ])

    def read_color(self):
        # Find the nearest discrete index into the image at the color sensor
        x = self.world.floor['x']
        y = self.world.floor['y']
        C = self.world.floor['C']

        xi = (np.abs(x - self.color_sensor[0])).argmin()
        yi = (np.abs(y - self.color_sensor[1])).argmin()

        self.floor_color = C[yi, xi, :]

    def draw_robot(self):
        self.graphics['h_body'].set_data(self.body[0, :], self.body[1, :])
        self.graphics['h_wheel_right'].set_xy(self.wheel_right.T)
        self.graphics['h_wheel_left'].set_xy(self.wheel_left.T)
        if self.track_history:
            self.graphics['h_history'].set_data(self.center_history[0, :], self.center_history[1, :])
        if self.draw_icc:
            self.graphics['h_icc'].set_data([self.icc[0]], [self.icc[1]])

        self.graphics['h_color_sensor'].set_data([self.color_sensor[0]], [self.color_sensor[1]])
        self.graphics['h_range_sensor'].set_data([self.range_sensor[0]], [self.range_sensor[1]])

        # Update LED positions
        self.graphics['h_led_r'].set_xy(self.led_r.T)
        self.graphics['h_led_g'].set_xy(self.led_g.T)
        self.graphics['h_led_b'].set_xy(self.led_b.T)

        # Update LEDs colors
        self.update_led(self.graphics['h_led_r'], self.led_values[0], 'r')
        self.update_led(self.graphics['h_led_g'], self.led_values[1], 'g')
        self.update_led(self.graphics['h_led_b'], self.led_values[2], 'b')

    def update_led(self, led_patch, is_on, color):
        facecolor = color if is_on else 'k'
        led_patch.set_facecolor(facecolor)

    def remote_control(self, key):
        # The UI can send letter commands to the robot (e.g. from the keyboard) to cause it to change behaviors.
        velocity_step_dps = 100  # degrees per second on little wheels
        if key == 'up':
            self.set_wheel_speed_dps(
                self.left_dps + velocity_step_dps,
                self.right_dps + velocity_step_dps)
        elif key == 'down':
            self.set_wheel_speed_dps(
                self.left_dps - velocity_step_dps,
                self.right_dps - velocity_step_dps)
        elif key == 'right':
            self.set_wheel_speed_dps(
                self.left_dps + velocity_step_dps,
                self.right_dps - velocity_step_dps)
        elif key == 'left':
            self.set_wheel_speed_dps(
                self.left_dps - velocity_step_dps,
                self.right_dps + velocity_step_dps)
        elif key == ' ':
            self.set_wheel_speed_dps(0, 0)
        elif key == 'r':
            self.led_values[0] = not self.led_values[0]
        elif key == 'g':
            self.led_values[1] = not self.led_values[1]
        elif key == 'b':
            self.led_values[2] = not self.led_values[2]
        # Add other key controls if necessary

    def _line_segment_intersection(self, p1, p2, q1, q2):
        """
        Finds the intersection point of two line segments p1-p2 and q1-q2.
        Returns the intersection point or None if there is no intersection.
        """
        # Line p represented as p1 + r*(p2 - p1)
        # Line q represented as q1 + s*(q2 - q1)
        d = (p2 - p1)
        e = (q2 - q1)
        denominator = d[0] * e[1] - d[1] * e[0]
        if abs(denominator) < 1e-10:
            return None  # Lines are parallel

        numerator = (q1 - p1)
        t = (numerator[0] * e[1] - numerator[1] * e[0]) / denominator
        u = (numerator[0] * d[1] - numerator[1] * d[0]) / denominator
        if 0 <= t <= 1 and 0 <= u <= 1:
            intersection = p1 + t * d
            return intersection
        else:
            return None

    def draw_marker(self):
        """
        Draws on the floor image at the marker's position.
        """
        # Calculate the position of the marker, slightly in front of the wheel axis
        marker_offset = 0.05  # 5 cm in front of the robot's center

        # Get the robot's heading angle in radians
        theta_deg = self.read_compass_degrees()
        theta_rad = np.deg2rad(theta_deg)

        # Calculate the marker position in world coordinates
        dx = marker_offset * np.cos(theta_rad)
        dy = marker_offset * np.sin(theta_rad)
        marker_pos = self.center + np.array([dx, dy])

        # Draw on the floor image at marker_pos
        self.world.draw_on_floor(marker_pos, self.marker_color)
