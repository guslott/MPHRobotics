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
from matplotlib.animation import FuncAnimation

class RobotWorld:
    def __init__(self, obstacles = None):
        # Initialize the RobotWorld
        # Define the wall corners
        self.wall_corners = np.array([
            [-1, 1, 1, -1, -1],
            [1, 1, -1, -1, 1]])
        self.obstacles = obstacles

        # Initialize floor
        self.floor = {}
        self.dirt = np.zeros((2, 0))
        self.dump_center = None
        self.h_dump_station = None

        # Magnetic field vector
        self.magnetic_field_nT = np.array([0.0, 52000.0])

        # Robots
        self.robot = []

        # Create figure and axis
        self.fig, self.ax = plt.subplots()
        self.h_title = self.ax.set_title('0 sec')
        self.h_wall, = self.ax.plot(
            self.wall_corners[0, :], self.wall_corners[1, :], 'r-', linewidth=2)

        # Initialize floor image
        self.floor['C'] = np.ones((1000, 1000, 3), dtype=np.uint8) * 255  # All white floor
        self.floor_image = self.floor['C']  # Reference to the floor image array
        xrange = [np.min(self.wall_corners[0, :]), np.max(self.wall_corners[0, :])]
        yrange = [np.min(self.wall_corners[1, :]), np.max(self.wall_corners[1, :])]
        self.floor['x'] = np.linspace(xrange[0], xrange[1], self.floor['C'].shape[1])
        self.floor['y'] = np.linspace(yrange[0], yrange[1], self.floor['C'].shape[0])
        self.h_floor = self.ax.imshow(
            self.floor['C'], extent=(
                self.floor['x'][0], self.floor['x'][-1],
                self.floor['y'][0], self.floor['y'][-1]),
            origin='lower')

        # Dirt
        self.h_dirt, = self.ax.plot([], [], '.', color=(0.8, 0.6, 0.6))

        # Obstacles
        self.h_obstacles = []
        for obstacle in self.obstacles:
            d = obstacle['side_length_m']
            center = obstacle['center']
            pts = np.array([
                [-1, 1, 1, -1, -1],
                [1, 1, -1, -1, 1]
            ]) * (d / 2) + np.reshape(center, (2, 1))
            obstacle['corners'] = pts
            line, = self.ax.plot(pts[0, :], pts[1, :], color=obstacle.get('color', 'k'), linewidth=2)
            self.h_obstacles.append(line)

        self.h_scanned_points = None

        # Set up axis
        self.ax.set_aspect('equal')
        self.ax.grid(False)
        mx = np.mean(xrange)
        my = np.mean(yrange)
        xrange = (np.array(xrange) - mx) * 1.1 + mx
        yrange = (np.array(yrange) - my) * 1.1 + my
        self.ax.set_xlim(xrange)
        self.ax.set_ylim(yrange)

        # Event handling
        self.fig.canvas.mpl_connect('key_press_event', self.figure_keypress)

        # Animation
        self.anim = None

    def start(self, speedup=1):
        # Start the world's animation
        adjusted_interval = max(1, 50 / speedup)  # Ensure interval is at least 1ms
        self.anim = FuncAnimation(
            self.fig, self.updateWorld, frames=None,
            interval=adjusted_interval, blit=False, cache_frame_data=False)
        plt.show()

    def add_robot(self, robot):
        self.robot.append(robot)

    def figure_keypress(self, event):
        for r in self.robot:
            r.remote_control(event.key)

    def updateWorld(self, frame):
        # This is the main time function that updates the world at the given timer interval
        dt = 0.1  # Assuming 10Hz update rate
        for r in self.robot:
            r.update(dt)
        # Update title with time
        for robot in self.robot:
            scanned_points = robot.get_scanned_points()
            if scanned_points.size > 0:
                # Check if this robot has a plot handle yet
                if not hasattr(robot, 'h_scanned_points') or robot.h_scanned_points is None:
                    # First time plotting scanned points for this robot
                    robot.h_scanned_points, = self.ax.plot(
                        scanned_points[0, :], scanned_points[1, :], 'b.', markersize=5, label='Scanned Points'
                    )
                else:
                    # Update existing plot for this robot
                    robot.h_scanned_points.set_data(scanned_points[0, :], scanned_points[1, :])

            time_s = robot.time_s
            self.h_title.set_text('{:.1f} sec'.format(time_s))

        self.fig.canvas.draw_idle()
        return []

    def draw_circle(self, center, radius, thickness, color):
        # All measurements in meters
        C = self.floor['C']
        x = self.floor['x']
        y = self.floor['y']

        X, Y = np.meshgrid(x, y)
        D = np.sqrt((X - center[0])**2 + (Y - center[1])**2)
        mask = (D >= (radius - thickness / 2)) & (D <= (radius + thickness / 2))

        # Ensure color is in range 0-255
        color = np.array(color, dtype=np.uint8)

        for i in range(3):
            C[:, :, i][mask] = color[i]

        self.h_floor.set_data(C)
        self.floor['C'] = C

    def draw_dot(self, center, radius, color):
        # All measurements in meters
        C = self.floor['C']
        x = self.floor['x']
        y = self.floor['y']

        X, Y = np.meshgrid(x, y)
        D = np.sqrt((X - center[0])**2 + (Y - center[1])**2)
        mask = D <= radius

        # Ensure color is in range 0-255
        color = np.array(color, dtype=np.uint8)

        for i in range(3):
            C[:, :, i][mask] = color[i]

        self.h_floor.set_data(C)
        self.floor['C'] = C

    # Implement other drawing methods similarly if needed

    def add_dirt(self):
        """
        Add specks of dirt all over the floor that the robot will clean up.
        """
        # Compute the space the walls enclose
        minx = np.min(self.wall_corners[0, :])
        maxx = np.max(self.wall_corners[0, :])
        miny = np.min(self.wall_corners[1, :])
        maxy = np.max(self.wall_corners[1, :])
        meanval = np.array([(minx + maxx) / 2, (miny + maxy) / 2])
        amp = np.array([maxx - meanval[0], maxy - meanval[1]])

        # Create random values in this space
        npoints = 1000
        data = (np.random.rand(2, npoints) - 0.5) * 2  # random values between [-1, 1]
        data[0, :] = data[0, :] * amp[0] * 0.98 + meanval[0]
        data[1, :] = data[1, :] * amp[1] * 0.98 + meanval[1]

        # Eliminate dirt outside the walls
        from matplotlib.path import Path
        wall_path = Path(self.wall_corners.T)
        inside = wall_path.contains_points(data.T)
        data = data[:, inside]

        # Eliminate dirt inside obstacles
        for obstacle in self.obstacles:
            obstacle_path = Path(obstacle['corners'].T)
            inside = obstacle_path.contains_points(data.T)
            data = data[:, ~inside]

        self.dirt = np.hstack((self.dirt, data))
        self.h_dirt.set_data(self.dirt[0, :], self.dirt[1, :])

    def add_dump_station(self, dump_center):
        """
        Adds a dump station at the specified location.
        """
        self.dump_center = np.array(dump_center)
        # For visualization, draw a square marker at dump_center
        self.h_dump_station, = self.ax.plot(
            [self.dump_center[0]], [self.dump_center[1]],  # Wrapped in lists
            'ks', markersize=15, label='Dump Station'
        )

    def draw_on_floor(self, position, color, width=3):
        """
        Draws a dot on the floor image at the given position with the specified color and width.
        """
        # Convert world coordinates to image indices
        x = self.floor['x']
        y = self.floor['y']
        xi = (np.abs(x - position[0])).argmin()
        yi = (np.abs(y - position[1])).argmin()

        # Ensure indices are within bounds
        height, width_img, _ = self.floor_image.shape
        yi = np.clip(yi, 0, height - 1)
        xi = np.clip(xi, 0, width_img - 1)

        # Draw the marker on the floor image
        pen_width = width  # Adjust the pen width as needed
        half_width = pen_width // 2

        # Update pixels around the marker position
        for dx in range(-half_width, half_width + 1):
            for dy in range(-half_width, half_width + 1):
                x_idx = xi + dx
                y_idx = yi + dy
                if 0 <= x_idx < width_img and 0 <= y_idx < height:
                    self.floor_image[y_idx, x_idx, :] = color

        # Update the floor image displayed
        self.update_floor_image()

    def update_floor_image(self):
        """
        Updates the displayed floor image.
        """
        self.h_floor.set_data(self.floor_image)
