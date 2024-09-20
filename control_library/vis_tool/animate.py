import math

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
from pynput import keyboard
from matplotlib.patches import Patch

from .IK_Engine import Quadruped


class Vizualizer:
    def __init__(self):
        # Setting up 3D matplotlib figure
        self.fig = plt.figure()
        self.fig.canvas.manager.set_window_title('Animate')
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_aspect("equal")

        self.x = self.y = self.z = self.yaw = self.pitch = self.roll = 0

        self.WINDOW_SIZE = 500
        self.ANIMATE_INTERVAL = 50
        self.start_height = 170

        self.robot = Quadruped(ax=self.ax, origin=(0, 0, 0), height=self.start_height)
        self.actions={
          '1': 'reset',
          'x': 'x',
          'y': 'y',
          'z': 'z',
          'a': 'yaw',
          'p': 'pitch',
          'r': 'roll',
        }
        self.switch = 'x'
        self.buffer = 0
        self.inc = 5

    def on_press(self,key):
        if key == keyboard.Key.esc:
            return
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # other keys
        if k in ['1']:
            self.x = self.y = self.z = self.yaw = self.pitch = self.roll = 0
        if k in ['x']:
            self.switch = 'x'
        if k in ['y']:
            self.switch = 'y'
        if k in ['z']:
            self.switch = 'z'
        if k in ['a']:
            self.switch = 'a'
        if k in ['p']:
            self.switch = 'p'
        if k in ['r']:
            self.switch = 'r'
        if k in ['up']:
            self.buffer = self.inc
        if k in ['down']:
            self.buffer = -self.inc
        if k in ['up', 'down']:
            if self.switch == 'x':
                self.x += self.buffer
            if self.switch == 'y':
                self.y += self.buffer
            if self.switch == 'z':
                self.z += self.buffer
            if self.switch == 'a':
                self.yaw += self.buffer
            if self.switch == 'p':
                self.pitch += self.buffer
            if self.switch == 'r':
                self.roll += self.buffer

    def setup_graph(self):
        self.ax.clear()
        # ax.set_aspect("equal")

        self.ax.set_xlim3d(-self.WINDOW_SIZE / 2, self.WINDOW_SIZE / 2)
        self.ax.set_ylim3d(-self.WINDOW_SIZE / 2, self.WINDOW_SIZE / 2)
        self.ax.set_zlim3d(-self.start_height, self.WINDOW_SIZE - self.start_height)

        self.ax.set_xlabel('x (mm)')
        self.ax.set_ylabel('y (mm)')
        self.ax.set_zlabel('z (mm)')
        self.ax.legend(
            loc='best',
            handles=[Patch(facecolor='b', label=f'[{key}] {label}') for (key,label) in self.actions.items()]
        )


    def animate(self,i):
        self.setup_graph()
        # Going to starting pose
        self.robot.start_position()
        # Shifting robot pose in cartesian system x-y-z (body-relative)
        self.robot.shift_body_xyz(self.x, self.y, self.z)
        # Shifting robot pose in Euler Angles yaw-pitch-roll (body-relative)
        self.robot.shift_body_rotation(
            math.radians(self.yaw),
            math.radians(self.pitch),
            math.radians(self.roll)
        )

        self.robot.draw_body()
        self.robot.draw_legs()

    def run(self):
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()  # start to listen on a separate thread

        ani = animation.FuncAnimation(self.fig, self.animate, interval=self.ANIMATE_INTERVAL)
        self.show()

    def show(self):
        plt.show()

if __name__ == "__main__":
    vizualizer = Vizualizer()
    vizualizer.run()