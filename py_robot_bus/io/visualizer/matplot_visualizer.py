import sys
import threading
import time

import matplotlib

from py_robot_bus.bus import Bus
from py_robot_bus.bus.message import Channels
from py_robot_bus.bus.message.system import QuitCommand
from py_robot_bus.bus.redis import create_redis

matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
from py_robot_bus.bus.message.quadruped_position_event import QuadrupedPositionEvent


class MatplotVisualizer:
    def __init__(
        self, window_size: int = 500, animate_interval: int = 50, height: int = 170
    ):
        self.window_size = window_size
        self.animate_interval = animate_interval
        self.start_height = height

        self.redis_client, self.pubsub = create_redis([Channels.position, Channels.sys])
        self.fig = plt.figure()
        self.fig.canvas.manager.set_window_title("Animate")
        plt.get_current_fig_manager().window.wm_geometry("400x400+1800+0")
        self.ax = self.fig.add_subplot(111, projection="3d")

        self.legend = True
        self.actions = {
            "1": "reset",
            "x": "x",
            "y": "y",
            "z": "z",
            "a": "yaw",
            "p": "pitch",
            "r": "roll",
        }

    def _run(self):
        while True:
            for message in Bus.receive_messages(self.pubsub):
                if isinstance(message, QuitCommand):
                    plt.close("all")
                    sys.exit(0)
                if isinstance(message, QuadrupedPositionEvent):
                    self.draw_robot(message)
            time.sleep(0.05)

    def run(self):
        thread = threading.Thread(target=self._run)
        thread.start()
        plt.show()

    def draw_robot(self, position: QuadrupedPositionEvent):
        self._setup_graph()
        vectors = position.body
        self.ax.plot(vectors.x, vectors.y, vectors.z, color="blue")
        vectors = position.fr
        self.ax.plot(vectors.x, vectors.y, vectors.z, color="blue")
        vectors = position.fl
        self.ax.plot(vectors.x, vectors.y, vectors.z, color="red")
        vectors = position.rr
        self.ax.plot(vectors.x, vectors.y, vectors.z, color="orange")
        vectors = position.rl
        self.ax.plot(vectors.x, vectors.y, vectors.z, color="green")

        self.fig.canvas.draw_idle()

    def _setup_graph(self):
        self.ax.clear()
        self.ax.set_aspect("equal")

        self.ax.set_xlim3d(-self.window_size / 2, self.window_size / 2)
        self.ax.set_ylim3d(-self.window_size / 2, self.window_size / 2)
        self.ax.set_zlim3d(-self.start_height, self.window_size - self.start_height)

        if self.legend:
            self.ax.set_xlabel("x (mm)")
            self.ax.set_ylabel("y (mm)")
            self.ax.set_zlabel("z (mm)")
            # self.ax.legend(
            #     loc='best',
            #     handles=[Patch(facecolor='b', label=f'[{key}] {label}') for (key, label) in self.actions.items()]
            # )
            return
        self.ax.axes.set_axis_off()
