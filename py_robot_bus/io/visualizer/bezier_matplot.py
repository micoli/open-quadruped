import matplotlib
import numpy as np

from py_robot_bus.engine.gait.parameters import CycleGaitParameters
from py_robot_bus.engine.gait.planner.cycle_gait_planner import fast_trot
from py_robot_bus.math.bezier import Bezier

matplotlib.use("TkAgg")

import matplotlib.pyplot as plt


class BezierVisualizer:
    def __init__(self, window_size: int = 500, datasets=[]):
        self.window_size = window_size
        self.datasets = datasets

    def run(self):
        for index, dataset in enumerate(self.datasets):
            fig = plt.figure()
            fig.canvas.manager.set_window_title(f"Bezier {index}")
            ax = fig.add_subplot(111, projection="3d")
            self.draw(fig, ax, dataset)
        plt.show()

    def draw(self, fig, ax, dataset):
        self._setup_graph(ax)
        pxs, pys, pzs = [], [], []
        for index, [x, y] in enumerate(dataset):
            pxs.append(x)
            pys.append(y)
            pzs.append(0)

        xs, ys, zs = self.bezier_points(Bezier(dataset), 100)

        ax.plot(xs, zs, ys, color="blue")
        ax.plot(pxs, pzs, pys, color="red")
        fig.canvas.draw_idle()

    @staticmethod
    def bezier_points(bezier, num_points):
        xs, ys, zs = [], [], []
        for index in np.linspace(0, 1, num_points):
            x, y = bezier.sample_bezier(index)
            xs.append(x)
            ys.append(y)
            zs.append(0)
        return xs, ys, zs

    def _setup_graph(self, ax):
        ax.clear()
        ax.set_aspect("equal")
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("z (mm)")
        ax.set_zlabel("y (mm)")


def gait_to_bezier(gait: CycleGaitParameters):
    return [
        Bezier.get_cp_from_param(
            L_span=gait.L_span,
            base_height=gait.base_height,
            clearance=gait.clearance,
        ),
        [
            [gait.L_span, gait.base_height],
            [0, gait.base_height + gait.penetration_alpha],
            [-gait.L_span, gait.base_height],
        ],
    ]


if __name__ == "__main__":
    beziers1 = [
        [
            [10, -70],
            [20, -10],
            [30, 70],
            [-40, 20],
            [-50, -20],
            [-80, 80],
        ],
        [
            [-10, +10],
            [+10, +10],
            [+10, -10],
            [-10, -10],
            [-10, +10],
        ],
    ]
    visualizer = BezierVisualizer(
        # datasets=beziers1
        datasets=gait_to_bezier(fast_trot)
    )
    visualizer.run()
