import pprint

from py_robot_bus.engine.gait import Phase
from py_robot_bus.engine.gait.parameters import CycleGaitParameters
from py_robot_bus.engine.gait.planner import GaitPlanner
from py_robot_bus.math.bezier import Bezier

crawl = CycleGaitParameters(
    'crawl',
    phase_lag=[0, 0.5, 0.75, 0.25],
    T_swing=0.4,
    L_span=70,
    v_d=50,
    penetration_alpha=5,
    base_height=160,
    y=55,
    x_shift=-25,
    clearance=25
)

trot = CycleGaitParameters(
    'trot',
    phase_lag=[0, 0.5, 0.5, 0],
    T_swing=0.3,
    L_span=50,
    v_d=100,
    penetration_alpha=5,
    base_height=150,
    y=55,
    x_shift=-40,
    clearance=5
)

fast_trot = CycleGaitParameters(
    'fast_trot',
    phase_lag=[0, 0.5, 0.5, 0],
    T_swing=0.2,
    L_span=40,
    v_d=130,
    penetration_alpha=5,
    base_height=150,
    y=60,
    x_shift=-60,
    clearance=10
)


class CycleGaitPlanner(GaitPlanner):
    def __init__(self, gait: CycleGaitParameters):
        self.stance = Bezier(Bezier.get_cp_from_param(
            L_span=gait.L_span,
            base_height=gait.base_height,
            clearance=gait.clearance
        ))
        self.swing = Bezier([
            [gait.L_span, gait.base_height],
            [0, gait.base_height + gait.penetration_alpha],
            [-gait.L_span, gait.base_height]
        ])
        self.t_stance = gait.T_stance
        self.t_swing = gait.T_swing
        self.phase_lag = gait.phase_lag
        self.T_stride = self.t_swing + self.t_stance

    def generate_x_y_z(self, time: float, leg_index: int, joy_x: float, joy_y: float):
        phase, t = self._signal_sample(time, leg_index)
        if phase == Phase.Swing:
            x, z = self.swing.sample_bezier(t)
        if phase == Phase.Stance:
            x, z = self.stance.sample_bezier(t)

        if joy_y < -0.2:
            theta = 180
        else:
            theta = joy_x * 90

        return Bezier.rotate_about_z(x, z, theta)

    def _signal_sample(self, time, leg):
        phase_i = self.phase_lag[leg]
        phase_time = phase_i * self.T_stride

        base_time = (time - phase_time) % self.T_stride
        while base_time < 0:
            base_time = self.T_stride - base_time

        if base_time <= self.t_stance:
            # in stride period
            return [Phase.Stance, base_time / self.t_stance]

        # in swing period
        return [Phase.Swing, (base_time - self.t_stance) / self.t_swing]
