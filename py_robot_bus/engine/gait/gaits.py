import pprint
import time
from typing import List

from py_robot_bus.engine.gait.parameters import (
    CycleGaitParameters,
    GaitParameters,
    DirectGaitParameters,
)
from py_robot_bus.engine.gait.planner import GaitPlanner
from py_robot_bus.engine.gait.planner.cycle_gait_planner import (
    CycleGaitPlanner,
    crawl,
    trot,
    fast_trot,
)
from py_robot_bus.engine.gait.planner.direct_gait_planner import (
    DirectGaitPlanner,
    get_up,
)

DEBUG = False


class Gaits:
    def __init__(self, _time):
        self.gait_parameters: List[GaitParameters] = [crawl, trot, fast_trot, get_up]
        self.gait_index: int = 0
        self.gait_start_time: float = 0
        self.gait_names = [param.get_name() for param in self.gait_parameters]
        self.planner: GaitPlanner | None = None

        self.set_gait_by_index(self.gait_index)
        self.set_start_time(_time)

    def set_gait_by_name(self, name: str) -> None:
        self.set_gait_by_index(self.gait_names.index(name))

    def set_gait_by_index(self, index: int):
        self.gait_index = index
        gait_parameter = self.gait_parameters[index]
        self.set_start_time(time.time())
        pprint.pprint(f"Gait: {gait_parameter.get_name()}")

        if isinstance(gait_parameter, CycleGaitParameters):
            self.planner = CycleGaitPlanner(gait_parameter)

        if isinstance(gait_parameter, DirectGaitParameters):
            self.planner = DirectGaitPlanner(gait_parameter)

    def next(self):
        self.gait_index = (self.gait_index + 1) % len(self.gait_parameters)
        self.set_gait_by_index(self.gait_index)

    def get_legs_xyz(self, _time: float, joy_x: float, joy_y: float):
        # 'RR', 'FR', 'FL', 'RL'
        legs_pos = []
        for leg_index in range(0, 4):
            x, y, z = self.planner.generate_x_y_z(
                _time - self.gait_start_time, leg_index, joy_x, joy_y
            )
            if DEBUG:
                pprint.pprint(f"{leg_index},x:{x}/y:{y}/z:{z}")
            legs_pos.append((x, y, z))

        return legs_pos

    def set_start_time(self, _time: float):
        self.gait_start_time = _time
