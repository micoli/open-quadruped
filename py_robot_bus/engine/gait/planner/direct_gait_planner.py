from py_robot_bus.engine.gait.parameters import DirectGaitParameters
from py_robot_bus.engine.gait.planner import GaitPlanner


get_up = DirectGaitParameters(name='get_up')


class DirectGaitPlanner(GaitPlanner):
    def __init__(self, gait: DirectGaitParameters):
        self.gait = gait

    """
    map FL_FR_RL_RR to RR_FR_FL_RL
    """
    @staticmethod
    def _map(pos):
        return [pos[3], pos[1], pos[0], pos[2]]

    # FL FR
    # RL RR
    # 0:RR, 1:FR, 2:FL, 3:RL
    def generate_x_y_z(self, time: float, leg_index: int, joy_x: float, joy_y: float):
        if time < 4:
            return self._map([
                [0, 60, 30], [0, 30, 160],
                [0, 60, 30], [0, 30, 160],
            ])[leg_index]
        if time < 8:
            return self._map([
                [0, 80, 30], [0, 30, 170],
                [0, 80, 30], [0, 30, 170],
            ])[leg_index]
        from py_robot_bus.engine.gait import GaitEndException
        raise GaitEndException


