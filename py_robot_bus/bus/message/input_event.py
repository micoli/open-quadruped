from pprint import pformat
from typing import List

from pydantic import BaseModel

from py_robot_bus import DisplayMode
from py_robot_bus.bus import Bus
from py_robot_bus.bus.message import Channels


@Bus.register('input')
class ChangeDisplayModeCommand(BaseModel):
    mode: DisplayMode


@Bus.register(Channels.input)
class SetNextGaitCommand(BaseModel):
    pass


@Bus.register(Channels.input)
class SetGaitByNameCommand(BaseModel):
    name: str


@Bus.register(Channels.input)
class QuadrupedPoseParameter(BaseModel):
    x: int = 0
    y: int = 0
    z: int = 0
    yaw: int = 0
    pitch: int = 0
    roll: int = 0
    right_joy: List[int] = [0, 0]

    def __repr__(self):
        return pformat(vars(self), compact=True, underscore_numbers=True)

    def reset(self):
        self.right_joy = [0, 0]
        self.x = self.y = self.z = self.yaw = self.pitch = self.roll = 0

    def inc(self, parameter, increment):
        if parameter == 'x':
            self.x += increment
        if parameter == 'y':
            self.y += increment
        if parameter == 'z':
            self.z += increment
        if parameter == 'yaw':
            self.yaw += increment
        if parameter == 'pitch':
            self.pitch += increment
        if parameter == 'roll':
            self.roll += increment
