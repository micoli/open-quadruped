from pydantic import BaseModel

from py_robot_bus.bus import Bus
from py_robot_bus.bus.message import Channels


class MotorsPosition(BaseModel):
    hip: float
    shoulder: float
    wrist: float


@Bus.register(Channels.position)
class MotorsPositionCommand(BaseModel):
    rr: MotorsPosition
    fr: MotorsPosition
    fl: MotorsPosition
    rl: MotorsPosition
