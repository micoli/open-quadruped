from pydantic import BaseModel
from py_robot_bus.bus import Bus
from py_robot_bus.bus.message import Channels


@Bus.register(Channels.sys)
class QuitCommand(BaseModel):
    pass
