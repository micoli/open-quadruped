from pydantic import BaseModel
from typing import List

from py_robot_bus.bus.message import Channels
from py_robot_bus.math.vector import Vector3D
from py_robot_bus.bus import Bus


class Vector3DM(BaseModel):
    x: List[float]
    y: List[float]
    z: List[float]

    @staticmethod
    def from_vector3d(vector3d: Vector3D):
        return Vector3DM(
            x=[float(pos) for pos in vector3d.x],
            y=[float(pos) for pos in vector3d.y],
            z=[float(pos) for pos in vector3d.z],
        )


@Bus.register(Channels.position)
class QuadrupedPositionEvent(BaseModel):
    body: Vector3DM
    rr: Vector3DM
    fr: Vector3DM
    fl: Vector3DM
    rl: Vector3DM
