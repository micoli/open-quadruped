from enum import Enum


class Phase(str, Enum):
    Stance = 1
    Swing = 2


class GaitEndException(Exception):
    pass
