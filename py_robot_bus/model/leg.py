import pprint

import math

import numpy as np

DEBUG = False


class Leg:
    def __init__(self, name: str, origin):
        self.name = name
        self.init_origin = origin
        self.origin = origin
        self.hip_rad = None
        self.shoulder_rad = None
        self.wrist_rad = None
        # local
        self.x = None
        self.y = None
        self.z = None
        # global
        self.g_x = None
        self.g_y = None
        self.g_z = None

    def __str__(self):
        return f'origin: {self.origin}, angles: [hip: {math.degrees(self.hip_rad)}, shoulder: {math.degrees(self.shoulder_rad)}, wrist: {math.degrees(self.wrist_rad)}], endpoint: {(self.x, self.y, self.z)}'
