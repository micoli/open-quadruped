import numpy as np
import math

DEBUG = False


class Vector3D:
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Vector:
    @staticmethod
    def add(base, increment):
        """
        Adds the given vectors element-wise in the order: base + increment
        """
        assert len(base) == len(increment)
        return [val + increment[i] for i, val in enumerate(base)]

    @staticmethod
    def subtract(base, increment):
        """
        Subtracts the given vectors element-wise in the order: base - increment
        """
        assert len(base) == len(increment)
        return [val - increment[i] for i, val in enumerate(base)]

    @staticmethod
    def rotate(vector, axis, theta):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians.
        """
        global DEBUG
        if DEBUG:
            print(f"vector: {vector}, axis: {axis}, theta: {math.degrees(theta)}")
        axis = np.asarray(axis)
        axis = axis / math.sqrt(np.dot(axis, axis))
        a = math.cos(theta / 2.0)
        b, c, d = -axis * math.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        rotation_matrix = np.array(
            [
                [aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc],
            ]
        )
        return np.dot(rotation_matrix, vector)
