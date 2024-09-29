import math

import numpy as np
from py_robot_bus.model.inverse_kinematics.inverse_kinematics import InverseKinematics
from py_robot_bus.model.leg import Leg
from py_robot_bus.math.vector import Vector, Vector3D

DEBUG = False


class Quadruped:
    def __init__(
            self,
            origin=(0, 0, 100),
            body_dim=(230, 78),
            limb_lengths=(107, 130),
            offsets=(10, 60),
            leg_offset=(50, 80),
            height=170
    ):
        """
        body_dim: (length, width,thickness) in mm
        limb_lengths: (upper_arm, bottom_arm) in mm
        offsets: (z_offset, y_offset) in mm
        """
        self.body_dim = body_dim
        self.limb_lengths = limb_lengths
        self.offsets = offsets
        self.init_origin = origin
        self.origin = origin
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.height = height
        self.leg_offset = leg_offset
        self.ik = InverseKinematics(limb_lengths[1], limb_lengths[0], self.body_dim, self.offsets)

        self.body = [
            (origin[0] - self.body_dim[0] / 2, origin[1] - self.body_dim[1] / 2, origin[2]),
            (origin[0] + self.body_dim[0] / 2, origin[1] - self.body_dim[1] / 2, origin[2]),
            (origin[0] + self.body_dim[0] / 2, origin[1] + self.body_dim[1] / 2, origin[2]),
            (origin[0] - self.body_dim[0] / 2, origin[1] + self.body_dim[1] / 2, origin[2]),
            (origin[0] - self.body_dim[0] / 2, origin[1] - self.body_dim[1] / 2, origin[2])
        ]

        self.legs = [
            Leg('RR', (-self.body_dim[0] / 2, -self.body_dim[1] / 2, origin[2])),
            Leg('FR', (+self.body_dim[0] / 2, -self.body_dim[1] / 2, origin[2])),
            Leg('FL', (+self.body_dim[0] / 2, +self.body_dim[1] / 2, origin[2])),
            Leg('RL', (-self.body_dim[0] / 2, +self.body_dim[1] / 2, origin[2]))
        ]
        self.start_position()

    def set_pose(self, quadruped_parameter):
        # Shifting robot pose in cartesian system x-y-z (body-relative)
        self._shift_body_xyz(
            quadruped_parameter.x,
            quadruped_parameter.y,
            quadruped_parameter.z
        )
        # Shifting robot pose in Euler Angles yaw-pitch-roll (body-relative)
        self._shift_body_rotation(
            math.radians(quadruped_parameter.yaw),
            math.radians(quadruped_parameter.pitch),
            math.radians(quadruped_parameter.roll)
        )

    def start_position(self):
        for leg in self.legs:
            leg.origin = leg.init_origin
        self.origin = self.init_origin
        leg_offset_x = self.leg_offset[0]
        leg_offset_y = self.leg_offset[1]
        starting_points = [
            (+leg_offset_x, leg_offset_y, self.height),
            (-leg_offset_x, leg_offset_y, self.height),
            (-leg_offset_x, leg_offset_y, self.height),
            (+leg_offset_x, leg_offset_y, self.height)
        ]
        self.fully_define(starting_points)
        if DEBUG:
            for leg in self.legs:
                print(leg)

    def get_body_vectors(self) -> Vector3D:
        x_data = [vector[0] for vector in self.body]
        y_data = [vector[1] for vector in self.body]
        z_data = [vector[2] for vector in self.body]
        return Vector3D(x_data, y_data, z_data)

    def get_legs_vectors(self):

        legs_vectors = dict()
        for index, leg in enumerate(self.legs):
            leg_vectors = []

            # IMPORTANT
            # changing frame of reference for drawing vectors
            wrist_rad = np.pi + leg.wrist_rad
            shoulder_rad = leg.shoulder_rad
            hip_rad = -leg.hip_rad

            if index < 2:
                horizontal_offset = -self.offsets[1]
            else:
                horizontal_offset = self.offsets[1]

            hip_axis = (1, 0, 0)
            shoulder_axis = (0, 1, 0)
            wrist_axis = (0, 1, 0)

            # respective corner of robot
            leg_vectors.append(leg.origin)
            # z offset on hip
            leg_vectors.append(Vector.add(leg_vectors[0], (0, 0, -self.offsets[0])))
            # y offset on hip
            leg_vectors.append(Vector.add(leg_vectors[-1], (0, horizontal_offset, 0)))
            # upper arm
            leg_vectors.append(Vector.add(leg_vectors[-1], (0, 0, -self.limb_lengths[0])))
            # lower arm
            leg_vectors.append(Vector.add(leg_vectors[-1], (0, 0, -self.limb_lengths[1])))

            # apply rotations
            # wrist rotation 1
            leg_vectors[-1] = Vector.add(
                leg_vectors[-2],
                Vector.rotate(
                    Vector.subtract(leg_vectors[-1], leg_vectors[-2]),
                    wrist_axis,
                    wrist_rad
                )
            )
            # wrist rotation 2
            leg_vectors[-1] = Vector.add(
                leg_vectors[-3],
                Vector.rotate(
                    Vector.subtract(leg_vectors[-1], leg_vectors[-3]),
                    shoulder_axis,
                    shoulder_rad
                )
            )
            # wrist rotation 3
            leg_vectors[-1] = Vector.add(
                leg_vectors[0],
                Vector.rotate(
                    Vector.subtract(leg_vectors[-1], leg_vectors[0]),
                    hip_axis,
                    hip_rad
                )
            )
            # shoulder rotation 1
            leg_vectors[-2] = Vector.add(
                leg_vectors[-3],
                Vector.rotate(
                    Vector.subtract(leg_vectors[-2], leg_vectors[-3]),
                    shoulder_axis,
                    shoulder_rad
                )
            )
            # shoulder rotation 2
            leg_vectors[-2] = Vector.add(
                leg_vectors[0],
                Vector.rotate(
                    Vector.subtract(leg_vectors[-2], leg_vectors[0]),
                    hip_axis,
                    hip_rad
                )
            )
            # hip rotation 1
            leg_vectors[-3] = Vector.add(
                leg_vectors[0],
                Vector.rotate(
                    Vector.subtract(leg_vectors[-3], leg_vectors[0]),
                    hip_axis,
                    hip_rad
                )
            )
            # hip rotation 1
            leg_vectors[-4] = Vector.add(
                leg_vectors[0],
                Vector.rotate(
                    Vector.subtract(leg_vectors[-4], leg_vectors[0]),
                    hip_axis,
                    hip_rad
                )
            )

            for i, vector in enumerate(leg_vectors):
                leg_vectors[i] = Vector.rotate(leg_vectors[i], [0, 0, 1], -self.yaw)
                leg_vectors[i] = Vector.rotate(leg_vectors[i], [0, 1, 0], -self.pitch)
                leg_vectors[i] = Vector.rotate(leg_vectors[i], [1, 0, 0], -self.roll)

            x_data = [vector[0] for vector in leg_vectors]
            y_data = [vector[1] for vector in leg_vectors]
            z_data = [vector[2] for vector in leg_vectors]
            legs_vectors[leg.name] = Vector3D(x_data, y_data, z_data)
        return legs_vectors

    def dump_legs(self,):
        for (index, leg) in enumerate(self.legs):
            print(f"""
                leg:{leg.name}
                angles: [
                  hip: {math.degrees(leg.hip_rad)},
                  shoulder: {math.degrees(leg.shoulder_rad)},
                  wrist: {math.degrees(leg.wrist_rad)}
                ]""")

    def fully_define(self, leg_points):
        global DEBUG
        for i, leg in enumerate(self.legs):
            leg.x = leg_points[i][0]
            leg.y = leg_points[i][1]
            leg.z = leg_points[i][2]

        leg_angle_sets = self.ik.local_translation_engine(leg_points)

        for i, leg_angle_set in enumerate(leg_angle_sets):
            self.legs[i].hip_rad = leg_angle_set[0]
            self.legs[i].shoulder_rad = leg_angle_set[1]
            self.legs[i].wrist_rad = leg_angle_set[2]

    def _get_points_from_buffer(self):
        return [(leg.x, leg.y, leg.z) for leg in self.legs]

    def _shift_body_xyz(self, x, y, z):
        local_x_shift = x
        local_y_shift = y
        local_z_shift = z
        shifts = (local_x_shift, local_y_shift, local_z_shift)
        for i, leg in enumerate(self.legs):
            if i == 1 or i == 2:
                leg.x += -local_x_shift
            else:
                leg.x += local_x_shift
            if i < 2:
                leg.y += local_y_shift
            else:
                leg.y += -local_y_shift
            leg.z += local_z_shift
        self.fully_define(self._get_points_from_buffer())
        self.origin = (
            self.origin[0] + local_x_shift,
            self.origin[1] + local_y_shift,
            self.origin[2] + local_z_shift
        )

        self.body = [
            (self.origin[0] - self.body_dim[0] / 2, self.origin[1] - self.body_dim[1] / 2, self.origin[2]),
            (self.origin[0] + self.body_dim[0] / 2, self.origin[1] - self.body_dim[1] / 2, self.origin[2]),
            (self.origin[0] + self.body_dim[0] / 2, self.origin[1] + self.body_dim[1] / 2, self.origin[2]),
            (self.origin[0] - self.body_dim[0] / 2, self.origin[1] + self.body_dim[1] / 2, self.origin[2]),
            (self.origin[0] - self.body_dim[0] / 2, self.origin[1] - self.body_dim[1] / 2, self.origin[2])
        ]
        for leg in self.legs:
            leg.origin = [leg.origin[i] + shift for i, shift in enumerate(shifts)]

    def _shift_body_rotation(self, yaw, pitch, roll):
        try:
            # YAW CALCULATIONS
            self.yaw = yaw
            self.pitch = pitch
            self.roll = roll
            # YAW

            for i, leg in enumerate(self.legs):
                # Front Right Leg
                if i == 1:
                    x_g = self.init_origin[0] + self.body_dim[0] / 2 + leg.x
                    y_g = self.init_origin[1] + self.body_dim[1] / 2 + leg.y
                    alpha_0 = math.atan(x_g / y_g)
                    radius = math.sqrt(x_g**2 + y_g**2)
                    alpha_1 = alpha_0 + yaw
                    x_g = radius * math.sin(alpha_1)
                    y_g = radius * math.cos(alpha_1)
                    leg.x = x_g - (self.body_dim[0] / 2 + self.init_origin[0])
                    leg.y = y_g - (self.body_dim[1] / 2 + self.init_origin[1])
                # Front Left Leg
                if i == 2:
                    x_g = self.init_origin[0] + self.body_dim[0] / 2 + leg.x
                    y_g = self.init_origin[1] + self.body_dim[1] / 2 + leg.y
                    alpha_0 = math.atan(x_g / y_g)
                    radius = math.sqrt(x_g**2 + y_g**2)
                    alpha_1 = alpha_0 - yaw
                    x_g = radius * math.sin(alpha_1)
                    y_g = radius * math.cos(alpha_1)
                    leg.x = x_g - (self.body_dim[0] / 2 + self.init_origin[0])
                    leg.y = y_g - (self.body_dim[1] / 2 + self.init_origin[1])
                # Back Right Leg
                if i == 0:
                    x_g = self.init_origin[0] + self.body_dim[0] / 2 + leg.x
                    y_g = self.init_origin[1] + self.body_dim[1] / 2 + leg.y
                    alpha_0 = math.atan(y_g / x_g)
                    radius = math.sqrt(x_g**2 + y_g**2)
                    alpha_1 = alpha_0 + yaw
                    x_g = radius * math.cos(alpha_1)
                    y_g = radius * math.sin(alpha_1)
                    leg.x = -x_g + (self.init_origin[0] + self.body_dim[0] / 2)
                    leg.y = y_g - (self.init_origin[1] + self.body_dim[1] / 2)
                # Back Left Leg
                if i == 3:
                    x_g = self.init_origin[0] + self.body_dim[0] / 2 + leg.x
                    y_g = self.init_origin[1] + self.body_dim[1] / 2 + leg.y
                    alpha_0 = math.atan(y_g / x_g)
                    radius = math.sqrt(x_g**2 + y_g**2)
                    alpha_1 = alpha_0 - yaw
                    x_g = radius * math.cos(alpha_1)
                    y_g = radius * math.sin(alpha_1)
                    leg.x = -x_g + (self.init_origin[0] + self.body_dim[0] / 2)
                    leg.y = y_g - (self.init_origin[1] + self.body_dim[1] / 2)

            # PITCH CALCULATIONS
            sig_z = sum([leg.z for leg in self.legs]) / 4
            z_i = self.body_dim[0] / 2 * math.sin(pitch)
            x_i = z_i / math.tan((math.pi - pitch) / 2)
            for i, leg in enumerate(self.legs):
                if i == 1 or i == 2:  # front
                    self.legs[i].z = sig_z + z_i
                    self.legs[i].x = self.legs[i].x - x_i
                if i == 0 or i == 3:  # back
                    self.legs[i].z = sig_z - z_i
                    self.legs[i].x = self.legs[i].x - x_i

            # ROLL CALCULATIONS
            sig_z_front = (self.legs[1].z + self.legs[2].z) / 2
            sig_z_back = (self.legs[0].z + self.legs[3].z) / 2
            z_i = self.body_dim[1] / 2 * math.sin(roll)
            y_i = z_i / math.tan((math.pi - roll) / 2)
            for i, leg in enumerate(self.legs):
                if i == 0:
                    self.legs[i].z = sig_z_back + z_i
                    leg.y -= y_i
                if i == 1:
                    self.legs[i].z = sig_z_front + z_i
                    leg.y -= y_i
                if i == 2:
                    self.legs[i].z = sig_z_front - z_i
                    leg.y += y_i
                if i == 3:
                    self.legs[i].z = sig_z_back - z_i
                    leg.y += y_i
        except Exception as e:
            print("Out of bounds." + str(e))

        self.fully_define(self._get_points_from_buffer())

        for i, vector in enumerate(self.body):
            self.body[i] = Vector.rotate(self.body[i], [0, 0, 1], -self.yaw)
            self.body[i] = Vector.rotate(self.body[i], [0, 1, 0], -self.pitch)
            self.body[i] = Vector.rotate(self.body[i], [1, 0, 0], -self.roll)
