import numpy
from py_robot_bus.io.visualizer.py_bullet import QuadrupedPyBulletSimulation


def main():
    py_bullet_simulation = QuadrupedPyBulletSimulation(
        "./models/pupper-v2.xacro.xml",
        True,
        start_position=[0, 0, .15],
        xacro_variables={
            "torso_width": .304,
            "torso_height": .1,
            "torso_depth": .069,
            "torso_mass": .658,
            "hip_width": .062,
            "hip_height": .104,
            "hip_depth": .044,
            "hip_mass": .192,
            "hip_torso_offset": .031,
            "hip_upper_offset": .019,
            "hip_joint_range": numpy.pi / 6,
            "hip_joint_effort": 0,
            "hip_joint_vel": 0,
            "upper_length": .125,
            "upper_radius": .015,
            "upper_mass": .0165,
            "lower_length": .115,
            "lower_radius": .013,
            "lower_mass": .0165,
            "leg_joint_range": numpy.pi / 4,
            "leg_joint_effort": 0,
            "leg_joint_vel": 0,
            "foot_radius": .015,
            "foot_mass": .001,
            "front_hip_x": .106,
            "front_hip_y": .04,
            "front_hip_z": .0115,
            "rear_hip_x": .095,
            "rear_hip_y": .04,
            "rear_hip_z": .0115,
            "upper_to_lower_joint_angle": -1.57079632679,
            "hip_to_upper_joint_angle": 0.78539816339,
            "hip_to_upper_joint_x": .008,
            "foot_joint_z": 0.11,
        }
    )
    py_bullet_simulation.run()