import numpy
from py_robot_bus.io.visualizer.py_bullet import QuadrupedPyBulletSimulation


def main():
    py_bullet_simulation = QuadrupedPyBulletSimulation(
        "./models/pupper-v2.xacro.xml",
        True,
        start_position=[0, 0, 0.15],
        xacro_variables={
            "torso_width": 0.304,
            "torso_height": 0.1,
            "torso_depth": 0.069,
            "torso_mass": 0.658,
            "hip_width": 0.062,
            "hip_height": 0.104,
            "hip_depth": 0.044,
            "hip_mass": 0.192,
            "hip_torso_offset": 0.031,
            "hip_upper_offset": 0.019,
            "hip_joint_range": numpy.pi / 6,
            "hip_joint_effort": 0,
            "hip_joint_vel": 0,
            "upper_length": 0.125,
            "upper_radius": 0.015,
            "upper_mass": 0.0165,
            "lower_length": 0.115,
            "lower_radius": 0.013,
            "lower_mass": 0.0165,
            "leg_joint_range": numpy.pi / 4,
            "leg_joint_effort": 0,
            "leg_joint_vel": 0,
            "foot_radius": 0.015,
            "foot_mass": 0.001,
            "front_hip_x": 0.106,
            "front_hip_y": 0.04,
            "front_hip_z": 0.0115,
            "rear_hip_x": 0.095,
            "rear_hip_y": 0.04,
            "rear_hip_z": 0.0115,
            "upper_to_lower_joint_angle": -1.57079632679,
            "hip_to_upper_joint_angle": 0.78539816339,
            "hip_to_upper_joint_x": 0.008,
            "foot_joint_z": 0.11,
        },
    )
    py_bullet_simulation.run()
