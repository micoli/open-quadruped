import pprint
import sys

import pybullet
import time
import pybullet_data
import pybullet_utils.bullet_client as bullet_client

from py_robot_bus.bus import Bus
from py_robot_bus.bus.message import Channels
from py_robot_bus.bus.message.motor_position_command import MotorsPositionCommand
from py_robot_bus.bus.message.system import QuitCommand
from py_robot_bus.bus.redis import create_redis

from py_robot_bus.io.visualizer.urdf.helpers import UrdfLoader
import numpy


class MotorsPositionCommandDecoder:
    def __init__(self, names):
        pi = numpy.pi
        self.offset_hip = pi / 8
        self.offset_shoulder = pi / 4
        self.offset_wrist = pi / 2
        self.indices = [
            names.index(f'FR_hip'), names.index(f'FR_upper'), names.index(f'FR_lower'),
            names.index(f'FL_hip'), names.index(f'FL_upper'), names.index(f'FL_lower'),
            names.index(f'RR_hip'), names.index(f'RR_upper'), names.index(f'RR_lower'),
            names.index(f'RL_hip'), names.index(f'RL_upper'), names.index(f'RL_lower'),
        ]

    def decode(self, pos):
        return [
            pos.fr.hip - self.offset_hip, pos.fr.shoulder - self.offset_shoulder, pos.fr.wrist - self.offset_wrist,
            pos.fl.hip + self.offset_hip, pos.fl.shoulder - self.offset_shoulder, pos.fl.wrist - self.offset_wrist,
            pos.rr.hip - self.offset_hip, pos.rr.shoulder - self.offset_shoulder, pos.rr.wrist - self.offset_wrist,
            pos.rl.hip + self.offset_hip, pos.rl.shoulder - self.offset_shoulder, pos.rl.wrist - self.offset_wrist
        ]


class QuadrupedPyBulletSimulation:
    def __init__(
            self,
            urdf_path: str,
            with_debug_parameters: bool = True,
            start_position=None,
            xacro_variables=None
    ):
        if start_position is None:
            start_position = [0, 0, .25]
        if xacro_variables is None:
            xacro_variables = {}

        self.redis_client, self.pubsub = create_redis([Channels.position, Channels.sys])

        self.frequency = 1.0 / 240
        self.start = time.time()
        self.with_debug_parameters = with_debug_parameters
        self.start_position = start_position
        self.motor_names = []
        self.debug_parameters = []
        self.physics_client = bullet_client.BulletClient(pybullet.GUI)
        self.physics_client.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
        self.physics_client.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
        self.physics_client.configureDebugVisualizer(pybullet.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)
        self.start_orientation = pybullet.getQuaternionFromEuler([0, 0, 0])

        self.physics_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.physics_client.resetDebugVisualizerCamera(
            cameraDistance=1,
            cameraYaw=135,
            cameraPitch=-45,
            cameraTargetPosition=[0, 0, 0]
        )
        self.physics_client.setTimeStep(self.frequency)

        self.physics_client.loadURDF("plane.urdf")
        urdf = UrdfLoader(
            urdf_path,
            xacro_variables=xacro_variables
        )
        self.robot = self.physics_client.loadURDF(
            urdf.path,
            self.start_position,
            self.start_orientation,
            useFixedBase=0,
            useMaximalCoordinates=0,
            globalScaling=1,
        )

        self.physics_client.setGravity(0, 0, -9.81/2)
        self.physics_client.setRealTimeSimulation(1)
        self.physics_client.stepSimulation()
        self._load_motors()
        self.motors_position_command_decoder = MotorsPositionCommandDecoder(self.motor_names)
        self.physics_client.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

    def _load_motors(self):
        for index in range(self.physics_client.getNumJoints(self.robot)):
            joint_info = pybullet.getJointInfo(self.robot, index)
            joint_name = joint_info[1].decode()
            self.motor_names.append(joint_name[:-6])
            if not self.with_debug_parameters:
                continue
            if "_joint" != joint_name[-6:]:
                continue
            j_min = joint_info[8]
            j_max = joint_info[9]
            j_center = (j_min + j_max) / 2
            self.debug_parameters.append(
                self.physics_client.addUserDebugParameter(joint_name[:-6], j_min, j_max, j_center)
            )

    def run(self):
        while self.physics_client.isConnected():
            # if self.with_debug_parameters:
            #     for index in range(len(self.motor_names)):
            #         try:
            #             self.set_robot_joint_angle(
            #                 index,
            #                 self.physics_client.readUserDebugParameter(self.debug_parameters[index])
            #             )
            #         except pybullet.error as err:
            #             pprint.pprint(err)

            self.redis_listener()
            try:
                self.step_simulation()
            except pybullet.error as err:
                pprint.pprint(err)

            time.sleep(self.frequency)

    def redis_listener(self):
        for message in Bus.receive_messages(self.pubsub):
            if isinstance(message, QuitCommand):
                sys.exit(0)
            if isinstance(message, MotorsPositionCommand):
                self.physics_client.setJointMotorControlArray(
                    self.robot,
                    self.motors_position_command_decoder.indices,
                    pybullet.POSITION_CONTROL,
                    targetPositions=self.motors_position_command_decoder.decode(message)
                )

    def step_simulation(self):
        self.physics_client.stepSimulation()

    def set_robot_joint_angle(self, index, position):
        try:
            self.physics_client.setJointMotorControl2(
                self.robot,
                index,
                pybullet.POSITION_CONTROL,
                targetPosition=position
            )
        except TypeError as err:
            pprint.pprint(err)
        except pybullet.error as err:
            pprint.pprint(err)


if __name__ == '__main__':
    py_bullet_simulation = QuadrupedPyBulletSimulation("../../../models/pupper-v2.xacro.xml", True)
    py_bullet_simulation.run()
