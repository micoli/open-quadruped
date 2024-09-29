import pprint
import sys
import time

from py_robot_bus import DisplayMode
from py_robot_bus.bus.message import Channels
from py_robot_bus.bus.message.system import QuitCommand
from py_robot_bus.bus.redis import create_redis
from py_robot_bus.engine.gait import GaitEndException
from py_robot_bus.engine.gait.gaits import Gaits
from py_robot_bus.model.quadruped import Quadruped
from py_robot_bus.bus.message.input_event import QuadrupedPoseParameter, SetNextGaitCommand, ChangeDisplayModeCommand, SetGaitByNameCommand
from py_robot_bus.bus.message.motor_position_command import MotorsPositionCommand, MotorsPosition
from py_robot_bus.bus.message.quadruped_position_event import QuadrupedPositionEvent, Vector3DM
from py_robot_bus.bus import Bus
from pytictoc import TicToc


DEBUG = True


class Engine:
    def __init__(self):
        self.t = TicToc()

        self.redis_client, self.pubsub = create_redis([Channels.input, Channels.sys])

        self.quadruped = Quadruped(
            body_dim=(304, 68),
            limb_lengths=(125, 115),
            offsets=(1, 30),
            leg_offset=(50, 80),
            height=170
        )
        self.pose_parameter = QuadrupedPoseParameter()
        self.gaits = Gaits(time.time())

        self.switch = 'x'
        self.display_mode = DisplayMode.Pose
        self.updated = True

    def _run(self):
        while True:
            try:
                self.t.tic()
                if self.display_mode == DisplayMode.Pose and self.updated:
                    self.quadruped.start_position()
                    self.quadruped.set_pose(self.pose_parameter)
                    self.publish_robot_state()
                    self.updated = False
                if self.display_mode == DisplayMode.Gait:
                    try:
                        pprint.pprint(self.pose_parameter)
                        # 'RR', 'FR', 'FL', 'RL'
                        legs_positions = self.gaits.get_legs_xyz(
                            time.time(),
                            self.pose_parameter.right_joy[0],
                            self.pose_parameter.right_joy[1]
                        )
                        self.quadruped.fully_define(legs_positions)
                        self.publish_robot_state()
                    except GaitEndException:
                        self.display_mode = DisplayMode.Pose
                        self.updated = True

                # if DEBUG:
                #     print(f'fps: {round(1 / self.t.tocvalue(), 1)}, mode: {self.display_mode}', end='\r')

            except IOError:
                time.sleep(1)
            self.redis_listener()
            time.sleep(0.1)

    def publish_robot_state(self):
        rr = self.quadruped.legs[0]
        fr = self.quadruped.legs[1]
        fl = self.quadruped.legs[2]
        rl = self.quadruped.legs[3]
        Bus.publish_messages(self.redis_client, MotorsPositionCommand(
            rr=MotorsPosition(hip=rr.hip_rad, shoulder=rr.shoulder_rad, wrist=rr.wrist_rad),
            fr=MotorsPosition(hip=fr.hip_rad, shoulder=fr.shoulder_rad, wrist=fr.wrist_rad),
            fl=MotorsPosition(hip=fl.hip_rad, shoulder=fl.shoulder_rad, wrist=fl.wrist_rad),
            rl=MotorsPosition(hip=rl.hip_rad, shoulder=rl.shoulder_rad, wrist=rl.wrist_rad),
        ))

        body_vector = self.quadruped.get_body_vectors()
        legs_vector = self.quadruped.get_legs_vectors()

        Bus.publish_messages(self.redis_client, QuadrupedPositionEvent(
            body=Vector3DM.from_vector3d(body_vector),
            rr=Vector3DM.from_vector3d(legs_vector['RR']),
            fr=Vector3DM.from_vector3d(legs_vector['FR']),
            fl=Vector3DM.from_vector3d(legs_vector['FL']),
            rl=Vector3DM.from_vector3d(legs_vector['RL']),
        ))

    def redis_listener(self):
        for message in Bus.receive_messages(self.pubsub):
            if isinstance(message, QuitCommand):
                sys.exit(0)
            if isinstance(message, SetNextGaitCommand):
                self.display_mode = DisplayMode.Gait
                self.gaits.next()
            if isinstance(message, ChangeDisplayModeCommand):
                self.display_mode = message.mode
            if isinstance(message, SetGaitByNameCommand):
                self.display_mode = DisplayMode.Gait
                self.gaits.set_gait_by_name(message.name)
            if isinstance(message, QuadrupedPoseParameter):
                self.updated = True
                self.pose_parameter = message

    def run(self):
        self.publish_robot_state()
        self._run()
