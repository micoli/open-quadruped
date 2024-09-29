from tkinter import *

from py_robot_bus import DisplayMode
from py_robot_bus.bus import Bus
from py_robot_bus.bus.message.input_event import QuadrupedPoseParameter, ChangeDisplayModeCommand, SetNextGaitCommand, \
    SetGaitByNameCommand

from py_robot_bus.bus.message.system import QuitCommand
from py_robot_bus.bus.redis import create_redis

DEBUG = True


class PadControl:
    def __init__(self):
        self.pose_parameter = QuadrupedPoseParameter()
        self.switch = 'x'
        self.mode = DisplayMode.Pose
        self.redis_client = create_redis()

        self.root = Tk()
        self.root.title("Bot Control")
        self.root.geometry("420x130")

        self.switch_label = Label(self.root, text=self.switch)
        self.switch_label.grid(row=0, column=0)

        self.keys = Label(self.root, text='')
        self.keys.grid(row=1, column=0)

        Button(self.root, text="Quit", command=self.quit).grid(row=2, column=0)

        self.command_label = Label(self.root, text="-")
        self.command_label.grid(row=3, column=0)

        available_keys = [
            'q', '1', 'm', 'n', 'g', 'x', 'y',
            'z', 'a', 'p', 'r', 'k', 'l',
            '<Escape>',
            '<Up>', '<Down>',
        ]
        for char in available_keys:
            self.root.bind(char, self.on_press)
        self.keys.config(text=','.join(char for char in available_keys))

    def quit(self):
        self.publish_message(QuitCommand())
        self.root.quit()

    def on_press(self, event):
        inc = 5
        k = event.keysym

        if k in ['q', 'Escape']:
            self.quit()
        if k in ['1']:
            self.pose_parameter.reset()
        if k in ['m']:
            self.mode = DisplayMode.Pose if self.mode != DisplayMode.Pose else DisplayMode.Gait
            self.publish_message(ChangeDisplayModeCommand(mode=self.mode))
            return
        if k in ['n']:
            self.publish_message(SetNextGaitCommand())
            return
        if k in ['g']:
            self.publish_message(SetGaitByNameCommand(name='get_up'))
            return

        if k in ['x']:
            self.switch = 'x'
        if k in ['y']:
            self.switch = 'y'
        if k in ['z']:
            self.switch = 'z'
        if k in ['a']:
            self.switch = 'yaw'
        if k in ['p']:
            self.switch = 'pitch'
        if k in ['r']:
            self.switch = 'roll'
        if k in ['k']:
            self.switch = 'k'
        if k in ['l']:
            self.switch = 'l'
        self.switch_label.config(text=self.switch)
        if k in ['Up', 'Down']:
            factor = (1 if k == 'Up' else -1)
            if self.switch == 'k':
                self.pose_parameter.right_joy[0] += factor
                self.publish_message(self.pose_parameter)
                return
            if self.switch == 'l':
                self.pose_parameter.right_joy[1] += factor
                self.publish_message(self.pose_parameter)
                return

            self.pose_parameter.inc(self.switch, factor * inc)
            self.publish_message(self.pose_parameter)

    def publish_message(self, command):
        self.command_label.config(text=f'{type(command).__name__}\n{command.model_dump_json()}')
        Bus.publish_messages(self.redis_client, command)

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    _pad = PadControl()
    _pad.run()
