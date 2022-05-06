import enum

import rospy
from std_msgs.msg import UInt16


class ControlCommand(enum.Enum):
    """
        Available control commands.
    """

    Stop = 0
    Soft_Stop = 1
    Run = 2


class ControlCommander():
    """
        Issues commands to the control module.
    """

    def __init__(self) -> None:
        self.pub = rospy.Publisher(rospy.get_param('control_command_topic', 'sp_state_control'), UInt16, queue_size=1)


    def __command(self, command: ControlCommand):
        self.pub.publish(command)


    def stop(self):
        self.__command(ControlCommand.Stop)


    def soft_stop(self):
        self.__command(ControlCommand.Soft_Stop)


    def run(self):
        self.__command(ControlCommand.Run)