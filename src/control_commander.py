from typing import Callable

import rospy

from std_msgs.msg import UInt16
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

from asurt_msgs.msg import WheelSpeeds, WheelSpeedsStamped


class ControlCommander():
    """
        Issues commands to the control module.
    """

    WHEEL_RADIUS = 0.253


    def __init__(self) -> None:
        # Commands to the low-level control via the CAN.
        self.publisher_can_command = rospy.Publisher(
            rospy.get_param('can_command_topic', '/cmd'), 
            AckermannDriveStamped, 
            queue_size=1)
        # Commands from the navigation module.
        self.subscriber_navigation_command = rospy.Subscriber(
            rospy.get_param('navigation_commnad_topic', 'sub_control_actions'), 
            AckermannDriveStamped, 
            callback=self.__navigation_command_handler)
        # Wheel speeds and steering angle feedback.
        self.subscriber_wheel_speedsfeedback = rospy.Subscriber(
            rospy.get_param('can_wheel_speeds_topic', '/ros_can/wheel_speeds'), 
            WheelSpeedsStamped, 
            callback=self.__update_last_wheel_speeds)

        # State.
        self.last_steering_angle_command = 0
        self.last_wheel_speeds = WheelSpeedsStamped(WheelSpeeds(0, 0, 0, 0, 0))
        self.middlewares: dict[str, Callable[[AckermannDriveStamped], AckermannDriveStamped]] = {}


    def __navigation_command_handler(self, command: AckermannDriveStamped):
        for middleware in self.middlewares.values():
            command = middleware(command)
        self.__low_level_command(command)
    

    def __low_level_command(self, command: AckermannDriveStamped):
        self.last_steering_angle_command = command.drive.steering_angle
        self.publisher_can_command.publish(command)

    
    def __update_last_wheel_speeds(self, msg: WheelSpeedsStamped):
        self.last_wheel_speeds = msg


    @property
    def current_steering_angle(self) -> float:
        """
            Last steering angle feedback in degrees.
        """
        return self.last_wheel_speeds.speeds.steering * 57.296


    @property
    def current_speed(self) -> float:
        """
            Last speed feedback in m/s.
        """
        return (
            self.last_wheel_speeds.speeds.lf_speed \
            + self.last_wheel_speeds.speeds.rf_speed \
            + self.last_wheel_speeds.speeds.lb_speed \
            + self.last_wheel_speeds.speeds.rb_speed \
            ) / 4


    def stop(self):
        self.soft_stop()


    def soft_stop(self):
        cmd = AckermannDriveStamped(
            AckermannDrive(
                self.last_steering_angle_command,
                0,
                0,
                0,
                0
            )
        )
        self.__low_level_command(cmd)

    
    def set_speed_rpm(self, rpm: float):
        speed = rpm * 0.10472 * ControlCommander.WHEEL_RADIUS
        
        cmd = AckermannDriveStamped(
            AckermannDrive(
                0,
                0,
                speed,
                0,
                0
            )
        )
        self.__low_level_command(cmd)


    def set_speed_kph(self, kph: float):
        speed = kph / 3.6
        
        cmd = AckermannDriveStamped(
            AckermannDrive(
                0,
                0,
                speed,
                0,
                0
            )
        )
        self.__low_level_command(cmd)


    def set_steering_angle(self, angle: float):
        cmd = AckermannDriveStamped(
            AckermannDrive(
                angle,
                0,
                0,
                0,
                0
            )
        )
        self.__low_level_command(cmd)


    def add_middleware(self, key: str, middleware: Callable[[AckermannDriveStamped], AckermannDriveStamped]):
        """
            A function that transforms the command originating from
            navigation into a new command.
        """

        self.middlewares[key] = middleware

    
    def remove_middleware(self, key: str):
        """
            Remove a middleware using its key.
        """

        self.middlewares.pop(key)