import rospy
from std_msgs.msg import UInt16
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

from asurt_msgs.msg import WheelSpeeds, WheelSpeedsStamped


class ControlCommander():
    """
        Issues commands to the control module.
    """

    def __init__(self) -> None:
        # Commands to the low-level control via the CAN.
        self.publisher_can_command = rospy.Publisher(rospy.get_param('can_command_topic', '/cmd'), AckermannDriveStamped, queue_size=1)
        # Commands from the navigation module.
        self.subscriber_navigation_command = rospy.Subscriber(rospy.get_param('navigation_commnad_topic', 'sub_control_actions'), AckermannDriveStamped, callback=self.__low_level_command)
        # Wheel speeds and steering angle feedback.
        self.subscriber_wheel_speedsfeedback = rospy.Subscriber(rospy.get_param('can_wheel_speeds_topic', '/ros_can/wheel_speeds'), WheelSpeedsStamped, callback=self.__update_last_wheel_speeds)

        # State.
        self.last_steering_angle_command = 0
        self.last_wheel_speeds = WheelSpeedsStamped(WheelSpeeds(0, 0, 0, 0, 0))


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
            Last steering angle feedback in degrees.
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

    
    def set_speed(self, rpm: float):
        wheel_radius = 0.253

        speed = rpm * 0.10472 * wheel_radius
        
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

