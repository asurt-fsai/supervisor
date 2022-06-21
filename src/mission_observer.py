import enum
from dataclasses import dataclass
from typing import Callable

import rospy
from std_msgs.msg import UInt16, String

from asurt_msgs.msg import CanState
from asurt_msgs.msg import RoadState

from control_commander import ControlCommander


class MissionObserver():
    """
        Observers the progress towards a specific mission
        to fire the respective events.
    """

    def __init__(self, launch_system: Callable, control_commmander: ControlCommander) -> None:
        self.__as_state: int = CanState.AS_OFF
        self.__mission: int = CanState.AMI_NOT_SELECTED
        self.__mission_received_callback: Callable[[int], None] = None
        self.__mission_finished_callback: Callable[[], None] = None
        self.__is_finished_checker: Callable[[RoadState], bool] = None
        self.__launch_system: Callable = None
        self.__control_commander: ControlCommander = control_commmander
        self.__subscriber_mission: rospy.Subscriber = None
        self.__subscriber_road_state: rospy.Subscriber = None

        self.__mission_to_checker: dict[int, Callable[[RoadState], bool]] = {
            CanState.AMI_SKIDPAD: self.__is_finished_skidpad,
            CanState.AMI_ACCELERATION: self.__is_finished_acceleration,
            CanState.AMI_AUTOCROSS: self.__is_finished_autocross,
            CanState.AMI_TRACK_DRIVE: self.__is_finished_trackdrive,
        }

        self.__subscriber_mission = rospy.Subscriber(rospy.get_param('/ros_can/state', 'CanState'), CanState, self.__can_state_handler, queue_size=1)
        return


    def __can_state_handler(self, can_state: CanState):
        # Act for AS_DRIVING transition (Run the system).
        if self.__as_state != can_state.AS_DRIVING and can_state.as_state == can_state.AS_DRIVING:
            self.__launch_system()
            self.__control_commander.run()
        self.__as_state = can_state.as_state

        # Act on the mission.
        self.__mission = can_state.ami_state

        if self.__mission_received_callback is not None:
            self.__mission_received_callback(self.__mission)

        self.__is_finished_checker = self.__mission_to_checker[self.__mission]
        self.__subscriber_road_state = rospy.Subscriber(rospy.get_param('road_state_topic', 'RoadState'), RoadState, self.__is_finished, queue_size=1)


    def __is_finished(self, road_state: RoadState) -> bool:
        is_finished: bool = self.__is_finished_checker(road_state)

        # Stop the vehicle.
        self.__control_commander.soft_stop()

        if is_finished and self.__mission_finished_callback is not None:
            self.__mission_finished_callback()
        
        return is_finished
    

    def __is_finished_skidpad(self, road_state: RoadState) -> bool:
        if road_state.laps >= 1:
            return True
        
        return False


    def __is_finished_acceleration(self, road_state: RoadState) -> bool:
        if road_state.distance >= 76:
            return True

        return False


    def __is_finished_autocross(self, road_state: RoadState) -> bool:
        if road_state.laps >= 1:
            return True
        
        return False


    def __is_finished_trackdrive(self, road_state: RoadState) -> bool:
        if road_state.laps >= 10:
            return True
        
        return False


    def on_mission_received(self, callback: Callable[[int], bool]) -> None:
        self.__mission_received_callback = callback


    def on_mission_finished(self, callback: Callable[[], None]) -> None:
        self.__mission_finished_callback = callback
