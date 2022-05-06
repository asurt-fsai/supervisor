import enum
from dataclasses import dataclass
from typing import Callable

import rospy
from std_msgs.msg import UInt16, String
from asurt_msgs.msg import RoadState


class Mission(enum.Enum):
    """
        Available missions.
    """

    Skidpad = 'Skidpad'
    Acceleration = 'Acceleration'
    Autocross = 'Autocross'


class MissionObserver():
    """
        Observers the progress towards a specific mission
        to fire the respective events.
    """

    def __init__(self, mission: Mission = None) -> None:
        self.__mission: Mission = mission
        self.__mission_received_callback: Callable[[Mission], None] = None
        self.__mission_finished_callback: Callable[[], None] = None
        self.__is_finished_checker: Callable[[RoadState], bool] = None
        self.__sub_road_state = None
        self.__sub_mission = None

        self.__mission_to_checker: dict[Mission, Callable[[RoadState], bool]] = {
            Mission.Skidpad: self.__is_finished_skidpad,
            Mission.Acceleration: self.__is_finished_acceleration,
            Mission.Autocross: self.__is_finished_autocross,
        }

        # Passing a mission in the constructor
        # overrides the one coming from ros topic.
        if mission is not None:
            # TODO: get mission topic and message names.
            self.__sub_mission = rospy.Subscriber(rospy.get_param('mission_topic', 'Mission'), String, self.__mission_handler, queue_size=1)
            return

        self.__mission_handler(String(mission))

    def __mission_handler(self, mission: String):
        mission = mission.data.casefold()
        if mission == 'skidpad'.casefold():
            self.__mission = Mission.Skidpad

        if self.__mission_received_callback is not None:
            self.__mission_received_callback(self.__mission)

        self.__is_finished_checker = self.__mission_to_checker[self.__mission]

        self.__sub_road_state = rospy.Subscriber(rospy.get_param('road_state_topic', 'RoadState'), RoadState, self.__is_finished, queue_size=1)

    def __is_finished(self, road_state: RoadState) -> bool:
        is_finished: bool = self.__is_finished_checker(road_state)

        if is_finished and self.__mission_finished_callback is not None:
            self.__mission_finished_callback()
        
        return is_finished
    
    def __is_finished_skidpad(self, road_state: RoadState) -> bool:
        return True

    def __is_finished_acceleration(self, road_state: RoadState) -> bool:
        return True

    def __is_finished_autocross(self, road_state: RoadState) -> bool:
        return True

    def on_mission_received(self, callback: Callable[[Mission], bool]) -> None:
        self.__mission_received_callback = callback

    def on_mission_finished(self, callback: Callable[[], None]) -> None:
        self.__mission_finished_callback = callback
