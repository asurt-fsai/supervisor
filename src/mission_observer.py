from typing import Callable

import rospy

from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

from asurt_msgs.msg import CanState, RoadState

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
        self.__last_road_state: RoadState = RoadState(0, 0)

        self.__mission_to_checker: dict[int, Callable[[RoadState], bool]] = {
            CanState.AMI_SKIDPAD: self.__is_finished_skidpad,
            CanState.AMI_ACCELERATION: self.__is_finished_acceleration,
            CanState.AMI_AUTOCROSS: self.__is_finished_autocross,
            CanState.AMI_TRACK_DRIVE: self.__is_finished_trackdrive,
        }

        self.__subscriber_mission = rospy.Subscriber(rospy.get_param('/ros_can/state', 'CanState'), CanState, self.__can_state_handler, queue_size=1)
        self.__publisher_mission_status = rospy.Publisher(rospy.get_param('mission_status_topic', '/ros_can/mission_flag'), Bool, queue_size=1)
        self.__publisher_driving_flag = rospy.Publisher(rospy.get_param('driving_flag_topic', '/state_machine/driving_flag'), Bool, queue_size=1)


    def __can_state_handler(self, can_state: CanState):
        # Act on the mission.
        self.__mission = can_state.ami_state
        rospy.set_param('navigation_mission', self.__mission) # Notify navigation via param server.
        if self.__mission_received_callback is not None:
            self.__mission_received_callback(self.__mission)

        # Act for AS_READY transition (driving flag).
        if self.__as_state != can_state.AS_READY and can_state.as_state == can_state.AS_READY:
            self.__publisher_driving_flag.publish(Bool(True))

        # Act for AS_DRIVING transition (run the system).
        previous_state = self.__as_state
        self.__as_state = can_state.as_state
        if previous_state != can_state.AS_DRIVING and can_state.as_state == can_state.AS_DRIVING:
            # Inspection missions.
            if self.__mission == CanState.AMI_DDT_INSPECTION_A:
                self.__start_inspection_a()
            elif self.__mission == CanState.AMI_DDT_INSPECTION_B:
                self.__start_inspection_b()
            elif self.__mission == CanState.AMI_AUTONOMOUS_DEMO:
                self.__start_autonomous_demo()
                self.__subscriber_road_state = rospy.Subscriber(rospy.get_param('road_state_topic', 'RoadState'), RoadState, self.__road_state_autonomous_demo_handler, queue_size=1)
            # Main missions.
            else:
                self.__launch_system()
                self.__is_finished_checker = self.__mission_to_checker[self.__mission]
                self.__subscriber_road_state = rospy.Subscriber(rospy.get_param('road_state_topic', 'RoadState'), RoadState, self.__road_state_handler, queue_size=1)
        

    def __road_state_handler(self, road_state: RoadState):
        self.__last_road_state = road_state

        
    def __road_state_handler(self, road_state: RoadState):
        self.__last_road_state = road_state
        self.__is_finished(road_state)


    def __is_finished(self, road_state: RoadState) -> bool:
        is_finished: bool = self.__is_finished_checker(road_state)

        if is_finished:
            # Stop the vehicle.
            self.__control_commander.soft_stop()
            # Wait until deccelerated.
            while self.__control_commander.current_speed > 9:
                rospy.sleep(0.1)
            # Signal mission status.
            self.__publisher_mission_status.publish(Bool(True))
            if self.__mission_finished_callback is not None:
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

    
    def __start_inspection_a(self):
        ###############################
        # Sweep full streeing range.
        ###############################

        # Steer left.
        self.__control_commander.set_steering_angle(-27.2)
        # Until steered
        while self.__control_commander.current_steering_angle > -24:
            rospy.sleep(0.1)
        # Buffer.
        rospy.sleep(1)

        # Steer right.
        self.__control_commander.set_steering_angle(27.2)
        # Until steered
        while self.__control_commander.current_steering_angle < 24:
            rospy.sleep(0.1)
        # Buffer.
        rospy.sleep(1)

        # Steer to straight ahead position.
        self.__control_commander.set_steering_angle(0)
        while not (-0.5 < self.__control_commander.current_steering_angle < 0.5):
            rospy.sleep(0.1)
        # Buffer.
        rospy.sleep(1)

        ###############################
        # Acceleration.
        ###############################

        # Ramp up.
        for i in range(1, 11):
            rospy.sleep(1)
            self.__control_commander.set_speed_rpm(max(i * 25, 200))
        
        # Stop.
        for i in range(1, 6):
            rospy.sleep(1)
            self.__control_commander.set_speed_rpm(min(200 - i * 50, 0))

        # Signal mission finish status.
        self.__publisher_mission_status.publish(Bool(True))
        if self.__mission_finished_callback is not None:
            self.__mission_finished_callback()
        

    def __start_inspection_b(self):
        # Spin up the speed to 50 rpm.
        self.__control_commander.set_speed_rpm(51)
        # Until reached.
        while self.__control_commander.current_speed < 50:
            rospy.sleep(0.1)
        # Buffer.
        rospy.sleep(1)

        # Trigger EBS system.
        # Signal mission finish status.
        self.__publisher_mission_status.publish(Bool(True))
        if self.__mission_finished_callback is not None:
            self.__mission_finished_callback()

    
    def __start_autonomous_demo(self):
        ###############################
        # Steer left and right, then forward.
        ###############################

        # Steer left.
        self.__control_commander.set_steering_angle(-27.2)
        rospy.sleep(1)
        # Steer right.
        self.__control_commander.set_steering_angle(27.2)
        rospy.sleep(2)
        # Steer to straight ahead position.
        self.__control_commander.set_steering_angle(0)
        rospy.sleep(1.5)

        ###############################
        # Launch system to stay in track.
        # Add middlware to adjust the navigation
        # commands according to the demo mission.
        ###############################

        def adjust_speed(command: AckermannDriveStamped, speed) -> AckermannDriveStamped:
            command = AckermannDriveStamped(
                AckermannDrive(
                    command.drive.steering_angle,
                    0,
                    speed,
                    0,
                    0
                )
            )
            return command

        # Accelerate.
        self.__control_commander.add_middleware("accelerate", lambda x: adjust_speed(x, 16 * 3.6))
        self.__launch_system()

        # For 10 meters.
        while self.__last_road_state.distance < 10:
            rospy.sleep(0.05)
        
        # Decelerate.
        self.__control_commander.add_middleware("decelerate", lambda x: adjust_speed(x, 0))
        self.__control_commander.remove_middleware("accelerate")

        # Until stopped.
        while self.__control_commander.current_speed > 0.1:
            rospy.sleep(0.1)
        # Buffer.
        rospy.sleep(2)

        # Accelerate.
        start_distance = self.__last_road_state.distance
        self.__control_commander.add_middleware("accelerate", lambda x: adjust_speed(x, 16 * 3.6))
        self.__control_commander.remove_middleware("decelerate")

        # For 10 meters.
        while (self.__last_road_state.distance - start_distance) < 10:
            rospy.sleep(0.05)
        
        # Trigger EBS system.
        # Signal mission finish status.
        self.__publisher_mission_status.publish(Bool(True))
        if self.__mission_finished_callback is not None:
            self.__mission_finished_callback()


    def on_mission_received(self, callback: Callable[[int], bool]) -> None:
        self.__mission_received_callback = callback


    def on_mission_finished(self, callback: Callable[[], None]) -> None:
        self.__mission_finished_callback = callback
