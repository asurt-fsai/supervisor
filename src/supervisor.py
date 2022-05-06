#!/usr/bin/python3

import rospy

from inspector import Inspector, Module
from control_commander import ControlCommander
from mission_observer import MissionObserver


def main():
    # Initialize node.
    rospy.init_node('supervisor')

    # Initialize inspector.
    inspector = Inspector([
        Module("mrpython_pcl", "lidar.launch",) # "mrpython_pcl", "rviz.launch"),
    ])

    # Initialize control commander.
    control_commander = ControlCommander()

    # Initialize mission observer.
    mission_observer = MissionObserver()
    mission_observer.on_mission_received(control_commander.run)
    mission_observer.on_mission_finished(control_commander.soft_stop)

    # Start inspection.
    # inspector.manual_inspect()

    # Auto launch.
    inspector.auto_launch()

    # Pool.
    rospy.spin()


if __name__ == "__main__":
    main()