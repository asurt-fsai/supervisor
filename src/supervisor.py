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
        Module("mrpython_pcl", "test.launch",) # "mrpython_pcl", "rviz.launch"),
    ])

    # Initialize control commander.
    control_commander = ControlCommander()

    # Initialize mission observer.
    mission_observer = MissionObserver(
        launch_system = inspector.auto_launch,
        control_commmander = control_commander,
    )

    # Ex. Start a manual inspection.
    # inspector.manual_inspect()

    # Ex. Auto launch.
    # inspector.auto_launch()

    # Pool.
    rospy.spin()


if __name__ == "__main__":
    main()