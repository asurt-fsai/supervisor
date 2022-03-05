#!/usr/bin/python3

from inspector import Inspector, Module
import rospy


def main():
    # initialize node
    rospy.init_node('supervisor')

    # initialize inspector
    inspector = Inspector([
        Module("mrpython_pcl", "lidar.launch"),
    ])

    # start inspection
    inspector.manual_inspect()

    # pool
    rospy.spin()


if __name__ == "__main__":
    main()