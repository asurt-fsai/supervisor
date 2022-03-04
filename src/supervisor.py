#!/usr/bin/python3

from inspector import Inspector
import rospy


def main():
    # initialize the node
    rospy.init_node('supervisor')

    # manual inspiction
    inspector = Inspector([
        ("mrmono_pcl", "lidar.launch")
    ])

    inspector.manual_inspect()

    # pool
    rospy.spin()


if __name__ == "__main__":
    main()