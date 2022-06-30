#!/usr/bin/python3

import rospy

from inspector import Inspector, Module
from control_commander import ControlCommander
from mission_observer import MissionObserver
from visualization_msgs.msg import Marker, MarkerArray

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
import threading
import time
class Visualizer:
    def __init__(self, publish_name):
        self.counter = 0
        self.publish_name = publish_name
        self.publisher = rospy.Publisher(publish_name, MarkerArray, queue_size=1)

    def del_msg(self, frame_id):
        msg_del = Marker()
        msg_del.header.frame_id = frame_id
        msg_del.header.stamp = rospy.Time.now()
        msg_del.type = Marker.TEXT_VIEW_FACING
        msg_del.action = Marker.DELETEALL
        return msg_del
    def dets_to_marker(self, x, y, text, col_type,frame_id, idx):
        msg = Marker()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        msg.ns = str(self.counter)
        msg.id = idx
        msg.type = Marker.TEXT_VIEW_FACING
        msg.action = Marker.ADD
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.scale.z = 0.5
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1
        msg.text = text

        color = [1,1,1]
        if col_type in [0,1]: # Yellow
            color = [0,1,1]
        elif col_type in [3,4,5]:
            color = [1,0,0]

        msg.color.r = color[0]
        msg.color.g = color[1]
        msg.color.b = color[2]
        msg.color.a = 1.0
        return msg
        
    def visualize_text(self, text_arr):
        frame_id = 'velodyne'
        markers = [self.del_msg(frame_id)]
        self.counter += 1

        for idx, det in enumerate(text_arr):
            markers.append(self.dets_to_marker(det[0], det[1], det[2],det[3], frame_id, idx))
        msg = MarkerArray()
        msg.markers = markers
        self.publisher.publish(msg)

class IntMarkers:
    def __init__(self):
        self.server = InteractiveMarkerServer("supervisor_markers")

    def add_button(self, x, y, module):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'velodyne'
        int_marker.name = module.pkg
        int_marker.description = ''
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.orientation.w = 1

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.pose.orientation.w = 1
        box_marker.scale.x= 0.8
        box_marker.scale.y = 0.45
        box_marker.scale.z= 0.45
        box_marker.color.r = 0
        box_marker.color.g = 1
        box_marker.color.b = 0
        box_marker.color.a = 1

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode =InteractiveMarkerControl.BUTTON
        button_control.always_visible=True
        button_control.markers.append(box_marker)
        int_marker.controls.append(button_control)
        def handle_input(input):
            if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
                module.schedule_restart = 1
        self.server.insert(int_marker, handle_input)
        self.server.applyChanges()

def main():
    # Initialize node.
    rospy.init_node('supervisor')
    viz = Visualizer('/marker/supervisor')

    inspector = Inspector([
        #Module("aloam_velodyne", "aloam_velodyne_HDL_32.launch"),
        Module("velodyne_pointcloud", "VLP-32C_points.launch"),
        #Module("spinnaker_sdk_camera_driver", "main_cam.launch","/status/spinnaker"),
        #Module("darknet_ros", "fsoco_darknet.launch","/status/darknet_ros"),
        #Module("calibrationBroadcaster", "camera.launch"),
        #Module("darknet_ros", "fsoco_old.launch","/status/darknet_ros"),
        #Module("bounding_box_filter", "bounding_box_filter.launch","/status/bounding_box_filter"),
        #Module("marker_viz", "marker_viz.launch",'status/marker_viz'),
        #Module("mrpython_pcl", "lidar.launch",'/status/lidar'),
        #Module("smoreo", "smoreo_sys_flir.launch",'/status/smoreo'),
        #Module("smornn", "smornn.launch",'status/smornn'),
        #Module("graph_slam", "graph_slam.launch"),
        #Module("fs_planning-2022", "path_planner_python.launch","/status/planning"),
        #Module("navigation", "main.launch")
    ])

    # Initialize control commander.
    control_commander = ControlCommander()

    # Initialize mission observer.
    mission_observer = MissionObserver(
        launch_system = inspector.auto_launch,
        control_commmander = control_commander,
    )

    int_markers = IntMarkers()
    for idx, module in enumerate(inspector.modules):
        int_markers.add_button(7.5, -0.7*idx, module)
    
    def update_data():
        try:
            while True:
                data = inspector.get_data()
                to_viz = []
                for idx, row in enumerate(data):
                    to_viz.append([0,-0.7*idx,row[0],row[3]])
                    to_viz.append([2.5,-0.7*idx,row[1],row[3]])
                    to_viz.append([5,-0.7*idx,row[2],row[3]])
                viz.visualize_text(to_viz)
                time.sleep(1)
        except rospy.ROSInterruptException:
            pass

    threading.Thread(target=update_data).start()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        r.sleep()
        inspector.update()
        
    # Pool.
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
