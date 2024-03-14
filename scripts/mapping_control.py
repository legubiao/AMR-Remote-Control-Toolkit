#!/usr/bin/env python

import rospy
import subprocess
from std_msgs.msg import String

def start_mapping():
    subprocess.Popen(["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch", "slam_methods:=gmapping"])

def command_callback(msg):
    global mapping_process
    if msg.data == "start_mapping":
        # 启动SLAM建图
        mapping_process = subprocess.Popen(["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch", "slam_methods:=gmapping"])
        rospy.loginfo("Received command to start mapping")
    elif msg.data == "save_map":
        # 保存地图
        subprocess.call(["rosrun", "map_server", "map_saver", "-f", "/home/biao/maps/my_map"])
        rospy.loginfo("Map saved")
        # 停止SLAM建图节点
        mapping_process.terminate()
        rospy.loginfo("SLAM mapping process terminated")

if __name__ == "__main__":
    rospy.init_node("mapping_node")
    rospy.Subscriber("map_command", String, command_callback)
    rospy.spin()
