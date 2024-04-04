#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
import os
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

def publish_map_state(event=None):
    global map_state
    map_state_pub.publish(map_state)

def launch_amcl(map_name):
    global map_state
    global mapping_process

    if map_state == "mapping":
        rospy.logwarn("Terminate SLAM process before launching AMCL")
        mapping_process.terminate()

    amcl_command = "roslaunch " + amcl_launch + " map_file:=" + foldername+'/'+map_name + ".yaml"
    rospy.logwarn(amcl_command)
    process = subprocess.Popen(amcl_command.split())
    map_state = "navigation"
    publish_map_state()
    return process

def handle_get_map_files(req):
    map_files = [f[:-5] for f in os.listdir(foldername) if f.endswith('.yaml')]
    return TriggerResponse(True, ','.join(map_files))

def command_callback(msg):
    global mapping_process
    global amcl_process
    global map_state
    command = msg.data.split()

    if command[0] == "start":
        if map_state == "mapping":
            rospy.logwarn("Already in mapping status")
            return
        
        rospy.loginfo("Received command to start mapping")
        if map_state == "navigation":
            rospy.logwarn("Terminate AMCL process before mapping")
            amcl_process.terminate()

        # 启动SLAM建图
        mapping_process = subprocess.Popen(start_mapping_command.split())
        map_state = "mapping"
        publish_map_state()

    elif command[0] == "save":
        if map_state == "mapping":
            mapPath = foldername+'/'+command[1]
            saveMapCommand = save_map_command.split()
            saveMapCommand.append(mapPath)

            # 保存地图
            subprocess.call(saveMapCommand)
            rospy.loginfo("Map saved")

            # 启动AMCL节点
            amcl_process = launch_amcl(command[1])
        else:
            rospy.logwarn("Not mapping yet")

    elif command[0] == "load":
            # 启动AMCL节点
            rospy.loginfo("Load map file: " + command[1])
            amcl_process = launch_amcl(command[1])

if __name__ == "__main__":
    rospy.init_node("mapping_node")
    
    foldername = rospy.get_param("~foldername", "/path/to/your/folder")
    amcl_launch = rospy.get_param("~amcl_launch", "amr_platform amcl.launch")
    start_mapping_command = rospy.get_param("~start_mapping_command", "roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping")
    save_map_command = rospy.get_param("~save_map_command", "rosrun map_server map_saver -f")

    map_state = "idle"

    rospy.Subscriber("map_command", String, command_callback)
    rospy.Service('get_map_files', Trigger, handle_get_map_files)
    map_state_pub = rospy.Publisher("map_state", String, queue_size=10)
    rospy.Timer(rospy.Duration(1), publish_map_state)

    rospy.spin()
