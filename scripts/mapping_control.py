#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
import os
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from amr_platform.srv import PoseList, PoseListResponse
from geometry_msgs.msg import PoseStamped
import yaml

def publish_map_state(event=None):
    global map_state
    map_state_pub.publish(map_state)

def launch_amcl(map_name):
    global map_state
    global mapping_process
    global map_id

    if map_state == "mapping":
        rospy.logwarn("Terminate SLAM process before launching AMCL")
        mapping_process.terminate()
    elif map_state == "navigation":
        rospy.logwarn("Terminate old AMCL process before launching AMCL")
        amcl_process.terminate()

    amcl_command = "roslaunch " + amcl_launch + " map_file:=" + foldername+'/'+map_name + ".yaml"
    rospy.logwarn(amcl_command)
    process = subprocess.Popen(amcl_command.split())
    map_state = "navigation"
    map_id = map_name
    publish_map_state()
    return process

def handle_get_map_files(req):
    map_files = [f[:-4] for f in os.listdir(foldername) if f.endswith('.pgm')]
    return TriggerResponse(True, ','.join(map_files))

def map_command_callback(msg):
    global mapping_process
    global amcl_process
    global map_state
    global map_id
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

def pose_to_dict(pose):
    return {
        'header': {
            'seq': pose.header.seq,
            'stamp': {'secs': pose.header.stamp.secs, 'nsecs': pose.header.stamp.nsecs},
            'frame_id': pose.header.frame_id,
        },
        'pose': {
            'position': {'x': pose.pose.position.x, 'y': pose.pose.position.y, 'z': pose.pose.position.z},
            'orientation': {'x': pose.pose.orientation.x, 'y': pose.pose.orientation.y, 'z': pose.pose.orientation.z, 'w': pose.pose.orientation.w},
        },
    }

def dict_to_pose(pose_dict):
    pose = PoseStamped()
    pose.header.seq = pose_dict['header']['seq']
    pose.header.stamp.secs = pose_dict['header']['stamp']['secs']
    pose.header.stamp.nsecs = pose_dict['header']['stamp']['nsecs']
    pose.header.frame_id = pose_dict['header']['frame_id']
    pose.pose.position.x = pose_dict['pose']['position']['x']
    pose.pose.position.y = pose_dict['pose']['position']['y']
    pose.pose.position.z = pose_dict['pose']['position']['z']
    pose.pose.orientation.x = pose_dict['pose']['orientation']['x']
    pose.pose.orientation.y = pose_dict['pose']['orientation']['y']
    pose.pose.orientation.z = pose_dict['pose']['orientation']['z']
    pose.pose.orientation.w = pose_dict['pose']['orientation']['w']
    return pose

def load_pose_by_map(map_id):
    global pose_list
    
    pose_file_path = foldername+'/'+ map_id + '_poses.yaml'
    try:
        with open(pose_file_path, 'r') as file:
            pose_list = [dict_to_pose(pose_dict) for pose_dict in yaml.load(file, Loader=yaml.FullLoader)]
        return PoseListResponse(True, "Poses loaded.", pose_list)
    except FileNotFoundError:
        with open(pose_file_path, 'w') as file:
            yaml.dump([], file)
        pose_list = []
        return PoseListResponse(True, "File not found. A new file has been created.", [])

def handle_pose_list(req):
    global pose_list

    if map_id == '':
        return PoseListResponse(False, "Not selected map yet.", [])

    if req.command == "add":
        pose_list.append(req.pose)
        return PoseListResponse(True, "Pose added.", pose_list)

    elif req.command == "delete":
        pose_list.remove(req.pose)
        return PoseListResponse(True, "Pose deleted.", pose_list)

    elif req.command == "get":
        return PoseListResponse(True, "Success.", pose_list)

    elif req.command == "save":
        pose_file_path = foldername+'/'+ map_id + '_poses.yaml'
        with open(pose_file_path, 'w') as file:
             yaml.dump([pose_to_dict(pose) for pose in pose_list], file)
        return PoseListResponse(True, "Poses saved.", pose_list)

    elif req.command == "load":
        return load_pose_by_map(map_id)

    else:
        return PoseListResponse(False, "Invalid command.", [])

if __name__ == "__main__":
    rospy.init_node("mapping_node")
    
    foldername = rospy.get_param("~foldername", "/path/to/your/folder")
    amcl_launch = rospy.get_param("~amcl_launch", "amr_platform amcl.launch")
    start_mapping_command = rospy.get_param("~start_mapping_command", "roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping")
    save_map_command = rospy.get_param("~save_map_command", "rosrun map_server map_saver -f")

    map_state = "idle"
    map_id = ''
    pose_list = []

    rospy.Subscriber("map_command", String, map_command_callback)
    rospy.Service('get_map_files', Trigger, handle_get_map_files)
    rospy.Service('pose_list', PoseList, handle_pose_list)

    map_state_pub = rospy.Publisher("map_state", String, queue_size=10)
    rospy.Timer(rospy.Duration(1), publish_map_state)

    rospy.spin()
