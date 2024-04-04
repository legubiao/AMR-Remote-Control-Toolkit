#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_pose_publisher");

  ros::NodeHandle node;

  ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>("/robot_pose", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_link",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = transform.getOrigin().x();
    pose_msg.pose.position.y = transform.getOrigin().y();
    pose_msg.pose.position.z = transform.getOrigin().z();
    pose_msg.pose.orientation.x = transform.getRotation().x();
    pose_msg.pose.orientation.y = transform.getRotation().y();
    pose_msg.pose.orientation.z = transform.getRotation().z();
    pose_msg.pose.orientation.w = transform.getRotation().w();

    pose_pub.publish(pose_msg);

    rate.sleep();
  }
  return 0;
};
