#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <amr_rctk/TransformFrames.h>

bool transform_frames(amr_rctk::TransformFrames::Request &req,
                      amr_rctk::TransformFrames::Response &res)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    listener.waitForTransform(req.frame1, req.frame2, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(req.frame1, req.frame2, ros::Time(0), transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = req.frame1;
  tf::poseTFToMsg(transform, pose.pose);
  res.pose = pose;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_frames_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("transform_frames", transform_frames);
  ROS_INFO("Ready to transform frames.");

  ros::spin();

  return 0;
}