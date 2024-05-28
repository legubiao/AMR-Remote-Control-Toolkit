//
// Created by biao on 24-5-28.
//

#ifndef PATH_TRACKER_PID_H
#define PATH_TRACKER_PID_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "amr_rctk/pid.hpp"

namespace amr_rctk {
    class PathTrackerPID : public rclcpp::Node {
    public:
        PathTrackerPID();
        ~PathTrackerPID() override;
    private:
        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

        static double computeStanelyControl(const double heading_error, const double cross_track_error, const double velocity);
        geometry_msgs::msg::Twist computeControlOutputs(const nav_msgs::msg::Odometry& odom_robot, const geometry_msgs::msg::Pose& pose_current, const geometry_msgs::msg::Pose& pose_goal);

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_{};
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_local_path_{};
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_{};

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_{};

        nav_msgs::msg::Odometry odom_robot_{};
        geometry_msgs::msg::Pose current_pose_{};
        geometry_msgs::msg::Pose world_goal_pose_{};
        control::PID pid_;
    };
}
#endif //PATH_TRACKER_PID_H
