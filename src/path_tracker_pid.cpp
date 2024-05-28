//
// Created by biao on 24-5-28.
//

#include "amr_rctk/path_tracker_pid.hpp"

#include <amr_rctk/path_publisher.hpp>

#include "amr_rctk/math_utils.hpp"

namespace amr_rctk {
    double SPEED_TARGET = 0.15;
    double PID_Kp = 0.15, PID_Ki = 0.2, PID_Kd = 0.5;
    double STANLEY_K = 0.5;

    PathTrackerPID::PathTrackerPID() : Node("path_tracker_pid") {
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10, std::bind(&PathTrackerPID::poseCallback, this, std::placeholders::_1));
        sub_local_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "local_path", 10, std::bind(&PathTrackerPID::localPathCallback, this, std::placeholders::_1));
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
    }

    PathTrackerPID::~PathTrackerPID() = default;


    void PathTrackerPID::localPathCallback(const nav_msgs::msg::Path::SharedPtr path) {
        // Calculate absolute errors (wrt to world frame)
        world_goal_pose_ = path->poses[11].pose;
        pub_cmd_vel_->publish(computeControlOutputs(odom_robot_, current_pose_, world_goal_pose_));
    }

    void PathTrackerPID::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = msg->pose;
    }

    void PathTrackerPID::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_robot_ = *msg;
    }

    double PathTrackerPID::computeStanelyControl(const double heading_error,
                                                 const double cross_track_error,
                                                 const double velocity) {
        const double stanley_output = -1.0 * (heading_error + std::atan2(
                                                  STANLEY_K * cross_track_error, std::max(velocity, 0.3)));

        return std::min(std::max(stanley_output, -2.2), 2.2);
    }


    geometry_msgs::msg::Twist PathTrackerPID::computeControlOutputs(
        const nav_msgs::msg::Odometry &odom_robot,
        const geometry_msgs::msg::Pose &pose_current,
        const geometry_msgs::msg::Pose &pose_goal) {
        // Heading Error
        tf2::Quaternion q_robot, q_goal;
        tf2::fromMsg(pose_current.orientation, q_robot);
        tf2::fromMsg(pose_goal.orientation, q_goal);
        const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
        const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

        double roll, pitch, yaw_robot, yaw_goal;
        m_robot.getRPY(roll, pitch, yaw_robot);
        m_goal.getRPY(roll, pitch, yaw_goal);

        const double heading_error = unifyAngleRange(yaw_robot - yaw_goal);

        // Lateral Error
        tf2::Vector3 point_robot, point_goal;
        tf2::fromMsg(pose_current.position, point_robot);
        tf2::fromMsg(pose_goal.position, point_goal);
        const tf2::Vector3 V_goal_robot = point_robot - point_goal;
        const double angle_goal_robot = std::atan2(V_goal_robot.getY(), V_goal_robot.getX());
        const double angle_diff = angle_goal_robot - yaw_goal;
        const double lat_error = V_goal_robot.length() * std::sin(angle_diff);

        // Velocity
        tf2::Vector3 robot_vel;
        tf2::fromMsg(odom_robot.twist.twist.linear, robot_vel);
        const double velocity = robot_vel.length();

        geometry_msgs::msg::Twist cmd_vel;

        cmd_vel.linear.x = this->pid_.calculate(SPEED_TARGET, velocity);
        cmd_vel.angular.z = computeStanelyControl(heading_error, lat_error, velocity);

        return cmd_vel;
    }
}

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<amr_rctk::PathTrackerPID>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
