#ifndef HEADER_FILE_H
#define HEADER_FILE_H

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace amr_rctk {
    class PathPublisher : public rclcpp::Node {
        void timerCallback();

        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void targetPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

        void publishLocalPath(const geometry_msgs::msg::Pose &robot_pose, const int n_wp_prev, const int n_wp_post);

        void publishGlobalPath();

        static int closestWaypoint(const geometry_msgs::msg::Pose &robot_pose, const nav_msgs::msg::Path &path,
                                   const int id_start);

        static int nextWaypoint(const geometry_msgs::msg::Pose &robot_pose, const nav_msgs::msg::Path &path,
                                const int id_start);

        static double getYawFromOrientation(const geometry_msgs::msg::Quaternion &orientation);

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_{};
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_{};

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_{};
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_path_sub_{};

        rclcpp::TimerBase::SharedPtr timer_;

        nav_msgs::msg::Path global_path_msg_{};
        nav_msgs::msg::Path local_path_msg_{};
        geometry_msgs::msg::Pose current_pose_{};

        std::string world_frame_;
        std::string robot_frame_;

        int current_id_;

    public:
        PathPublisher(/* args */);

        ~PathPublisher() override;
    };
}

#endif // HEADER_FILE_H
