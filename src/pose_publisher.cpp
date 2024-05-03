#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("robot_pose_publisher");

    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);

    tf2_ros::Buffer tfBuffer(node->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);

    bool print_error = node->declare_parameter<bool>("print_error", false);

    rclcpp::Rate rate(10.0);
    while (rclcpp::ok()){
        geometry_msgs::msg::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex) {
            if (print_error){
                RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
            continue;
        }

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = node->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = transformStamped.transform.translation.x;
        pose_msg.pose.position.y = transformStamped.transform.translation.y;
        pose_msg.pose.position.z = transformStamped.transform.translation.z;
        pose_msg.pose.orientation = transformStamped.transform.rotation;

        pose_pub->publish(pose_msg);

        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
};