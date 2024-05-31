#include <amr_rctk/path_publisher.hpp>

namespace amr_rctk {
    using std::placeholders::_1;

    int LOCAL_PREV_WP_NUM = 10;
    int LOCAL_NEXT_WP_NUM = 50;

    PathPublisher::PathPublisher(/* args */): Node("path_publisher") {
        RCLCPP_INFO(get_logger(), "Path publisher launching");
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { timerCallback(); });
        global_path_pub_ = create_publisher<nav_msgs::msg::Path>("global_path", 1);
        local_path_pub_ = create_publisher<nav_msgs::msg::Path>("local_path", 1);

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 1, std::bind(&PathPublisher::poseCallback, this, _1));
        target_path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "target_path", 10, std::bind(&PathPublisher::targetPathCallback, this, _1));

        global_path_msg_.header.frame_id = "map";
        local_path_msg_.header.frame_id = "map";

        current_id_ = 0;
        RCLCPP_INFO(get_logger(), "Path publisher initialized");
    }

    PathPublisher::~PathPublisher()
    = default;

    void PathPublisher::timerCallback() {
        publishGlobalPath();
        publishLocalPath(current_pose_, LOCAL_PREV_WP_NUM, LOCAL_NEXT_WP_NUM);
    }

    /**
     * @brief Callback function for subscribtion of the robot's pose
    */
    void PathPublisher::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = msg->pose;
    }

    /**
     * @brief Initialize the global path according to the input target path
    */
    void PathPublisher::targetPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received target path");
        std::vector<geometry_msgs::msg::PoseStamped> poses = msg->poses;

        tf2::Quaternion q;
        for (int i = 0; i < poses.size(); i++) {
            const double x_d = poses[i + 1].pose.position.x - poses[i].pose.position.x;
            const double y_d = poses[i + 1].pose.position.y - poses[i].pose.position.y;
            const double yaw = std::atan2(y_d, x_d);

            q.setRPY(0.0, 0.0, yaw);
            q.normalize();
            poses[i].pose.orientation = tf2::toMsg(q);
        }
        poses.back().pose.orientation = tf2::toMsg(q);
        this->global_path_msg_.poses = msg->poses;
    }

    void PathPublisher::publishGlobalPath() {
        global_path_msg_.header.stamp = now();
        global_path_pub_->publish(global_path_msg_);
    }

    void PathPublisher::publishLocalPath(const geometry_msgs::msg::Pose &robot_pose,
                                         const int n_wp_prev,
                                         const int n_wp_post) {
        if (global_path_msg_.poses.empty()) {
            RCLCPP_WARN(get_logger(), "Global Path not published yet, waiting");
            return;
        }
        if (const int id_next = nextWaypoint(robot_pose, global_path_msg_, current_id_);
            id_next >= global_path_msg_.poses.size() - 1) {
            RCLCPP_WARN(get_logger(), "Robot has reached the end of the track, please restart");
            current_id_ = 0;
        } else {
            current_id_ = std::max(current_id_, id_next - 1);
            const int id_start = std::max(id_next - n_wp_prev, 0);
            const int id_end = std::min(id_next + n_wp_post, static_cast<int>(global_path_msg_.poses.size() - 1));

            const auto start = global_path_msg_.poses.begin() + id_start;
            const auto end = global_path_msg_.poses.begin() + id_end;

            // Update the message
            local_path_msg_.header.stamp = this->now();
            local_path_msg_.poses = std::vector(start, end);
            local_path_pub_->publish(local_path_msg_);
        }
    }

    /**
     * @brief Find the closest waypoint to the robot in the path accoring to the robot's pose
    */
    int PathPublisher::closestWaypoint(const geometry_msgs::msg::Pose &robot_pose,
                                       const nav_msgs::msg::Path &path,
                                       const int id_start = 0) {
        auto min_dist = DBL_MAX;
        int id_closest = id_start;
        for (int i = id_start; i < path.poses.size(); i++) {
            const double dist = std::hypot(robot_pose.position.x - path.poses[i].pose.position.x,
                                           robot_pose.position.y - path.poses[i].pose.position.y);

            if (dist <= min_dist) {
                min_dist = dist;
                id_closest = i;
            } else {
                break;
            }
        }

        return id_closest;
    }

    int PathPublisher::nextWaypoint(const geometry_msgs::msg::Pose &robot_pose,
                                    const nav_msgs::msg::Path &path,
                                    const int id_start = 0) {
        int id_closest = closestWaypoint(robot_pose, path, id_start);
        double yaw_T_robot_wp = atan2((path.poses[id_closest].pose.position.y - robot_pose.position.y),
                                      (path.poses[id_closest].pose.position.x - robot_pose.position.x));

        const double yaw_robot = getYawFromOrientation(robot_pose.orientation);
        const double angle = std::fabs(yaw_robot - yaw_T_robot_wp);
        const double angle_norm = std::min(2 * M_PI - angle, angle);

        if (angle_norm > M_PI / 2) {
            id_closest++;
        }

        return id_closest;
    }

    double PathPublisher::getYawFromOrientation(const geometry_msgs::msg::Quaternion &orientation) {
        tf2::Quaternion q;
        fromMsg(orientation, q);
        const auto m = tf2::Matrix3x3(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        return yaw;
    }
}

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<amr_rctk::PathPublisher>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
