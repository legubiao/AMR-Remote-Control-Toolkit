#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdomTransfer
{
public:
    OdomTransfer()
    {
        ros::NodeHandle private_nh("~");
        private_nh.param("frame1", frame1_, std::string("odom"));
        private_nh.param("frame2", frame2_, std::string("base_footprint"));
        sub_ = node_.subscribe("odom", 1000, &OdomTransfer::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame1_, frame2_));
    }

private:
    ros::NodeHandle node_;
    ros::Subscriber sub_;
    std::string frame1_, frame2_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_transfer");
    OdomTransfer ot;
    ros::spin();
    return 0;
}