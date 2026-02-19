#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <string>
#include <chrono>
#include <functional>

using std::placeholders::_1;

class OdomPublisher : public rclcpp::Node
{
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double linear_scale_x_ = 1.0;
    double linear_scale_y_ = 1.0;
    double vel_dt_ = 0.0;
    double x_pos_ = 0.0;
    double y_pos_ = 0.0;
    double heading_ = 0.0;
    double linear_velocity_x_ = 0.0;
    double linear_velocity_y_ = 0.0;
    double angular_velocity_z_ = 0.0;
    bool pub_odom_tf_ = false;
    rclcpp::Time last_vel_time_;

public:
    OdomPublisher() : Node("base_node")
    {
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
        this->declare_parameter<double>("linear_scale_x", 1.0);
        this->declare_parameter<double>("linear_scale_y", 1.0);
        this->declare_parameter<bool>("pub_odom_tf", true);

        this->get_parameter("linear_scale_x", linear_scale_x_);
        this->get_parameter("linear_scale_y", linear_scale_y_);
        this->get_parameter("pub_odom_tf", pub_odom_tf_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "vel_raw", 50, std::bind(&OdomPublisher::handle_vel, this, _1));
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
        
        last_vel_time_ = this->get_clock()->now();
    }

private:
    void handle_vel(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
    {
        rclcpp::Time curren_time = this->get_clock()->now();
        vel_dt_ = (curren_time - last_vel_time_).seconds();
        last_vel_time_ = curren_time;

        if (vel_dt_ > 0.5) vel_dt_ = 0.0; // Sécurité premier message

        linear_velocity_x_ = msg->linear.x * linear_scale_x_;
        linear_velocity_y_ = msg->linear.y * linear_scale_y_;
        angular_velocity_z_ = msg->angular.z;

        double delta_heading = angular_velocity_z_ * vel_dt_;
        double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_;
        double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_;
        
        x_pos_ += delta_x;
        y_pos_ += delta_y;
        heading_ += delta_heading;

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, heading_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = curren_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x_pos_;
        odom.pose.pose.position.y = y_pos_;
        odom.pose.pose.orientation.x = myQuaternion.x();
        odom.pose.pose.orientation.y = myQuaternion.y();
        odom.pose.pose.orientation.z = myQuaternion.z();
        odom.pose.pose.orientation.w = myQuaternion.w();

        odom.twist.twist.linear.x = linear_velocity_x_;
        odom.twist.twist.linear.y = linear_velocity_y_;
        odom.twist.twist.angular.z = angular_velocity_z_;

        odom_publisher_->publish(odom);

        if (pub_odom_tf_)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = curren_time;
            t.header.frame_id = "odom";
            t.child_frame_id = "base_footprint";
            t.transform.translation.x = x_pos_;
            t.transform.translation.y = y_pos_;
            t.transform.rotation.x = myQuaternion.x();
            t.transform.rotation.y = myQuaternion.y();
            t.transform.rotation.z = myQuaternion.z();
            t.transform.rotation.w = myQuaternion.w();
            tf_broadcaster_->sendTransform(t);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}


