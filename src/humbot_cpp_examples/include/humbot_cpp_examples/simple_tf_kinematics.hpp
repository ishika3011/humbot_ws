#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <humbot_msgs/srv/get_transform.hpp>
#include <tf2/LinearMath/Quaternion.h>


#include<memory>

class SimpleTfKinematics : public rclcpp::Node
{

public: 
    SimpleTfKinematics(const std::string &name);



private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

    rclcpp::Service<humbot_msgs::srv::GetTransform>::SharedPtr get_transform_srv_;


    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double x_increment_;
    double last_x_;
    int rotations_counter_;
    tf2::Quaternion last_orientation_;
    tf2::Quaternion orientation_increment_;
    

    void timerCallback();

    bool getTransformCallback(
        const std::shared_ptr<humbot_msgs::srv::GetTransform::Request> req,
        const std::shared_ptr<humbot_msgs::srv::GetTransform::Response> res);
    
};

#endif