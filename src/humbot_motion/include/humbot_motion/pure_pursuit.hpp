#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "nav2_core/controller.hpp"

namespace humbot_motion
{
class PurePursuit : public nav2_core::Controller
{
public:
    PurePursuit() = default;
    ~PurePursuit() = default;

    void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pose_pub_;

    double look_ahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    nav_msgs::msg::Path global_plan_;

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::Logger logger_ {rclcpp::get_logger("PurePursuit")};
    rclcpp::Clock::SharedPtr clock_;

    bool transformPlan(const std::string & frame);

    geometry_msgs::msg::PoseStamped getCarrotPose(
        const geometry_msgs::msg::PoseStamped & robot_pose);
    
    double getCurvature(const geometry_msgs::msg::Pose & carrot_pose);
};
}  // namespace humbot_motion

#endif // PURE_PURSUIT_HPP