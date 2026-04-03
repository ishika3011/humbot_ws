#ifndef HUMBOT_NMPC__NMPC_CONTROLLER_HPP_
#define HUMBOT_NMPC__NMPC_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"


//below included in braces will be treated as C
extern "C" {
#include "acados_solver_unicycle.h"
#include "acados/utils/math.h"
}

namespace humbot_nmpc
{
class NMPCController : public nav2_core::Controller
{
public:
    NMPCController() = default;
    ~NMPCController() override = default;

    //initializing acados solver
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    //called when Global path sent and 
    //want to extract waypoints to use as x_ref
    void setPlan(const nav_msgs::msg::Path & path) override;

    // Gets current pose + velocity, returns cmd_vel
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;
        
    void cleanup() override;    // free acados memory here
    void activate() override;   // solver ready to run
    void deactivate() override; // solver paused

    // Speed limit callback (required by Nav2 interface)
    void setSpeedLimit(
        const double & speed_limit,
        const bool & percentage) override;

    
private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    rclcpp::Logger logger_{rclcpp::get_logger("NMPCController")};
    std::string plugin_name_;

    //----------------------------ACADOS solver---------------
    // one capsule = one robot's solver instance

    unicycle_solver_capsule*capsule_{nullptr};

    ocp_nlp_config * nlp_config_{nullptr};
    ocp_nlp_dims    * nlp_dims_{nullptr};
    ocp_nlp_in      * nlp_in_{nullptr};
    ocp_nlp_out     * nlp_out_{nullptr};
    ocp_nlp_solver  * nlp_solver_{nullptr};
    void            * nlp_opts_{nullptr};

    // ------------------ Horizon parameters--------------

    static constexpr int    N_  = 30;         // static constexpr --> compile time constraints
    static constexpr double Tf_ = 3.0;        // which are chnged to binary at compile time
    static constexpr double dt_ = Tf_ / N_;
    
    //current ref, set by setPlan() and used by computeVelComm()
    double x_ref_[3]{0.0, 0.0, 0.0};

    //current global path

    nav_msgs::msg::Path global_plan_;

    //need to get yaw
    double getYaw(const geometry_msgs::msg::PoseStamped & pose);

    //finding closest waypt to x_ref
    void updateReference(const geometry_msgs::msg::PoseStamped & pose);

};

}
#endif 