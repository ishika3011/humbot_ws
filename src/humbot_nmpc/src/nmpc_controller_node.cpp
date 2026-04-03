#include "humbot_nmpc/nmpc_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

//     (your class, base class)
PLUGINLIB_EXPORT_CLASS(humbot_nmpc::NMPCController, nav2_core::Controller)

namespace humbot_nmpc
{

void NMPCController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    plugin_name_ = name;
    auto node = node_.lock(); // WeakPtr → SharedPtr (safe access)

    RCLCPP_INFO(logger_, "Configuring NMPC controller: %s", plugin_name_.c_str());

    //---------ACADOS capsule----------------------

    capsule_ = unicycle_acados_create_capsule();
    if(!capsule_){
        RCLCPP_ERROR(logger_,"failed to create capsule");
        return;
    }
    // #initialise solver
    int status = unicycle_acados_create(capsule_);
    if(status != 0){
        RCLCPP_ERROR(logger_, "acados solver creation failed with status: %d", status);
        return;
    }

    //pointers to call set/get on solver
    nlp_config_ = unicycle_acados_get_nlp_config(capsule_);
    nlp_dims_ = unicycle_acados_get_nlp_dims(capsule_);
    nlp_in_ = unicycle_acados_get_nlp_in(capsule_);
    nlp_out_ = unicycle_acados_get_nlp_out(capsule_);
    nlp_solver_ = unicycle_acados_get_nlp_solver(capsule_);
    nlp_opts_ = unicycle_acados_get_nlp_opts(capsule_);

    RCLCPP_INFO(logger_, "acados solver initialised. N=%d, Tf=%.1fs, dt=%.2fs",
                N_, Tf_, dt_);

}

void NMPCController::setPlan(const nav_msgs::msg::Path & path)
{
    global_plan_ = path;
    RCLCPP_INFO(logger_, "eceived new plan with %zu waypoints", 
        global_plan_.poses.size());
}

double NMPCController::getYaw(const geometry_msgs::msg::PoseStamped & pose)
{
    return tf2::getYaw(pose.pose.orientation);
}

void NMPCController::updateReference(const geometry_msgs::msg::PoseStamped & pose)
{
    //finds the next waypoint on the global path to use as x_ref for acados.

    if(global_plan_.poses.empty()){
        return;
    }
    double robot_x = pose.pose.position.x;
    double robot_y = pose.pose.position.y;

    double lookahead = 0.5; //metre
    //Instead of picking the closest point, 
    //you pick a point 0.5m ahead of the robot along the path

    size_t best_idx = global_plan_.poses.size() - 1;

    for(size_t i = 0; i < global_plan_.poses.size(); i++){
        double dx = global_plan_.poses[i].pose.position.x - robot_x;
        double dy = global_plan_.poses[i].pose.position.y - robot_y;
        double dist = std::sqrt(dx*dx + dy*dy);

        if(dist >= lookahead){
            best_idx = i;
            break;
        }
    }

    auto & ref_pose = global_plan_.poses[best_idx];
    x_ref_[0] = ref_pose.pose.position.x;
    x_ref_[1] = ref_pose.pose.position.y;
    x_ref_[2] = tf2::getYaw(ref_pose.pose.orientation);

}

geometry_msgs::msg::TwistStamped NMPCController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker *goal_checker)
    {
        //output msg 
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.frame_id = "base_link";
        cmd.header.stamp = pose.header.stamp;

        //extract x0 from odom
        double x0[3];
        x0[0] = pose.pose.position.x;
        x0[1] = pose.pose.position.y;
        x0[2] = getYaw(pose);

        //update ref from global path
        updateReference(pose);
        // x_ref_[0,1,2] has nxt waypt

        //pin initial state in acados -- solver.set(0,"lbx", x0_val)
        ocp_nlp_constraints_model_set(
            nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x0);
        
        ocp_nlp_constraints_model_set(
            nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x0);

        //set ref across horizon
        double y_ref[5] = {x_ref_[0], x_ref_[1], x_ref_[2], 0.0, 0.0};
        for(int k = 0; k < N_; k++){
            ocp_nlp_cost_model_set(
                nlp_config_, nlp_dims_, nlp_in_, k, "yref", y_ref);
        }

        //ref at Terminal
        ocp_nlp_cost_model_set(
            nlp_config_, nlp_dims_, nlp_in_, N_, "yref", x_ref_);

        int status = unicycle_acados_solve(capsule_);
        if(status != 0 && status != 2){
            RCLCPP_WARN(logger_, "acados solver warning: status=%d", status);
            return cmd;
        }

        //extract first optimal control u0
        double u0[2];
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u0);

        //pack into twist msg
        cmd.twist.linear.x = u0[0];
        cmd.twist.angular.z = u0[1];

        RCLCPP_DEBUG(logger_, "u0: v=%.3f m/s, omega=%.3f rad/s", u0[0], u0[1]);

    return cmd;
    }


    //Free all memory when Nav2 suts down
    void NMPCController::cleanup(){
        if(capsule_){
            unicycle_acados_free(capsule_);
            unicycle_acados_free_capsule(capsule_);
            capsule_ = nullptr;
            RCLCPP_INFO(logger_, "acados solver freed");
        }   
    }

    void NMPCController::activate(){
        RCLCPP_INFO(logger_, "NMPC controller activated");
    }

    void NMPCController::deactivate()
    {
        RCLCPP_INFO(logger_, "NMPC controller deactivated.");
    }
    
    void NMPCController::setSpeedLimit(
        const double & speed_limit,
        const bool & percentage)
    {
        (void)speed_limit; 
        (void)percentage;
        //(void) -- I know this variable exists, I am intentionally not using it, 
        //please suppress the warning." It does absolutely nothing at runtime 
        //it's purely for the compiler.
    }

}