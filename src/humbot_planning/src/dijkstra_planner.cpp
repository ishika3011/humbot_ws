#include <queue>
#include <vector>

#include "humbot_planning/dijkstra_planner.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace humbot_planning
{

void DijkstraPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, 
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();


    smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_, "smooth_path");
    if(smooth_client_->wait_for_action_server(std::chrono::seconds(3)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting for 3s");
    }

}

void DijkstraPlanner::cleanup() 
{
    RCLCPP_INFO(node_->get_logger(), "Cleaning up Plugin %s of type DijkstraPlanner", name_.c_str());
}
void DijkstraPlanner::activate()
{
    RCLCPP_INFO(node_->get_logger(), "Activating up Plugin %s of type DijkstraPlanner", name_.c_str());
}

void DijkstraPlanner::deactivate() 
{
    RCLCPP_INFO(node_->get_logger(), "Deactivating up Plugin %s of type DijkstraPlanner", name_.c_str());
}

//implementing dijkstra algo
nav_msgs::msg::Path DijkstraPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal){
    //each cell has 4 neighbours
    std::vector<std::pair<int, int>> explore_directions = {
        {-1,0}, {1,0}, {0,-1}, {0,1}
    };
    //container storing graph nodes, for comparing graphnode objects
    std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
    std::vector<GraphNode> visited_nodes;

    pending_nodes.push(worldToGrid(start.pose));
    GraphNode active_node;

    while(!pending_nodes.empty() && rclcpp::ok())
    {
        //exploration logic
        active_node = pending_nodes.top();
        pending_nodes.pop();

        if(worldToGrid(goal.pose) == active_node){
            //found destination goal
            break;
        }
        //exploring neighbours of active node
        for(const auto & dir : explore_directions){
            GraphNode new_node = active_node + dir; // defined "+" in operator overload
            if(std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end() && //check if the new node is already visited or not
                poseOnMap(new_node) && costmap_->getCost(new_node.x, new_node.y) < 99 ) //on map? and if free/occupied cell>
            {
                new_node.cost = active_node.cost + 1 + costmap_->getCost(new_node.x, new_node.y); //all the cost are 1 for now
                new_node.prev = std::make_shared<GraphNode>(active_node);
                pending_nodes.push(new_node);
                visited_nodes.push_back(new_node);
            }
        }

    }

    //completed exploration
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    //traversing the vector back to the startign point
    while(active_node.prev && rclcpp::ok())
    {
        geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
        //inserting pose stamp msgs
        geometry_msgs::msg::PoseStamped last_pose_stamped;
        last_pose_stamped.header.frame_id = global_frame_;
        last_pose_stamped.pose = last_pose;
        path.poses.push_back(last_pose_stamped);
        active_node = *active_node.prev;
    }

    std::reverse(path.poses.begin(), path.poses.end());

    //for returning the smooth path
    if(smooth_client_->action_server_is_ready()){
    nav2_msgs::action::SmoothPath::Goal path_smooth;
    path_smooth.path = path;
    path_smooth.check_for_collisions = false;
    path_smooth.smoother_id = "simple_smoother";
    path_smooth.max_smoothing_duration.sec = 10;
    auto future = smooth_client_->async_send_goal(path_smooth);//sending goal msg

    //waiting till future is ready

    if(future.wait_for(std::chrono::seconds(3)) == std::future_status::ready){
      auto goal_handle = future.get();
      if(goal_handle){
        auto result_future = smooth_client_->async_get_result(goal_handle);
        if(result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready){
          auto result_path = result_future.get();
          if(result_path.code == rclcpp_action::ResultCode::SUCCEEDED){
            path = result_path.result->path;
          }
        }
      }
    }
  }

    return path;

}


GraphNode DijkstraPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
{
    int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX())/ costmap_->getResolution());
    int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginX())/ costmap_->getResolution());
    return GraphNode(grid_x, grid_y);
}

bool DijkstraPlanner::poseOnMap(const GraphNode & node)
{
    return node.x >= 0 && node.x < static_cast<int>(costmap_->getSizeInCellsX()) && 
    node.y >= 0 && node.y < static_cast<int>(costmap_->getSizeInCellsY());
}

unsigned int DijkstraPlanner::poseToCell(const GraphNode & node)// converts the new_node which has x, y coordinates as index of data of map
{
    return node.y * costmap_->getSizeInCellsX() + node.x;
}

geometry_msgs::msg::Pose DijkstraPlanner::gridToWorld(const GraphNode & node)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
    pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
    return pose;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(humbot_planning::DijkstraPlanner, nav2_core::GlobalPlanner)
