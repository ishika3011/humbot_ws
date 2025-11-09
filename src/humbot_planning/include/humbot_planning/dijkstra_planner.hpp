#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/smooth_path.hpp"

#include <memory>
#include <utility>   // for std::pair
#include <vector>

namespace humbot_planning
{

struct GraphNode
{
    int x;
    int y;
    int cost;
    std::shared_ptr<GraphNode> prev;

    GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0)
    {
    }

    GraphNode() : GraphNode(0, 0)
    {
    }

    bool operator>(const GraphNode & other) const
    {
        return cost > other.cost;
    }

    bool operator==(const GraphNode & other) const
    {
        return x == other.x && y == other.y;
    }

    GraphNode operator+(std::pair<int,int> const & other)
    {
        GraphNode res(x + other.first, y + other.second);
        return res;
    }

};

class DijkstraPlanner : public nav2_core::GlobalPlanner
{
public:
    DijkstraPlanner() = default;
    ~DijkstraPlanner() = default;

    //redefining some functions in base class(GlobaPlanner)
    //node, name of the plugin, tf buffer for calc current position of robot, global costmaps
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, 
        std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;

    void activate() override;

    void deactivate() override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;


private:

//created so that they have the value whatever we are directly rx from planner server
    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D * costmap_;
    std::string global_frame_, name_;

    rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smooth_client_;


    GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
    geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);
    bool poseOnMap(const GraphNode & node);
    unsigned int poseToCell(const GraphNode & node);

};

}

