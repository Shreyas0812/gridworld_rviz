// include/create_gridworld/create_gridworld_node.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "create_gridworld/grid_world.hpp"
#include "create_gridworld/config_manager.hpp"
#include "create_gridworld/visualization_manager.hpp"

namespace create_gridworld {

class CreateGridWorldNode : public rclcpp::Node {
public:
    CreateGridWorldNode();

private:
    void publishGridworld();
    void agentPositionCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);
    void updateAgentPosition(int agent_index, int new_x, int new_y, int new_z);
    
    // Core components
    std::unique_ptr<ConfigManager> config_manager_;
    std::unique_ptr<GridWorld> grid_world_;
    std::unique_ptr<VisualizationManager> viz_manager_;
    
    // ROS2 components
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr agent_position_sub_;
};

} // namespace create_gridworld
