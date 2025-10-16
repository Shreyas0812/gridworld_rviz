#include "create_gridworld/create_gridworld_node.hpp"
#include <chrono>

namespace create_gridworld {

CreateGridWorldNode::CreateGridWorldNode() : Node("create_gridworld_node")
{
    // Initialize components in order
    config_manager_ = std::make_unique<ConfigManager>(this);
    
    grid_world_ = std::make_unique<GridWorld>(
        config_manager_->getGridWidth(),
        config_manager_->getGridHeight(), 
        config_manager_->getGridDepth(),
        config_manager_->getGridResolution()
    );
    
    viz_manager_ = std::make_unique<VisualizationManager>(this, *config_manager_);
    
    // Add obstacles to grid
    grid_world_->addObstacleRegions(config_manager_->getObstacleRegions());
    
    // Create subscriber for agent positions
    agent_position_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
        "agent_position", 
        10,
        std::bind(&CreateGridWorldNode::agentPositionCallback, this, std::placeholders::_1)
    );

    // Timer to publish the grid at regular intervals
    int timer_period_ms = static_cast<int>(1000.0 / config_manager_->getPublishRate());
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&CreateGridWorldNode::publishGridworld, this)
    );

    RCLCPP_INFO(this->get_logger(), "CreateGridWorldNode started with %dx%dx%d grid, resolution %.2f", 
                config_manager_->getGridWidth(), config_manager_->getGridHeight(), 
                config_manager_->getGridDepth(), config_manager_->getGridResolution());
}

void CreateGridWorldNode::publishGridworld()
{
    viz_manager_->publishAll(*grid_world_);
}

void CreateGridWorldNode::agentPositionCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 4)
    {
        RCLCPP_WARN(this->get_logger(), "Received invalid agent positions array size: %zu, expected: 4 [agent_index, x, y, z])", msg->data.size());
        return;
    }

    int agent_index = msg->data[0];
    int64_t x = msg->data[1];
    int64_t y = msg->data[2];
    int64_t z = msg->data[3];

    if (x < 0 || x >= config_manager_->getGridWidth() || 
        y < 0 || y >= config_manager_->getGridHeight() || 
        z < 0 || z >= config_manager_->getGridDepth())
    {
        RCLCPP_WARN(this->get_logger(), "Received out-of-bounds agent position: [%ld, %ld, %ld]", x, y, z);
        return;
    }

    updateAgentPosition(agent_index, x, y, z);
    RCLCPP_DEBUG(this->get_logger(), "Updated agent %d to position (%ld, %ld, %ld)", 
                agent_index, x, y, z);
}

void CreateGridWorldNode::updateAgentPosition(int agent_index, int new_x, int new_y, int new_z)
{
    // Get current agent positions from config
    auto agent_positions = config_manager_->getAgentPositions();
    
    // Ensure the agent index is valid
    if (agent_index < 0) {
        RCLCPP_WARN(this->get_logger(), "Invalid agent index %d", agent_index);
        return;
    }
    
    // Expand agent positions if needed
    while (static_cast<int>(agent_positions.size()) <= agent_index) {
        agent_positions.emplace_back(std::vector<int64_t>{0, 0, 0});
        RCLCPP_INFO(this->get_logger(), "Initialized new agent %d", static_cast<int>(agent_positions.size() - 1));
    }
    
    // Update position
    agent_positions[agent_index][0] = new_x;
    agent_positions[agent_index][1] = new_y;
    agent_positions[agent_index][2] = new_z;
    
    // Update the visualization manager
    viz_manager_->updateAgentPositions(agent_positions);
}

} // namespace create_gridworld


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<create_gridworld::CreateGridWorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}