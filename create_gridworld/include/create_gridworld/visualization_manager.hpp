#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "create_gridworld/grid_world.hpp"
#include "create_gridworld/config_manager.hpp"
#include <memory>
#include <vector>
#include <cstdint>

namespace create_gridworld {

class VisualizationManager {
public:
    VisualizationManager(rclcpp::Node* node, const ConfigManager& config);
    
    void publishAll(const GridWorld& grid_world);
    void updateAgentPositions(const std::vector<std::vector<int64_t>>& agent_positions);

private:
    void createPublishers();
    void publishGridNodes(const GridWorld& grid_world);
    void publishGridEdges(const GridWorld& grid_world);
    void publishObstacles(const GridWorld& grid_world);
    void publishAgentMarkers();
    void publishInductStations();
    void publishEjectStations();
    
    rclcpp::Node* node_;
    const ConfigManager& config_;
    std::vector<std::vector<int64_t>> current_agent_positions_;
    
    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_nodes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_edges_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr agent_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr induct_stations_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr eject_stations_pub_;
};

} // namespace create_gridworld
