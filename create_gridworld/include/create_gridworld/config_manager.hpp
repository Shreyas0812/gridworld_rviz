// include/create_gridworld/config_manager.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <cstdint>

namespace create_gridworld {

struct VisualizationConfig {
    double node_scale;
    std::vector<double> node_color;
    double edge_width;
    std::vector<double> edge_color;
    double obstacle_scale;
    std::vector<double> obstacle_color;
    double agent_scale;
    std::vector<double> agent_color;
    double induct_station_scale;
    std::vector<double> induct_station_color;
    double eject_station_scale;
    std::vector<double> eject_station_color;
};

struct TopicConfig {
    std::string nodes;
    std::string edges;
    std::string obstacles;
    std::string agents;
    std::string induct_stations;
    std::string eject_stations;
};

class ConfigManager {
public:
    explicit ConfigManager(rclcpp::Node* node);
    
    // Grid parameters
    int getGridWidth() const { return grid_width_; }
    int getGridHeight() const { return grid_height_; }
    int getGridDepth() const { return grid_depth_; }
    double getGridResolution() const { return grid_resolution_; }
    double getPublishRate() const { return publish_rate_; }
    const std::string& getFrameId() const { return frame_id_; }
    
    // Entity data
    const std::vector<std::vector<int64_t>>& getObstacleRegions() const { return obstacle_regions_; }
    const std::vector<std::vector<int64_t>>& getAgentPositions() const { return agent_positions_; }
    const std::vector<std::vector<int64_t>>& getInductStations() const { return induct_stations_; }
    const std::vector<std::vector<int64_t>>& getEjectStations() const { return eject_stations_; }
    
    // Configuration structs
    const VisualizationConfig& getVisualizationConfig() const { return viz_config_; }
    const TopicConfig& getTopicConfig() const { return topic_config_; }

private:
    void loadParameters(rclcpp::Node* node);
    void loadVisualizationConfig(rclcpp::Node* node);
    void loadTopicConfig(rclcpp::Node* node);
    void validateParameters();
    
    // Grid parameters
    int grid_width_, grid_height_, grid_depth_;
    double grid_resolution_, publish_rate_;
    std::string frame_id_;
    
    // Entity data
    std::vector<std::vector<int64_t>> obstacle_regions_;
    std::vector<std::vector<int64_t>> agent_positions_;
    std::vector<std::vector<int64_t>> induct_stations_;
    std::vector<std::vector<int64_t>> eject_stations_;
    
    // Configuration structs
    VisualizationConfig viz_config_;
    TopicConfig topic_config_;
};

} // namespace create_gridworld
