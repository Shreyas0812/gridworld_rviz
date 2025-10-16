#include "create_gridworld/config_manager.hpp"
#include <sstream>

namespace create_gridworld {

ConfigManager::ConfigManager(rclcpp::Node* node)
{
    loadParameters(node);
    loadVisualizationConfig(node);
    loadTopicConfig(node);
    validateParameters();
    
    // Log all loaded parameters (moved from original constructor)
    RCLCPP_INFO(node->get_logger(), "Grid width: %d", grid_width_);
    RCLCPP_INFO(node->get_logger(), "Grid height: %d", grid_height_);
    RCLCPP_INFO(node->get_logger(), "Grid depth: %d", grid_depth_);
    RCLCPP_INFO(node->get_logger(), "Grid resolution: %.2f", grid_resolution_);
    RCLCPP_INFO(node->get_logger(), "Publish rate: %.2f Hz", publish_rate_);
    RCLCPP_INFO(node->get_logger(), "Frame ID: %s", frame_id_.c_str());

    RCLCPP_INFO(node->get_logger(), "Loaded %zu obstacle regions, %zu agent start positions, %zu induct stations, %zu eject stations", 
        obstacle_regions_.size(), agent_positions_.size(), induct_stations_.size(), eject_stations_.size());

    // Log obstacle regions
    RCLCPP_INFO(node->get_logger(), "Obstacle regions:");
    for (const auto & region : obstacle_regions_) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < region.size(); ++i) {
            oss << region[i];
            if (i + 1 < region.size()) oss << ", ";
        }
        oss << "]";
        RCLCPP_INFO(node->get_logger(), "%s", oss.str().c_str());
    }

    // Log agent positions
    RCLCPP_INFO(node->get_logger(), "Agent start positions:");
    for (const auto & pos : agent_positions_) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < pos.size(); ++i) {
            oss << pos[i];
            if (i + 1 < pos.size()) oss << ", ";
        }
        oss << "]";
        RCLCPP_INFO(node->get_logger(), "%s", oss.str().c_str());
    }

    // Log stations
    RCLCPP_INFO(node->get_logger(), "Induct stations:");
    for (const auto & station : induct_stations_) {
        RCLCPP_INFO(node->get_logger(), "  [%ld, %ld, %ld] ID: %ld", 
            station[0], station[1], station[2], station[3]);
    }

    RCLCPP_INFO(node->get_logger(), "Eject stations:");
    for (const auto & station : eject_stations_) {
        RCLCPP_INFO(node->get_logger(), "  [%ld, %ld, %ld] ID: %ld", 
            station[0], station[1], station[2], station[3]);
    }

    // Log visualization config
    RCLCPP_INFO(node->get_logger(), "Visualization config:");
    RCLCPP_INFO(node->get_logger(), "  node_scale: %.2f", viz_config_.node_scale);
    RCLCPP_INFO(node->get_logger(), "  node_color: [%.2f, %.2f, %.2f, %.2f]", 
        viz_config_.node_color[0], viz_config_.node_color[1], viz_config_.node_color[2], viz_config_.node_color[3]);
    RCLCPP_INFO(node->get_logger(), "  edge_width: %.2f", viz_config_.edge_width);
    RCLCPP_INFO(node->get_logger(), "  edge_color: [%.2f, %.2f, %.2f, %.2f]", 
        viz_config_.edge_color[0], viz_config_.edge_color[1], viz_config_.edge_color[2], viz_config_.edge_color[3]);
    RCLCPP_INFO(node->get_logger(), "  obstacle_scale: %.2f", viz_config_.obstacle_scale);
    RCLCPP_INFO(node->get_logger(), "  obstacle_color: [%.2f, %.2f, %.2f, %.2f]", 
        viz_config_.obstacle_color[0], viz_config_.obstacle_color[1], viz_config_.obstacle_color[2], viz_config_.obstacle_color[3]);
    RCLCPP_INFO(node->get_logger(), "  agent_scale: %.2f", viz_config_.agent_scale);
    RCLCPP_INFO(node->get_logger(), "  agent_color: [%.2f, %.2f, %.2f, %.2f]", 
        viz_config_.agent_color[0], viz_config_.agent_color[1], viz_config_.agent_color[2], viz_config_.agent_color[3]);
    RCLCPP_INFO(node->get_logger(), "  induct_station_scale: %.2f", viz_config_.induct_station_scale);
    RCLCPP_INFO(node->get_logger(), "  induct_station_color: [%.2f, %.2f, %.2f, %.2f]", 
        viz_config_.induct_station_color[0], viz_config_.induct_station_color[1], viz_config_.induct_station_color[2], viz_config_.induct_station_color[3]);
    RCLCPP_INFO(node->get_logger(), "  eject_station_scale: %.2f", viz_config_.eject_station_scale);
    RCLCPP_INFO(node->get_logger(), "  eject_station_color: [%.2f, %.2f, %.2f, %.2f]", 
        viz_config_.eject_station_color[0], viz_config_.eject_station_color[1], viz_config_.eject_station_color[2], viz_config_.eject_station_color[3]);

    // Log topic names
    RCLCPP_INFO(node->get_logger(), "Topic names:");
    RCLCPP_INFO(node->get_logger(), "  nodes: %s", topic_config_.nodes.c_str());
    RCLCPP_INFO(node->get_logger(), "  edges: %s", topic_config_.edges.c_str());
    RCLCPP_INFO(node->get_logger(), "  obstacles: %s", topic_config_.obstacles.c_str());
    RCLCPP_INFO(node->get_logger(), "  agents: %s", topic_config_.agents.c_str());
    RCLCPP_INFO(node->get_logger(), "  induct_stations: %s", topic_config_.induct_stations.c_str());
    RCLCPP_INFO(node->get_logger(), "  eject_stations: %s", topic_config_.eject_stations.c_str());
}

void ConfigManager::loadParameters(rclcpp::Node* node)
{
    // Declare and get basic grid parameters
    node->declare_parameter("grid_width", 50);
    node->declare_parameter("grid_height", 50);
    node->declare_parameter("grid_depth", 10);
    node->declare_parameter("grid_resolution", 1.0);
    node->declare_parameter("publish_rate", 10.0);
    node->declare_parameter("frame_id", std::string("map"));

    grid_width_ = node->get_parameter("grid_width").as_int();
    grid_height_ = node->get_parameter("grid_height").as_int();
    grid_depth_ = node->get_parameter("grid_depth").as_int();
    grid_resolution_ = node->get_parameter("grid_resolution").as_double();
    publish_rate_ = node->get_parameter("publish_rate").as_double();
    frame_id_ = node->get_parameter("frame_id").as_string();

    // Declare and load entity parameters
    node->declare_parameter("obstacle_regions", std::vector<int64_t>{});
    node->declare_parameter("agent_positions", std::vector<int64_t>{});
    node->declare_parameter("induct_stations", std::vector<int64_t>{});
    node->declare_parameter("eject_stations", std::vector<int64_t>{});

    // Process obstacle regions
    auto flat_obstacles = node->get_parameter("obstacle_regions").as_integer_array();
    obstacle_regions_.clear();
    for (size_t i = 0; i < flat_obstacles.size(); i += 6) {
        obstacle_regions_.emplace_back(std::vector<int64_t>{
            flat_obstacles[i], flat_obstacles[i + 1], flat_obstacles[i + 2],
            flat_obstacles[i + 3], flat_obstacles[i + 4], flat_obstacles[i + 5]
        });
    }

    // Process agent positions
    auto flat_agents = node->get_parameter("agent_positions").as_integer_array();
    agent_positions_.clear();
    for (size_t i = 0; i < flat_agents.size(); i += 3) {
        agent_positions_.emplace_back(std::vector<int64_t>{
            flat_agents[i], flat_agents[i + 1], flat_agents[i + 2]
        });
    }

    // Process induct stations
    auto flat_induct = node->get_parameter("induct_stations").as_integer_array();
    induct_stations_.clear();
    for (size_t i = 0; i < flat_induct.size(); i += 4) {
        induct_stations_.emplace_back(std::vector<int64_t>{
            flat_induct[i], flat_induct[i + 1], flat_induct[i + 2], flat_induct[i + 3]
        });
    }

    // Process eject stations
    auto flat_eject = node->get_parameter("eject_stations").as_integer_array();
    eject_stations_.clear();
    for (size_t i = 0; i < flat_eject.size(); i += 4) {
        eject_stations_.emplace_back(std::vector<int64_t>{
            flat_eject[i], flat_eject[i + 1], flat_eject[i + 2], flat_eject[i + 3]
        });
    }
}

void ConfigManager::loadVisualizationConfig(rclcpp::Node* node)
{
    // Declare visualization parameters
    node->declare_parameter("visualization.node_scale", 0.1);
    node->declare_parameter("visualization.node_color", std::vector<double>{0.0, 1.0, 0.0, 0.5});
    node->declare_parameter("visualization.edge_width", 0.05);
    node->declare_parameter("visualization.edge_color", std::vector<double>{0.0, 0.0, 1.0, 0.5});
    node->declare_parameter("visualization.obstacle_scale", 0.9);
    node->declare_parameter("visualization.obstacle_color", std::vector<double>{1.0, 0.0, 0.0, 0.8});
    node->declare_parameter("visualization.agent_scale", 0.8);
    node->declare_parameter("visualization.agent_color", std::vector<double>{0.0, 0.0, 1.0, 1.0});
    node->declare_parameter("visualization.induct_station_scale", 1.2);
    node->declare_parameter("visualization.induct_station_color", std::vector<double>{0.0, 0.5, 1.0, 1.0});
    node->declare_parameter("visualization.eject_station_scale", 1.2);
    node->declare_parameter("visualization.eject_station_color", std::vector<double>{1.0, 0.5, 0.0, 1.0});

    // Get visualization parameters
    viz_config_.node_scale = node->get_parameter("visualization.node_scale").as_double();
    viz_config_.node_color = node->get_parameter("visualization.node_color").as_double_array();
    viz_config_.edge_width = node->get_parameter("visualization.edge_width").as_double();
    viz_config_.edge_color = node->get_parameter("visualization.edge_color").as_double_array();
    viz_config_.obstacle_scale = node->get_parameter("visualization.obstacle_scale").as_double();
    viz_config_.obstacle_color = node->get_parameter("visualization.obstacle_color").as_double_array();
    viz_config_.agent_scale = node->get_parameter("visualization.agent_scale").as_double();
    viz_config_.agent_color = node->get_parameter("visualization.agent_color").as_double_array();
    viz_config_.induct_station_scale = node->get_parameter("visualization.induct_station_scale").as_double();
    viz_config_.induct_station_color = node->get_parameter("visualization.induct_station_color").as_double_array();
    viz_config_.eject_station_scale = node->get_parameter("visualization.eject_station_scale").as_double();
    viz_config_.eject_station_color = node->get_parameter("visualization.eject_station_color").as_double_array();
}

void ConfigManager::loadTopicConfig(rclcpp::Node* node)
{
    // Declare topic parameters
    node->declare_parameter("topics.nodes", std::string("gridworld/nodes"));
    node->declare_parameter("topics.edges", std::string("gridworld/edges"));
    node->declare_parameter("topics.obstacles", std::string("gridworld/obstacles"));
    node->declare_parameter("topics.agents", std::string("gridworld/agents"));
    node->declare_parameter("topics.induct_stations", std::string("gridworld/induct_stations"));
    node->declare_parameter("topics.eject_stations", std::string("gridworld/eject_stations"));

    // Get topic names
    topic_config_.nodes = node->get_parameter("topics.nodes").as_string();
    topic_config_.edges = node->get_parameter("topics.edges").as_string();
    topic_config_.obstacles = node->get_parameter("topics.obstacles").as_string();
    topic_config_.agents = node->get_parameter("topics.agents").as_string();
    topic_config_.induct_stations = node->get_parameter("topics.induct_stations").as_string();
    topic_config_.eject_stations = node->get_parameter("topics.eject_stations").as_string();
}

void ConfigManager::validateParameters()
{
    // Add any parameter validation logic here if needed
    if (grid_width_ <= 0 || grid_height_ <= 0 || grid_depth_ <= 0) {
        throw std::invalid_argument("Grid dimensions must be positive");
    }
    if (grid_resolution_ <= 0.0) {
        throw std::invalid_argument("Grid resolution must be positive");
    }
    if (publish_rate_ <= 0.0) {
        throw std::invalid_argument("Publish rate must be positive");
    }
}

} // namespace create_gridworld
