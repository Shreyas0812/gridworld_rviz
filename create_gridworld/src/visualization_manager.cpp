#include "create_gridworld/visualization_manager.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace create_gridworld {

VisualizationManager::VisualizationManager(rclcpp::Node* node, const ConfigManager& config)
    : node_(node), config_(config)
{
    current_agent_positions_ = config_.getAgentPositions();
    createPublishers();
}

void VisualizationManager::createPublishers()
{
    const auto& topics = config_.getTopicConfig();
    
    grid_nodes_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topics.nodes, 10);
    grid_edges_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topics.edges, 10);
    obstacles_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topics.obstacles, 10);
    agent_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topics.agents, 10);
    induct_stations_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topics.induct_stations, 10);
    eject_stations_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topics.eject_stations, 10);
}

void VisualizationManager::publishAll(const GridWorld& grid_world)
{
    publishGridNodes(grid_world);
    publishGridEdges(grid_world);
    publishObstacles(grid_world);
    publishAgentMarkers();
    publishInductStations();
    publishEjectStations();
}

void VisualizationManager::updateAgentPositions(const std::vector<std::vector<int64_t>>& agent_positions)
{
    current_agent_positions_ = agent_positions;
}

void VisualizationManager::publishGridNodes(const GridWorld& grid_world)
{
    auto timestamp = node_->now();
    const auto& viz_config = config_.getVisualizationConfig();
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = config_.getFrameId();
    marker.header.stamp = timestamp;
    marker.ns = "grid_nodes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = grid_world.getResolution() * viz_config.node_scale;
    marker.scale.y = grid_world.getResolution() * viz_config.node_scale;
    marker.scale.z = grid_world.getResolution() * viz_config.node_scale;

    marker.color.r = viz_config.node_color[0];
    marker.color.g = viz_config.node_color[1];
    marker.color.b = viz_config.node_color[2];
    marker.color.a = viz_config.node_color[3];

    for (const auto & node : grid_world.getNodes())
    {
        if (!node.is_obstacle)
        {
            geometry_msgs::msg::Point p;
            p.x = node.x * grid_world.getResolution();
            p.y = node.y * grid_world.getResolution();
            p.z = node.z * grid_world.getResolution();
            marker.points.push_back(p);
        }
    }
    
    marker_array.markers.push_back(marker);
    grid_nodes_pub_->publish(marker_array);
}

void VisualizationManager::publishGridEdges(const GridWorld& grid_world)
{
    auto timestamp = node_->now();
    const auto& viz_config = config_.getVisualizationConfig();
    
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker lines_marker;
    lines_marker.header.frame_id = config_.getFrameId();
    lines_marker.header.stamp = timestamp;
    lines_marker.ns = "grid_edges";
    lines_marker.id = 0;
    lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines_marker.action = visualization_msgs::msg::Marker::ADD;
    lines_marker.pose.orientation.w = 1.0;

    lines_marker.scale.x = viz_config.edge_width;
    lines_marker.color.r = viz_config.edge_color[0];
    lines_marker.color.g = viz_config.edge_color[1];
    lines_marker.color.b = viz_config.edge_color[2];
    lines_marker.color.a = viz_config.edge_color[3];

    const auto& nodes = grid_world.getNodes();
    for (size_t i = 0; i < nodes.size(); i++)
    {
        const auto &node = nodes[i];
        if (node.is_obstacle) continue;

        for (int neighbor_index : node.neighbors)
        {
            if (static_cast<int>(i) >= neighbor_index) continue; // Avoid duplicate edges

            geometry_msgs::msg::Point pstart, pend;
            pstart.x = node.x * grid_world.getResolution();
            pstart.y = node.y * grid_world.getResolution();
            pstart.z = node.z * grid_world.getResolution();

            pend.x = nodes[neighbor_index].x * grid_world.getResolution();
            pend.y = nodes[neighbor_index].y * grid_world.getResolution();
            pend.z = nodes[neighbor_index].z * grid_world.getResolution();
            
            lines_marker.points.push_back(pstart);
            lines_marker.points.push_back(pend);
        }
    }

    marker_array.markers.push_back(lines_marker);
    grid_edges_pub_->publish(marker_array);
}

void VisualizationManager::publishObstacles(const GridWorld& grid_world)
{
    auto timestamp = node_->now();
    const auto& viz_config = config_.getVisualizationConfig();
    
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker cubes_marker;

    cubes_marker.header.frame_id = config_.getFrameId();
    cubes_marker.header.stamp = timestamp;
    cubes_marker.ns = "obstacles";
    cubes_marker.id = 0;
    cubes_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cubes_marker.action = visualization_msgs::msg::Marker::ADD;
    cubes_marker.pose.orientation.w = 1.0;

    cubes_marker.scale.x = grid_world.getResolution() * viz_config.obstacle_scale;
    cubes_marker.scale.y = grid_world.getResolution() * viz_config.obstacle_scale;
    cubes_marker.scale.z = grid_world.getResolution() * viz_config.obstacle_scale;

    cubes_marker.color.r = viz_config.obstacle_color[0];
    cubes_marker.color.g = viz_config.obstacle_color[1];
    cubes_marker.color.b = viz_config.obstacle_color[2];
    cubes_marker.color.a = viz_config.obstacle_color[3];

    for (const auto & node : grid_world.getNodes())
    {
        if (node.is_obstacle)
        {
            geometry_msgs::msg::Point p;
            p.x = node.x * grid_world.getResolution();
            p.y = node.y * grid_world.getResolution();
            p.z = node.z * grid_world.getResolution();
            cubes_marker.points.push_back(p);
        }
    }

    marker_array.markers.push_back(cubes_marker);
    obstacles_pub_->publish(marker_array);
}

void VisualizationManager::publishAgentMarkers()
{
    auto timestamp = node_->now();
    const auto& viz_config = config_.getVisualizationConfig();
    
    visualization_msgs::msg::MarkerArray marker_array;

    std::vector<std::tuple<int, int, int>> agent_pos;
    for (const auto & pos : current_agent_positions_)
    {
        if (pos.size() < 3) continue;
        agent_pos.emplace_back(pos[0], pos[1], pos[2]);
    }
    
    int agent_id = 0;
    for (const auto & [x, y, z] : agent_pos)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = config_.getFrameId();
        marker.header.stamp = timestamp;
        marker.ns = "agents";
        marker.id = agent_id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x * config_.getGridResolution();
        marker.pose.position.y = y * config_.getGridResolution();
        marker.pose.position.z = z * config_.getGridResolution();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = config_.getGridResolution() * viz_config.agent_scale;
        marker.scale.y = config_.getGridResolution() * viz_config.agent_scale;
        marker.scale.z = config_.getGridResolution() * viz_config.agent_scale;
        marker.color.r = viz_config.agent_color[0];
        marker.color.g = viz_config.agent_color[1];
        marker.color.b = viz_config.agent_color[2];
        marker.color.a = viz_config.agent_color[3];

        marker_array.markers.push_back(marker);

        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = config_.getFrameId();
        text_marker.header.stamp = timestamp;
        text_marker.ns = "agent_labels";
        text_marker.id = agent_id;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = x * config_.getGridResolution();
        text_marker.pose.position.y = y * config_.getGridResolution();
        text_marker.pose.position.z = z * config_.getGridResolution() + (config_.getGridResolution() * viz_config.agent_scale * 0.5);
        text_marker.pose.orientation.w = 1.0;
        text_marker.text = "A_" + std::to_string(agent_id);

        text_marker.scale.z = config_.getGridResolution() * 0.5; // Text height
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        marker_array.markers.push_back(text_marker);

        agent_id++;
    }

    agent_markers_pub_->publish(marker_array);
}

void VisualizationManager::publishInductStations()
{
    auto timestamp = node_->now();
    const auto& viz_config = config_.getVisualizationConfig();
    
    visualization_msgs::msg::MarkerArray marker_array;

    int marker_id = 0;
    for (const auto & station : config_.getInductStations())
    {
        if (station.size() < 4) continue;

        int64_t x = station[0];
        int64_t y = station[1];
        int64_t z = station[2];
        int64_t station_id = station[3];

        // Create station marker (cylinder)
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = config_.getFrameId();
        marker.header.stamp = timestamp;
        marker.ns = "induct_stations";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x * config_.getGridResolution();
        marker.pose.position.y = y * config_.getGridResolution();
        marker.pose.position.z = z * config_.getGridResolution();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = config_.getGridResolution() * viz_config.induct_station_scale;
        marker.scale.y = config_.getGridResolution() * viz_config.induct_station_scale;
        marker.scale.z = config_.getGridResolution() * viz_config.induct_station_scale * 0.3;
        marker.color.r = viz_config.induct_station_color[0];
        marker.color.g = viz_config.induct_station_color[1];
        marker.color.b = viz_config.induct_station_color[2];
        marker.color.a = viz_config.induct_station_color[3];

        marker_array.markers.push_back(marker);

        // Create label marker
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = config_.getFrameId();
        text_marker.header.stamp = timestamp;
        text_marker.ns = "induct_station_labels";
        text_marker.id = marker_id;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = x * config_.getGridResolution();
        text_marker.pose.position.y = y * config_.getGridResolution();
        text_marker.pose.position.z = z * config_.getGridResolution() + (config_.getGridResolution() * viz_config.induct_station_scale * 0.5);
        text_marker.pose.orientation.w = 1.0;
        text_marker.text = "IN_" + std::to_string(station_id);
        text_marker.scale.z = config_.getGridResolution() * 0.4;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        marker_array.markers.push_back(text_marker);

        marker_id++;
    }

    induct_stations_pub_->publish(marker_array);
}

void VisualizationManager::publishEjectStations()
{
    auto timestamp = node_->now();
    const auto& viz_config = config_.getVisualizationConfig();
    
    visualization_msgs::msg::MarkerArray marker_array;

    int marker_id = 0;
    for (const auto & station : config_.getEjectStations())
    {
        if (station.size() < 4) continue;

        int64_t x = station[0];
        int64_t y = station[1];
        int64_t z = station[2];
        int64_t station_id = station[3];

        // Create station marker (cylinder)
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = config_.getFrameId();
        marker.header.stamp = timestamp;
        marker.ns = "eject_stations";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x * config_.getGridResolution();
        marker.pose.position.y = y * config_.getGridResolution();
        marker.pose.position.z = z * config_.getGridResolution();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = config_.getGridResolution() * viz_config.eject_station_scale;
        marker.scale.y = config_.getGridResolution() * viz_config.eject_station_scale;
        marker.scale.z = config_.getGridResolution() * viz_config.eject_station_scale * 0.3;
        marker.color.r = viz_config.eject_station_color[0];
        marker.color.g = viz_config.eject_station_color[1];
        marker.color.b = viz_config.eject_station_color[2];
        marker.color.a = viz_config.eject_station_color[3];

        marker_array.markers.push_back(marker);

        // Create label marker
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = config_.getFrameId();
        text_marker.header.stamp = timestamp;
        text_marker.ns = "eject_station_labels";
        text_marker.id = marker_id;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = x * config_.getGridResolution();
        text_marker.pose.position.y = y * config_.getGridResolution();
        text_marker.pose.position.z = z * config_.getGridResolution() + (config_.getGridResolution() * viz_config.eject_station_scale * 0.5);
        text_marker.pose.orientation.w = 1.0;
        text_marker.text = "OUT_" + std::to_string(station_id);
        text_marker.scale.z = config_.getGridResolution() * 0.4;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        marker_array.markers.push_back(text_marker);

        marker_id++;
    }

    eject_stations_pub_->publish(marker_array);
}

} // namespace create_gridworld
