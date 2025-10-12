#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include <chrono>
#include <memory>

class CreateGridWorldNode : public rclcpp::Node
{
public:
    CreateGridWorldNode() : Node("create_gridworld_node"), 
                            grid_width_(50), 
                            grid_height_(50), 
                            grid_depth_(10),
                            grid_resolution_(1.0),
                            publish_rate_(10.0),
                            frame_id_("map")
    {
        // Grid parameters
        this->declare_parameter("grid_width", grid_width_);
        this->declare_parameter("grid_height", grid_height_);
        this->declare_parameter("grid_depth", grid_depth_);
        this->declare_parameter("grid_resolution", grid_resolution_);
        this->declare_parameter("publish_rate", publish_rate_);
        this->declare_parameter("frame_id", frame_id_);
        
        // Obstacle regions as a list of lists: [[x1, y1, z1], [x2, y2, z2], ...]
        this->declare_parameter("obstacle_regions", std::vector<int64_t>{});

        // Agent positions as a list of lists: [[x1, y1, z1], [x2, y2, z2], ...]
        this->declare_parameter("agent_positions", std::vector<int64_t>{});

        // Station parameters: [x, y, z, station_id, x, y, z, station_id, ...]
        this->declare_parameter("induct_stations", std::vector<int64_t>{});
        this->declare_parameter("eject_stations", std::vector<int64_t>{});

        // Declare visualization parameters
        this->declare_parameter("visualization.node_scale", 0.1);
        this->declare_parameter("visualization.node_color", std::vector<double>{0.0, 1.0, 0.0, 0.5});
        this->declare_parameter("visualization.edge_width", 0.05);
        this->declare_parameter("visualization.edge_color", std::vector<double>{0.0, 0.0, 1.0, 0.5});
        this->declare_parameter("visualization.obstacle_scale", 0.9);
        this->declare_parameter("visualization.obstacle_color", std::vector<double>{1.0, 0.0, 0.0, 0.8});
        this->declare_parameter("visualization.agent_scale", 0.8);
        this->declare_parameter("visualization.agent_color", std::vector<double>{0.0, 0.0, 1.0, 1.0});
        this->declare_parameter("visualization.induct_station_scale", 1.2);
        this->declare_parameter("visualization.induct_station_color", std::vector<double>{0.0, 0.5, 1.0, 1.0});
        this->declare_parameter("visualization.eject_station_scale", 1.2);
        this->declare_parameter("visualization.eject_station_color", std::vector<double>{1.0, 0.5, 0.0, 1.0});

        // Declare topic parameters
        this->declare_parameter("topics.nodes", std::string("gridworld/nodes"));
        this->declare_parameter("topics.edges", std::string("gridworld/edges"));
        this->declare_parameter("topics.obstacles", std::string("gridworld/obstacles"));
        this->declare_parameter("topics.agents", std::string("gridworld/agents"));
        this->declare_parameter("topics.induct_stations", std::string("gridworld/induct_stations"));
        this->declare_parameter("topics.eject_stations", std::string("gridworld/eject_stations"));

        grid_width_ = this->get_parameter("grid_width").as_int();
        grid_height_ = this->get_parameter("grid_height").as_int();
        grid_depth_ = this->get_parameter("grid_depth").as_int();
        grid_resolution_ = this->get_parameter("grid_resolution").as_double();

        // Retrieve obstacle_regions as a vector of integer arrays
        auto flat_obstacles = this->get_parameter("obstacle_regions").as_integer_array();
        obstacle_regions_.clear();
        for (size_t i = 0; i < flat_obstacles.size(); i += 6) {
            obstacle_regions_.emplace_back(std::vector<int64_t>{
                flat_obstacles[i], flat_obstacles[i + 1], flat_obstacles[i + 2],
                flat_obstacles[i + 3], flat_obstacles[i + 4], flat_obstacles[i + 5]
            });
        }

        auto flat_agents = this->get_parameter("agent_positions").as_integer_array();
        agent_positions_.clear();
        for (size_t i = 0; i < flat_agents.size(); i += 3) {
            agent_positions_.emplace_back(std::vector<int64_t>{
                flat_agents[i], flat_agents[i + 1], flat_agents[i + 2]
            });
        }

        // Retrieve induct stations: [x, y, z, station_id, ...]
        auto flat_induct = this->get_parameter("induct_stations").as_integer_array();
        induct_stations_.clear();
        for (size_t i = 0; i < flat_induct.size(); i += 4) {
            induct_stations_.emplace_back(std::vector<int64_t>{
                flat_induct[i], flat_induct[i + 1], flat_induct[i + 2], flat_induct[i + 3]
            });
        }

        // Retrieve eject stations: [x, y, z, station_id, ...]
        auto flat_eject = this->get_parameter("eject_stations").as_integer_array();
        eject_stations_.clear();
        for (size_t i = 0; i < flat_eject.size(); i += 4) {
            eject_stations_.emplace_back(std::vector<int64_t>{
                flat_eject[i], flat_eject[i + 1], flat_eject[i + 2], flat_eject[i + 3]
            });
        }

        // Get visualization parameters
        viz_config_.node_scale = this->get_parameter("visualization.node_scale").as_double();
        viz_config_.node_color = this->get_parameter("visualization.node_color").as_double_array();
        viz_config_.edge_width = this->get_parameter("visualization.edge_width").as_double();
        viz_config_.edge_color = this->get_parameter("visualization.edge_color").as_double_array();
        viz_config_.obstacle_scale = this->get_parameter("visualization.obstacle_scale").as_double();
        viz_config_.obstacle_color = this->get_parameter("visualization.obstacle_color").as_double_array();
        viz_config_.agent_scale = this->get_parameter("visualization.agent_scale").as_double();
        viz_config_.agent_color = this->get_parameter("visualization.agent_color").as_double_array();
        viz_config_.induct_station_scale = this->get_parameter("visualization.induct_station_scale").as_double();
        viz_config_.induct_station_color = this->get_parameter("visualization.induct_station_color").as_double_array();
        viz_config_.eject_station_scale = this->get_parameter("visualization.eject_station_scale").as_double();
        viz_config_.eject_station_color = this->get_parameter("visualization.eject_station_color").as_double_array();
        
        // Get topic names
        topic_name_.nodes = this->get_parameter("topics.nodes").as_string();
        topic_name_.edges = this->get_parameter("topics.edges").as_string();
        topic_name_.obstacles = this->get_parameter("topics.obstacles").as_string();
        topic_name_.agents = this->get_parameter("topics.agents").as_string();
        topic_name_.induct_stations = this->get_parameter("topics.induct_stations").as_string();
        topic_name_.eject_stations = this->get_parameter("topics.eject_stations").as_string();

        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu obstacle regions, %zu agent start positions, %zu induct stations, %zu eject stations", 
            obstacle_regions_.size(), agent_positions_.size(), induct_stations_.size(), eject_stations_.size());
        
        // Publishers
        grid_nodes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name_.nodes, 10);
        grid_edges_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name_.edges, 10);
        obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name_.obstacles, 10);
        agent_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name_.agents, 10);
        induct_stations_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name_.induct_stations, 10);
        eject_stations_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name_.eject_stations, 10);

        // Subscriber for agent positions
        agent_position_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "agent_position", 
            10,
            std::bind(&CreateGridWorldNode::agent_position_callback, this, std::placeholders::_1)
        );

        // Timer to publish the grid at regular intervals
        int timer_period_ms = static_cast<int>(1000.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms),
            std::bind(&CreateGridWorldNode::publish_gridworld, this)
        );

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        RCLCPP_INFO(this->get_logger(), "Grid width: %d", grid_width_);
        RCLCPP_INFO(this->get_logger(), "Grid height: %d", grid_height_);
        RCLCPP_INFO(this->get_logger(), "Grid depth: %d", grid_depth_);
        RCLCPP_INFO(this->get_logger(), "Grid resolution: %.2f", grid_resolution_);
        RCLCPP_INFO(this->get_logger(), "Publish rate: %.2f Hz", publish_rate_);
        RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());

        RCLCPP_INFO(this->get_logger(), "Obstacle regions:");
        for (const auto & region : obstacle_regions_) {
            std::ostringstream oss;
            oss << "[";
            for (size_t i = 0; i < region.size(); ++i) {
                oss << region[i];
                if (i + 1 < region.size()) oss << ", ";
            }
            oss << "]";
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Agent start positions:");
        for (const auto & pos : agent_positions_) {
            std::ostringstream oss;
            oss << "[";
            for (size_t i = 0; i < pos.size(); ++i) {
                oss << pos[i];
                if (i + 1 < pos.size()) oss << ", ";
            }
            oss << "]";
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Induct stations:");
        for (const auto & station : induct_stations_) {
            RCLCPP_INFO(this->get_logger(), "  [%ld, %ld, %ld] ID: %ld", 
                station[0], station[1], station[2], station[3]);
        }

        RCLCPP_INFO(this->get_logger(), "Eject stations:");
        for (const auto & station : eject_stations_) {
            RCLCPP_INFO(this->get_logger(), "  [%ld, %ld, %ld] ID: %ld", 
                station[0], station[1], station[2], station[3]);
        }

        RCLCPP_INFO(this->get_logger(), "Visualization config:");
        RCLCPP_INFO(this->get_logger(), "  node_scale: %.2f", viz_config_.node_scale);
        RCLCPP_INFO(this->get_logger(), "  node_color: [%.2f, %.2f, %.2f, %.2f]", 
            viz_config_.node_color[0], viz_config_.node_color[1], viz_config_.node_color[2], viz_config_.node_color[3]);
        RCLCPP_INFO(this->get_logger(), "  edge_width: %.2f", viz_config_.edge_width);
        RCLCPP_INFO(this->get_logger(), "  edge_color: [%.2f, %.2f, %.2f, %.2f]", 
            viz_config_.edge_color[0], viz_config_.edge_color[1], viz_config_.edge_color[2], viz_config_.edge_color[3]);
        RCLCPP_INFO(this->get_logger(), "  obstacle_scale: %.2f", viz_config_.obstacle_scale);
        RCLCPP_INFO(this->get_logger(), "  obstacle_color: [%.2f, %.2f, %.2f, %.2f]", 
            viz_config_.obstacle_color[0], viz_config_.obstacle_color[1], viz_config_.obstacle_color[2], viz_config_.obstacle_color[3]);
        RCLCPP_INFO(this->get_logger(), "  agent_scale: %.2f", viz_config_.agent_scale);
        RCLCPP_INFO(this->get_logger(), "  agent_color: [%.2f, %.2f, %.2f, %.2f]", 
            viz_config_.agent_color[0], viz_config_.agent_color[1], viz_config_.agent_color[2], viz_config_.agent_color[3]);
        RCLCPP_INFO(this->get_logger(), "  induct_station_scale: %.2f", viz_config_.induct_station_scale);
        RCLCPP_INFO(this->get_logger(), "  induct_station_color: [%.2f, %.2f, %.2f, %.2f]", 
            viz_config_.induct_station_color[0], viz_config_.induct_station_color[1], viz_config_.induct_station_color[2], viz_config_.induct_station_color[3]);
        RCLCPP_INFO(this->get_logger(), "  eject_station_scale: %.2f", viz_config_.eject_station_scale);
        RCLCPP_INFO(this->get_logger(), "  eject_station_color: [%.2f, %.2f, %.2f, %.2f]", 
            viz_config_.eject_station_color[0], viz_config_.eject_station_color[1], viz_config_.eject_station_color[2], viz_config_.eject_station_color[3]);

        RCLCPP_INFO(this->get_logger(), "Topic names:");
        RCLCPP_INFO(this->get_logger(), "  nodes: %s", topic_name_.nodes.c_str());
        RCLCPP_INFO(this->get_logger(), "  edges: %s", topic_name_.edges.c_str());
        RCLCPP_INFO(this->get_logger(), "  obstacles: %s", topic_name_.obstacles.c_str());
        RCLCPP_INFO(this->get_logger(), "  agents: %s", topic_name_.agents.c_str());
        RCLCPP_INFO(this->get_logger(), "  induct_stations: %s", topic_name_.induct_stations.c_str());
        RCLCPP_INFO(this->get_logger(), "  eject_stations: %s", topic_name_.eject_stations.c_str());

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        initialize_grid();

        RCLCPP_INFO(this->get_logger(), "CreateGridWorldNode started with %dx%dx%d grid, resolution %.2f", 
                    grid_width_, grid_height_, grid_depth_, grid_resolution_);
    }

private:
    struct GridNode
    {
        int x;
        int y;
        int z;
        bool is_obstacle;
        bool is_traversable;
        std::vector<int> neighbors;
        
        GridNode(int x_, int y_, int z_)
            : x(x_), y(y_), z(z_), is_obstacle(false), is_traversable(true) {}
    };

    void initialize_grid()
    {
        nodes_.clear();

        for (int x = 0; x < grid_width_; ++x)
        {
            for (int y = 0; y < grid_height_; ++y)
            {
                for (int z = 0; z < grid_depth_; ++z)
                {
                    nodes_.emplace_back(x, y, z);
                }
            }
        }

        add_obstacles();
        compute_neighbors();
    }

    void add_obstacles()
    {
        for (const auto & region : obstacle_regions_)
        {
            if (region.size() < 6) continue; // Invalid region
            
            int start_x = region[0];
            int start_y = region[1];
            int start_z = region[2];
            int end_x = region[3];
            int end_y = region[4];
            int end_z = region[5];
            for (int x = start_x; x <= end_x; ++x)
            {
                for (int y = start_y; y <= end_y; ++y)
                {
                    for (int z = start_z; z <= end_z; ++z)
                    {
                        set_obstacle(x, y, z, true);
                    }
                }
            }
        }
    }

    void set_obstacle(int x, int y, int z, bool is_obstacle)
    {
        int index = get_node_index(x, y, z);
        if (index == -1) return; // Out of bounds
        nodes_[index].is_obstacle = is_obstacle;
        nodes_[index].is_traversable = !is_obstacle;
    }

    int get_node_index(int x, int y, int z)
    {
        if (x < 0 || x >= grid_width_ || y < 0 || y >= grid_height_ || z < 0 || z >= grid_depth_)
        {
            return -1; // Out of bounds
        }
        return x * grid_height_ * grid_depth_ + y * grid_depth_ + z;
    }

    void compute_neighbors()
    {
        const std::vector<std::tuple<int, int, int>> directions = {
            {1, 0, 0}, {-1, 0, 0},
            {0, 1, 0}, {0, -1, 0},
            {0, 0, 1}, {0, 0, -1}
        };

        for (auto & node : nodes_)
        {
            node.neighbors.clear();
            for (const auto & [dx, dy, dz] : directions)
            {
                int nx = node.x + dx;
                int ny = node.y + dy;
                int nz = node.z + dz;
                int neighbor_index = get_node_index(nx, ny, nz);
                if (neighbor_index != -1 && !nodes_[neighbor_index].is_obstacle)
                {
                    node.neighbors.push_back(neighbor_index);
                }
            }
        }
    }

    void publish_gridworld()
    {
        auto timestamp = this->now();

        publish_grid_nodes(timestamp);
        publish_grid_edges(timestamp);
        publish_obstacles(timestamp);
        publish_agent_markers(timestamp);
        publish_induct_stations(timestamp);
        publish_eject_stations(timestamp);
    }

    void publish_grid_nodes(const rclcpp::Time & timestamp)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = timestamp;
        marker.ns = "grid_nodes";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = grid_resolution_ * viz_config_.node_scale;
        marker.scale.y = grid_resolution_ * viz_config_.node_scale;
        marker.scale.z = grid_resolution_ * viz_config_.node_scale;

        marker.color.r = viz_config_.node_color[0];
        marker.color.g = viz_config_.node_color[1];
        marker.color.b = viz_config_.node_color[2];
        marker.color.a = viz_config_.node_color[3];

        for (const auto & node : nodes_)
        {
            if (!node.is_obstacle)
            {
                geometry_msgs::msg::Point p;
                p.x = node.x * grid_resolution_;
                p.y = node.y * grid_resolution_;
                p.z = node.z * grid_resolution_;
                marker.points.push_back(p);
            }
        }
        
        marker_array.markers.push_back(marker);
        grid_nodes_pub_->publish(marker_array);
    }

    void publish_grid_edges(const rclcpp::Time & timestamp)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker lines_marker;
        lines_marker.header.frame_id = frame_id_;
        lines_marker.header.stamp = timestamp;
        lines_marker.ns = "grid_edges";
        lines_marker.id = 0;
        lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        lines_marker.action = visualization_msgs::msg::Marker::ADD;
        lines_marker.pose.orientation.w = 1.0;

        lines_marker.scale.x = viz_config_.edge_width;
        lines_marker.color.r = viz_config_.edge_color[0];
        lines_marker.color.g = viz_config_.edge_color[1];
        lines_marker.color.b = viz_config_.edge_color[2];
        lines_marker.color.a = viz_config_.edge_color[3];

        for (size_t i=0; i< nodes_.size(); i++)
        {
            const auto &node = nodes_[i];
            if (node.is_obstacle) continue;

            for (int neighbor_index : node.neighbors)
            {
                if (static_cast<int>(i) >= neighbor_index) continue; // Avoid duplicate edges

                geometry_msgs::msg::Point pstart, pend;
                pstart.x = node.x * grid_resolution_;
                pstart.y = node.y * grid_resolution_;
                pstart.z = node.z * grid_resolution_;

                pend.x = nodes_[neighbor_index].x * grid_resolution_;
                pend.y = nodes_[neighbor_index].y * grid_resolution_;
                pend.z = nodes_[neighbor_index].z * grid_resolution_;
                
                lines_marker.points.push_back(pstart);
                lines_marker.points.push_back(pend);
            }
        }

        marker_array.markers.push_back(lines_marker);
        grid_edges_pub_->publish(marker_array);
    }

    void publish_obstacles(const rclcpp::Time & timestamp)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker cubes_marker;

        cubes_marker.header.frame_id = frame_id_;
        cubes_marker.header.stamp = timestamp;
        cubes_marker.ns = "obstacles";
        cubes_marker.id = 0;
        cubes_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        cubes_marker.action = visualization_msgs::msg::Marker::ADD;
        cubes_marker.pose.orientation.w = 1.0;

        cubes_marker.scale.x = grid_resolution_ * viz_config_.obstacle_scale;
        cubes_marker.scale.y = grid_resolution_ * viz_config_.obstacle_scale;
        cubes_marker.scale.z = grid_resolution_ * viz_config_.obstacle_scale;

        cubes_marker.color.r = viz_config_.obstacle_color[0];
        cubes_marker.color.g = viz_config_.obstacle_color[1];
        cubes_marker.color.b = viz_config_.obstacle_color[2];
        cubes_marker.color.a = viz_config_.obstacle_color[3];

        for (const auto & node : nodes_)
        {
            if (node.is_obstacle)
            {
                geometry_msgs::msg::Point p;
                p.x = node.x * grid_resolution_;
                p.y = node.y * grid_resolution_;
                p.z = node.z * grid_resolution_;
                cubes_marker.points.push_back(p);
            }
        }

        marker_array.markers.push_back(cubes_marker);
        obstacles_pub_->publish(marker_array);
    }

    void publish_agent_markers(const rclcpp::Time & timestamp)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        std::vector<std::tuple<int, int, int>> agent_pos;
        for (const auto & pos : agent_positions_)
        {
            if (pos.size() < 3) continue;
            agent_pos.emplace_back(pos[0], pos[1], pos[2]);
        }
        
        int agent_id = 0;
        for (const auto & [x, y, z] : agent_pos)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = timestamp;
            marker.ns = "agents";
            marker.id = agent_id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x * grid_resolution_;
            marker.pose.position.y = y * grid_resolution_;
            marker.pose.position.z = z * grid_resolution_;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = grid_resolution_ * viz_config_.agent_scale;
            marker.scale.y = grid_resolution_ * viz_config_.agent_scale;
            marker.scale.z = grid_resolution_ * viz_config_.agent_scale;
            marker.color.r = viz_config_.agent_color[0];
            marker.color.g = viz_config_.agent_color[1];
            marker.color.b = viz_config_.agent_color[2];
            marker.color.a = viz_config_.agent_color[3];

            marker_array.markers.push_back(marker);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = frame_id_;
            text_marker.header.stamp = timestamp;
            text_marker.ns = "agent_labels";
            text_marker.id = agent_id;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = x * grid_resolution_;
            text_marker.pose.position.y = y * grid_resolution_;
            text_marker.pose.position.z = z * grid_resolution_ + (grid_resolution_ * viz_config_.agent_scale * 0.5);
            text_marker.pose.orientation.w = 1.0;
            text_marker.text = "A_" + std::to_string(agent_id);

            text_marker.scale.z = grid_resolution_ * 0.5; // Text height
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;

            marker_array.markers.push_back(text_marker);

            agent_id++;
        }

        agent_markers_pub_->publish(marker_array);
    }

    void publish_induct_stations(const rclcpp::Time & timestamp)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        int marker_id = 0;
        for (const auto & station : induct_stations_)
        {
            if (station.size() < 4) continue;

            int64_t x = station[0];
            int64_t y = station[1];
            int64_t z = station[2];
            int64_t station_id = station[3];

            // Create station marker (cylinder)
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = timestamp;
            marker.ns = "induct_stations";
            marker.id = marker_id;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x * grid_resolution_;
            marker.pose.position.y = y * grid_resolution_;
            marker.pose.position.z = z * grid_resolution_;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = grid_resolution_ * viz_config_.induct_station_scale;
            marker.scale.y = grid_resolution_ * viz_config_.induct_station_scale;
            marker.scale.z = grid_resolution_ * viz_config_.induct_station_scale * 0.3;
            marker.color.r = viz_config_.induct_station_color[0];
            marker.color.g = viz_config_.induct_station_color[1];
            marker.color.b = viz_config_.induct_station_color[2];
            marker.color.a = viz_config_.induct_station_color[3];

            marker_array.markers.push_back(marker);

            // Create label marker
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = frame_id_;
            text_marker.header.stamp = timestamp;
            text_marker.ns = "induct_station_labels";
            text_marker.id = marker_id;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = x * grid_resolution_;
            text_marker.pose.position.y = y * grid_resolution_;
            text_marker.pose.position.z = z * grid_resolution_ + (grid_resolution_ * viz_config_.induct_station_scale * 0.5);
            text_marker.pose.orientation.w = 1.0;
            text_marker.text = "IN_" + std::to_string(station_id);
            text_marker.scale.z = grid_resolution_ * 0.4;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;

            marker_array.markers.push_back(text_marker);

            marker_id++;
        }

        induct_stations_pub_->publish(marker_array);
    }

    void publish_eject_stations(const rclcpp::Time & timestamp)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        int marker_id = 0;
        for (const auto & station : eject_stations_)
        {
            if (station.size() < 4) continue;

            int64_t x = station[0];
            int64_t y = station[1];
            int64_t z = station[2];
            int64_t station_id = station[3];

            // Create station marker (cylinder)
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = timestamp;
            marker.ns = "eject_stations";
            marker.id = marker_id;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x * grid_resolution_;
            marker.pose.position.y = y * grid_resolution_;
            marker.pose.position.z = z * grid_resolution_;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = grid_resolution_ * viz_config_.eject_station_scale;
            marker.scale.y = grid_resolution_ * viz_config_.eject_station_scale;
            marker.scale.z = grid_resolution_ * viz_config_.eject_station_scale * 0.3;
            marker.color.r = viz_config_.eject_station_color[0];
            marker.color.g = viz_config_.eject_station_color[1];
            marker.color.b = viz_config_.eject_station_color[2];
            marker.color.a = viz_config_.eject_station_color[3];

            marker_array.markers.push_back(marker);

            // Create label marker
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = frame_id_;
            text_marker.header.stamp = timestamp;
            text_marker.ns = "eject_station_labels";
            text_marker.id = marker_id;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = x * grid_resolution_;
            text_marker.pose.position.y = y * grid_resolution_;
            text_marker.pose.position.z = z * grid_resolution_ + (grid_resolution_ * viz_config_.eject_station_scale * 0.5);
            text_marker.pose.orientation.w = 1.0;
            text_marker.text = "OUT_" + std::to_string(station_id);
            text_marker.scale.z = grid_resolution_ * 0.4;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;

            marker_array.markers.push_back(text_marker);

            marker_id++;
        }

        eject_stations_pub_->publish(marker_array);
    }

    void agent_position_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
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

        if (x < 0 || x >= grid_width_ || y < 0 || y >= grid_height_ || z < 0 || z >= grid_depth_)
        {
            RCLCPP_WARN(this->get_logger(), "Received out-of-bounds agent position: [%ld, %ld, %ld]", x, y, z);
            return;
        }

        // Initialize new agent position if it doesn't exist
        if (agent_positions_[agent_index].empty())
        {
            agent_positions_[agent_index] = std::vector<int64_t>(3, 0);
            RCLCPP_INFO(this->get_logger(), "Initialized new agent %d", agent_index);
        }

        // Update agent position
        agent_positions_[agent_index][0] = x;
        agent_positions_[agent_index][1] = y;
        agent_positions_[agent_index][2] = z;

        RCLCPP_DEBUG(this->get_logger(), "Updated agent %d to position (%ld, %ld, %ld)", 
                    agent_index, x, y, z);
    }

    void update_agent_position(int agent_index, int new_x, int new_y, int new_z)
    {
        if (agent_index < 0 || agent_index >= static_cast<int>(agent_positions_.size()))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid agent index %d", agent_index);
            return;
        }
        agent_positions_[agent_index][0] = new_x;
        agent_positions_[agent_index][1] = new_y;
        agent_positions_[agent_index][2] = new_z;
    }

    // Grid
    int grid_width_;
    int grid_height_;
    int grid_depth_;
    double grid_resolution_;

    double publish_rate_;
    std::string frame_id_;
    std::vector<std::vector<int64_t>> obstacle_regions_;
    std::vector<std::vector<int64_t>> agent_positions_;
    std::vector<std::vector<int64_t>> induct_stations_;
    std::vector<std::vector<int64_t>> eject_stations_;

    // Visualization parameters
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
    } viz_config_;

    // Topic names
    struct TopicConfig {
        std::string nodes;
        std::string edges;
        std::string obstacles;
        std::string agents;
        std::string induct_stations;
        std::string eject_stations;
    } topic_name_;

    // Grid nodes
    std::vector<GridNode> nodes_;

    // ROS2 publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_nodes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_edges_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr agent_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr induct_stations_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr eject_stations_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ROS2 subscribers
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr agent_position_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CreateGridWorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
