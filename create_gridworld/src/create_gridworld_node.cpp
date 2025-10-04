#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
# include <chrono>
# include <memory>

class CreateGridWorldNode : public rclcpp::Node
{
public:
    CreateGridWorldNode() : Node("create_gridworld_node"), 
                            grid_width_(50), 
                            grid_height_(50), 
                            grid_depth_(10),
                            grid_resolution_(1.0)
    {
        this->declare_parameter("grid_width", grid_width_);
        this->declare_parameter("grid_height", grid_height_);
        this->declare_parameter("grid_depth", grid_depth_);
        this->declare_parameter("grid_resolution", grid_resolution_);

        grid_width_ = this->get_parameter("grid_width").as_int();
        grid_height_ = this->get_parameter("grid_height").as_int();
        grid_depth_ = this->get_parameter("grid_depth").as_int();
        grid_resolution_ = this->get_parameter("grid_resolution").as_double();

        // Publisher for the occupancy grid
        grid_nodes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("gridworld/nodes", 10);
        
        grid_edges_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("gridworld/edges", 10);
        
        agent_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("gridworld/agents", 10);

        obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("gridworld/obstacles", 10);

        // Timer to publish the grid at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CreateGridWorldNode::publish_gridworld, this)
        );

        initialize_grid();

        RCLCPP_INFO(this->get_logger(), "CreateGridWorldNode has been started.");
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
        for(int i=15; i<35; i++) {
            for(int j=15; j<35; j++) {
                for(int k=0; k<grid_depth_; k++) {
                    set_obstacle(i, j, k, true);
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
    }

    void publish_grid_nodes(const rclcpp::Time & timestamp)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = timestamp;
        marker.ns = "grid_nodes";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = grid_resolution_ * 0.1;
        marker.scale.y = grid_resolution_ * 0.1;
        marker.scale.z = grid_resolution_ * 0.1;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

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
        lines_marker.header.frame_id = "map";
        lines_marker.header.stamp = timestamp;
        lines_marker.ns = "grid_edges";
        lines_marker.id = 0;
        lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        lines_marker.action = visualization_msgs::msg::Marker::ADD;
        lines_marker.pose.orientation.w = 1.0;

        lines_marker.scale.x = 0.05; // Line width
        lines_marker.color.r = 0.0;
        lines_marker.color.g = 0.0;
        lines_marker.color.b = 1.0;
        lines_marker.color.a = 0.5;

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

        cubes_marker.header.frame_id = "map";
        cubes_marker.header.stamp = timestamp;
        cubes_marker.ns = "obstacles";
        cubes_marker.id = 0;
        cubes_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        cubes_marker.action = visualization_msgs::msg::Marker::ADD;
        cubes_marker.pose.orientation.w = 1.0;

        cubes_marker.scale.x = grid_resolution_ * 0.9;
        cubes_marker.scale.y = grid_resolution_ * 0.9;
        cubes_marker.scale.z = grid_resolution_ * 0.9;

        cubes_marker.color.r = 1.0;
        cubes_marker.color.g = 0.0;
        cubes_marker.color.b = 0.0;
        cubes_marker.color.a = 0.8;

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

        //agents are at node (5,5,0) and (20,20,0)
        std::vector<std::tuple<int, int, int>> agent_positions = { {5, 5, 0}, {20, 20, 0} };
        
        int agent_id = 0;
        for (const auto & [x, y, z] : agent_positions)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = timestamp;
            marker.ns = "agents";
            marker.id = agent_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x * grid_resolution_;
            marker.pose.position.y = y * grid_resolution_;
            marker.pose.position.z = z * grid_resolution_;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = grid_resolution_ * 0.8;
            marker.scale.y = grid_resolution_ * 0.8;
            marker.scale.z = grid_resolution_ * 0.8;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }

        agent_markers_pub_->publish(marker_array);
    }

    // Node members
    int grid_width_;
    int grid_height_;
    int grid_depth_;
    double grid_resolution_;

    // Occupancy grid message
    std::vector<GridNode> nodes_;

    // ROS2 publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_nodes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_edges_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr agent_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CreateGridWorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}