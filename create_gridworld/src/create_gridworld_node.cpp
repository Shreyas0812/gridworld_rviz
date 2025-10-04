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
    CreateGridWorldNode() : Node("create_gridworld_node"), grid_width_(50), grid_height_(50), grid_resolution_(0.1)
    {
        // Publisher for the occupancy grid
        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("gridworld/occupancy_grid", 10);
        agent_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("gridworld/agents", 10);

        // Timer to publish the grid at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CreateGridWorldNode::publish_gridworld, this)
        );

        initialize_grid();

        RCLCPP_INFO(this->get_logger(), "CreateGridWorldNode has been started.");
    }
private:
    void initialize_grid()
    {
        grid_msg_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        grid_msg_->header.frame_id = "map";
        
        grid_msg_->info.width = grid_width_;
        grid_msg_->info.height = grid_height_;
        grid_msg_->info.resolution = grid_resolution_;
        
        grid_msg_->info.origin.position.x = -(grid_width_ * grid_resolution_) / 2.0;
        grid_msg_->info.origin.position.y = -(grid_height_ * grid_resolution_) / 2.0;

        grid_msg_->info.origin.position.z = 0.0;
        grid_msg_->info.origin.orientation.w = 1.0;

        // Initialize all cells to free space (0=free, 100=occupied, -1=unknown)
        grid_msg_->data.resize(grid_width_ * grid_height_, 0);
    }

    void publish_gridworld()
    {
        // Update the header timestamp
        grid_msg_->header.stamp = this->now();

        // Publish the occupancy grid
        grid_pub_->publish(*grid_msg_);

        // Publish agent markers (for simplicity, publishing a single static agent here)
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker agent_marker;
        agent_marker.header.frame_id = "map";
        agent_marker.header.stamp = this->now();
        agent_marker.ns = "agents";
        agent_marker.id = 0;
        agent_marker.type = visualization_msgs::msg::Marker::SPHERE;
        agent_marker.action = visualization_msgs::msg::Marker::ADD;
        agent_marker.pose.position.x = 0.0; // Center of the grid
        agent_marker.pose.position.y = 0.0;
        agent_marker.pose.position.z = 0.5; // Slightly above the grid
        agent_marker.pose.orientation.w = 1.0;
        agent_marker.scale.x = 0.2; // Size of the marker
        agent_marker.scale.y = 0.2;
        agent_marker.scale.z = 0.2;
        agent_marker.color.a = 1.0; // Fully opaque
        agent_marker.color.r = 1.0; // Red color
        agent_marker.color.g = 0.0;
        agent_marker.color.b = 0.0;

        marker_array.markers.push_back(agent_marker);
        agent_marker_pub_->publish(marker_array);
    }

    // Node members
    int grid_width_;
    int grid_height_;
    double grid_resolution_;

    // Occupancy grid message
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_msg_;

    // ROS2 publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr agent_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CreateGridWorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}