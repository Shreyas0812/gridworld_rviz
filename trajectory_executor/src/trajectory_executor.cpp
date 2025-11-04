#include "rclcpp/rclcpp.hpp"

class TrajectoryExecutor : public rclcpp::Node
{
public:
    TrajectoryExecutor() : Node("trajectory_executor")
    {
        RCLCPP_INFO(this->get_logger(), "trajectory executor running");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryExecutor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}