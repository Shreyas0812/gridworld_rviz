#include "rclcpp/rclcpp.hpp"

class CreateGridWorldNode : public rclcpp::Node
{
public:
    CreateGridWorldNode() : Node("create_gridworld_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello cpp");
    }
private:
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CreateGridWorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}