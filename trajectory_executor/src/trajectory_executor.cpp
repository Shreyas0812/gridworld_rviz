#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

struct AgentPosition {
  int agent_id;
  int timestep;
  int location;
  double x;
  double y;
  int orientation;
};

class TrajectoryExecutor : public rclcpp::Node
{
public:
    TrajectoryExecutor() : Node("trajectory_executor")
    {
        RCLCPP_INFO(this->get_logger(), "trajectory executor running");
        // Declare parameters
        this->declare_parameter<std::string>("data_file", "trajectories.csv");

        // Get parameters
        std::string data_file_ = this->get_parameter("data_file").as_string();
        RCLCPP_INFO(get_logger(), "Loading trajectory data from: %s", data_file_.c_str());
    }
private:
    bool loadCSV(const std::string & filename)
    {
        RCLCPP_INFO(this->get_logger(), "Loading CSV file: %s", filename.c_str());
        std::ifstream file(filename);
        
        return true;
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