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

        if (!loadCSV(data_file_)) {
            RCLCPP_ERROR(get_logger(), "Failed to load CSV file!");
        } else {
            RCLCPP_INFO(get_logger(), "Successfully loaded trajectory data.");
        }
    }
private:
    bool loadCSV(const std::string & filename)
    {
        RCLCPP_INFO(this->get_logger(), "Loading CSV file: %s", filename.c_str());
        std::ifstream file(filename);

        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filename.c_str());
            return false;
        }

        std::string line;
        std::getline(file, line);  // Skip header

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            AgentPosition pos{};
            std::getline(ss, cell, ','); pos.agent_id = std::stoi(cell);
            std::getline(ss, cell, ','); pos.timestep = std::stoi(cell);
            std::getline(ss, cell, ','); pos.location = std::stoi(cell);
            std::getline(ss, cell, ','); pos.x = std::stod(cell);
            std::getline(ss, cell, ','); pos.y = std::stod(cell);
            std::getline(ss, cell, ','); pos.orientation = std::stoi(cell);
            trajectories_.push_back(pos);
        }
        file.close();
        return true;
    }

    // Data
    std::vector<AgentPosition> trajectories_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryExecutor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}