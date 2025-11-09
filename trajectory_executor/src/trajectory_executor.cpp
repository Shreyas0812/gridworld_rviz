#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

struct AgentPosition {
  int agent_id;
  int timestep;
  int location;
  int x;
  int y;
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
            max_timestep_ = 0;
        } else {
            RCLCPP_INFO(get_logger(), "Successfully loaded trajectory data.");
        }
    }

    void run()
    {
        RCLCPP_INFO(this->get_logger(), "Trajectory Executor is running.");
        // Main execution loop can be added here

        RCLCPP_INFO(get_logger(), "Starting simulation up to timestep %d", max_timestep_);

        const auto period = std::chrono::duration<double>(1.0 / 1.0);  // 10 Hz
        for (int t = 0; t <= max_timestep_ && rclcpp::ok(); ++t) {
            RCLCPP_INFO(get_logger(), "Executing timestep %d/%d", t, max_timestep_);
            const auto tic = std::chrono::steady_clock::now();
            executeTimestep(t);
            const auto toc = std::chrono::steady_clock::now();
            const auto elapsed = toc - tic;
            if (elapsed < period) {
                std::this_thread::sleep_for(period - elapsed);
            }
        }
        RCLCPP_INFO(get_logger(), "Simulation complete.");
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
            if (pos.timestep > max_timestep_) {
            max_timestep_ = pos.timestep;
            }
        }
        file.close();
        
        // Create publisher for agent positions
        agent_position_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/agent_position", 10);
        
        return true;
    }

    void executeTimestep(int timestep)
    {
        for (const auto & pos : trajectories_) {
            if (pos.timestep == timestep) {
            RCLCPP_INFO(get_logger(), "Agent %d at timestep %d: (x: %.2f, y: %.2f, orientation: %d)",
                    pos.agent_id, pos.timestep, pos.x, pos.y, pos.orientation);
            
            // Publish agent position
            std_msgs::msg::Int64MultiArray msg;
            msg.data = {pos.agent_id, pos.x, pos.y, 0};
            agent_position_pub_->publish(msg);
            }
        }
    }

    // Data
    std::vector<AgentPosition> trajectories_;
    int max_timestep_ = 0;
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr agent_position_pub_;
    };

    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<TrajectoryExecutor>();
        node->run();
        // rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }