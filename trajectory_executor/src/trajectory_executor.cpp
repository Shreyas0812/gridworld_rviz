#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <unordered_map>

struct AgentPosition {
  int agent_id;
  int x;
  int y;
  int z;
  int timestep;
};

class TrajectoryExecutor : public rclcpp::Node
{
public:
    TrajectoryExecutor() : Node("trajectory_executor")
    {
        RCLCPP_INFO(this->get_logger(), "trajectory executor running");
        // Declare parameters
        this->declare_parameter<std::string>("data_file", "trajectories.csv");
        this->declare_parameter<double>("rate_hz", 1.0);

        // Get parameters
        std::string data_file = this->get_parameter("data_file").as_string();
        rate_hz_ = this->get_parameter("rate_hz").as_double();
        RCLCPP_INFO(get_logger(), "Loading trajectory data from: %s", data_file.c_str());

        // Create publisher for agent positions
        agent_position_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/agent_position", 10);

        if (!loadCSV(data_file)) {
            RCLCPP_ERROR(get_logger(), "Failed to load CSV file!");
            max_timestep_ = 0;
        } else {
            RCLCPP_INFO(get_logger(), "Loaded trajectory data — max timestep: %d", max_timestep_);
        }
    }

    void run()
    {
        RCLCPP_INFO(get_logger(), "Starting simulation up to timestep %d at %.1f Hz", max_timestep_, rate_hz_);
        const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, rate_hz_));
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
            std::getline(ss, cell, ','); pos.x = std::stoi(cell);
            std::getline(ss, cell, ','); pos.y = std::stoi(cell);
            std::getline(ss, cell, ','); pos.z = std::stoi(cell);
            std::getline(ss, cell, ','); pos.timestep = std::stoi(cell);
            timestep_index_[pos.timestep].push_back(pos);
            if (pos.timestep > max_timestep_) {
                max_timestep_ = pos.timestep;
            }
        }
        file.close();
        
        return true;
    }

    void executeTimestep(int timestep)
    {
        auto it = timestep_index_.find(timestep);
        if (it == timestep_index_.end()) return;

        for (const auto & pos : it->second) {
            RCLCPP_INFO(get_logger(), "Agent %d at timestep %d: (x: %d, y: %d, z: %d)",
                    pos.agent_id, pos.timestep, pos.x, pos.y, pos.z);
            
            // Publish agent position
            std_msgs::msg::Int64MultiArray msg;
            msg.data = {pos.agent_id, pos.x, pos.y, pos.z};
            agent_position_pub_->publish(msg);
        }
    }

    // Data
    std::unordered_map<int, std::vector<AgentPosition>> timestep_index_;
    int max_timestep_ = 0;
    double rate_hz_ = 1.0;
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr agent_position_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryExecutor>();
    // Spin in background thread so publishers flush properly
    std::thread spin_thread([node]() { rclcpp::spin(node); });
    node->run();
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}