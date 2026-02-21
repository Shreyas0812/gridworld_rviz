#include "create_gridworld/grid_world.hpp"
#include <tuple>

namespace create_gridworld {

GridNode::GridNode(int x_, int y_, int z_)
    : x(x_), y(y_), z(z_), is_obstacle(false), is_traversable(true) {}

GridWorld::GridWorld(int width, int height, int depth, double resolution)
    : grid_width_(width), grid_height_(height), grid_depth_(depth), grid_resolution_(resolution)
{
    initializeGrid();
}

void GridWorld::initializeGrid()
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
}

void GridWorld::addObstacleRegions(const std::vector<std::vector<int64_t>>& regions)
{
    for (const auto & region : regions)
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
                    setObstacle(x, y, z, true);
                }
            }
        }
    }
    computeNeighbors();
}

void GridWorld::setObstacle(int x, int y, int z, bool is_obstacle)
{
    int index = getNodeIndex(x, y, z);
    if (index == -1) return; // Out of bounds
    nodes_[index].is_obstacle = is_obstacle;
    nodes_[index].is_traversable = !is_obstacle;
}

void GridWorld::clearObstacleCells(const std::vector<std::vector<int64_t>>& positions)
{
    for (const auto& pos : positions) {
        if (pos.size() < 3) continue;
        setObstacle(pos[0], pos[1], pos[2], false);
    }
    computeNeighbors();
}

int GridWorld::getNodeIndex(int x, int y, int z) const
{
    if (x < 0 || x >= grid_width_ || y < 0 || y >= grid_height_ || z < 0 || z >= grid_depth_)
    {
        return -1; // Out of bounds
    }
    return x * grid_height_ * grid_depth_ + y * grid_depth_ + z;
}

void GridWorld::computeNeighbors()
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
            int neighbor_index = getNodeIndex(nx, ny, nz);
            if (neighbor_index != -1 && !nodes_[neighbor_index].is_obstacle)
            {
                node.neighbors.push_back(neighbor_index);
            }
        }
    }
}

} // namespace create_gridworld
