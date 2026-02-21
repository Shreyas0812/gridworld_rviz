// include/create_gridworld/grid_world.hpp
#pragma once

#include <vector>
#include <tuple>
#include <cstdint>

namespace create_gridworld {

struct GridNode {
    int x, y, z;
    bool is_obstacle;
    bool is_traversable;
    std::vector<int> neighbors;
    
    GridNode(int x_, int y_, int z_);
};

class GridWorld {
public:
    GridWorld(int width, int height, int depth, double resolution);
    
    void addObstacleRegions(const std::vector<std::vector<int64_t>>& regions);
    void clearObstacleCells(const std::vector<std::vector<int64_t>>& positions);
    void setObstacle(int x, int y, int z, bool is_obstacle);
    int getNodeIndex(int x, int y, int z) const;
    const std::vector<GridNode>& getNodes() const { return nodes_; }
    
    // Grid properties
    int getWidth() const { return grid_width_; }
    int getHeight() const { return grid_height_; }
    int getDepth() const { return grid_depth_; }
    double getResolution() const { return grid_resolution_; }

private:
    void initializeGrid();
    void computeNeighbors();
    
    int grid_width_, grid_height_, grid_depth_;
    double grid_resolution_;
    std::vector<GridNode> nodes_;
};

} // namespace create_gridworld