# Gridworld Visualization in RViz2

A configurable 2D/3D grid environment for multi-agent path finding (MAPF) research, built with ROS2 and visualized in RViz2.

## Overview

This package creates dynamic grid worlds with configurable dimensions, obstacles, and agents.

## Quick Start

### Prerequisites

- ROS2 Humble or later
- RViz2
- C++17 compiler


### Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/Shreyas0812/gridworld_rviz.git
   ```

2. **Build the package**:
   ```bash
   colcon build --packages-select create_gridworld
   source install/setup.bash
   ```
   
### Launch

**Default Configuration (Small 2D)**:
```bash
ros2 launch create_gridworld create_gridworld.launch.py
```

**Specific Configurations**:
```bash
# Small 2D World (30×30, good for testing)
ros2 launch create_gridworld create_gridworld.launch.py config_type:=small_2D

# Small 3D World (20×20×5, default)
ros2 launch create_gridworld create_gridworld.launch.py config_type:=small_3D

# Large 2D Maze (150×150, complex navigation)
ros2 launch create_gridworld create_gridworld.launch.py config_type:=large_2D

# Large 3D Environment (100×100×20, multi-level)
ros2 launch create_gridworld create_gridworld.launch.py config_type:=large_3D
```

## Visualization Elements

### 🟢 **Grid Nodes**
- **Green points** representing traversable positions
- **Configurable**: Size, color, transparency

### 🔵 **Grid Edges** 
- **Blue lines** showing connectivity between nodes
- **Configurable**: Width, color, transparency

### 🔴 **Obstacles**
- **Red cubes** marking non-traversable areas
- **Configurable**: Size, color, regions

### 🔵 **Agents**
- **Blue spheres** (3D) or **Magenta circles** (2D) showing agent positions
- **Configurable**: Size, color, initial positions


## Topics Published

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/gridworld/nodes` | `visualization_msgs/MarkerArray` | Grid node visualization |
| `/gridworld/edges` | `visualization_msgs/MarkerArray` | Connectivity visualization |
| `/gridworld/obstacles` | `visualization_msgs/MarkerArray` | Obstacle visualization |
| `/gridworld/agents` | `visualization_msgs/MarkerArray` | Agent position visualization |

## Package Structure

```
create_gridworld/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│   └── create_gridworld_node.cpp
├── config/
│   ├── gridworld_small_2D.yaml
│   ├── gridworld_small_3D.yaml
│   ├── gridworld_large_2D.yaml
│   └── gridworld_large_3D.yaml
├── rviz/
│   └── gridworld.rviz
└── launch/
    └── create_gridworld.launch.py
```

## License

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)


