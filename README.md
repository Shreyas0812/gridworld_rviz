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

## Configuration Options

| Configuration | Dimensions | Agents | Obstacles | Use Case | Publish Rate |
|---------------|------------|---------|-----------|----------|--------------|
| `small_2D` | 30×30×1 | 5 | Borders + 4 internal | 2D algorithm testing | 20 Hz |
| `small_3D` | 20×20×5 | 3 | 3 block regions | 3D algorithm testing | 15 Hz |
| `large_2D` | 150×150×1 | 10 | Complex maze structure | Large-scale 2D navigation | 3 Hz |
| `large_3D` | 100×100×20 | 8 | Multi-level buildings | Large-scale 3D navigation | 5 Hz |

## Visualization Elements

### **Grid Nodes**
- **Green points** representing traversable positions
- **Configurable**: Size, color, transparency

### **Grid Edges** 
- **Blue lines** showing connectivity between nodes
- **Configurable**: Width, color, transparency

### **Obstacles**
- **Red cubes** marking non-traversable areas
- **Configurable**: Size, color, regions

### **Agents**
- **Magenta circles** (2D) showing agent positions
- **Configurable**: Size, color, initial positions

## Topics Published

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/gridworld/nodes` | `visualization_msgs/MarkerArray` | Grid node visualization |
| `/gridworld/edges` | `visualization_msgs/MarkerArray` | Connectivity visualization |
| `/gridworld/obstacles` | `visualization_msgs/MarkerArray` | Obstacle visualization |
| `/gridworld/agents` | `visualization_msgs/MarkerArray` | Agent position visualization |

## Custom Configuration

### Creating New Configurations

1. **Create a new YAML file** in `config/` directory:
   ```bash
   cp config/gridworld_small_3D.yaml config/gridworld_custom.yaml
   ```

2. **Edit the parameters**:
   ```yaml
   create_gridworld_node:
     ros__parameters:
       grid_width: 50
       grid_height: 50
       grid_depth: 10
       
       # Flattened obstacle regions: [start_x, start_y, start_z, end_x, end_y, end_z, ...]
       obstacle_regions: [10, 10, 0, 15, 15, 5, 20, 20, 2, 25, 25, 8]
       
       # Agent positions: [x, y, z, x, y, z, ...]
       agent_positions: [5, 5, 0, 45, 45, 0, 25, 25, 5]
       
       # Visualization settings
       visualization:
         node_scale: 0.15
         node_color: [0.0, 1.0, 0.0, 0.7]  # [R, G, B, Alpha]
   ```

3. **Launch with your configuration**:
   ```bash
   ros2 launch create_gridworld create_gridworld.launch.py config_type:=custom
   ```

### Parameter Descriptions

- **`grid_width/height/depth`**: Grid dimensions in cells
- **`grid_resolution`**: Size of each cell in meters
- **`publish_rate`**: Update frequency in Hz
- **`obstacle_regions`**: Flattened array of obstacle boxes `[x1,y1,z1,x2,y2,z2,...]`
- **`agent_positions`**: Flattened array of agent positions `[x1,y1,z1,x2,y2,z2,...]`
- **`visualization.*`**: Color (RGBA), scale, and size parameters

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

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](create_gridworld/LICENSE)


