# Gridworld Visualization in RViz2

A configurable 2D/3D grid environment for multi-agent path finding (MAPF) research, built with ROS2 and visualized in RViz2.

## Overview

This workspace contains two ROS2 packages:
- **`create_gridworld`** — Creates and visualizes dynamic grid worlds with obstacles, agents, and warehouse infrastructure (induct/eject/charging stations)
- **`trajectory_executor`** — Replays multi-agent trajectories from CSV files, driving agent positions in the visualizer

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

2. **Build both packages**:
   ```bash
   colcon build
   source install/setup.bash
   ```
   
### Launch

**Default Configuration (Small Warehouse 2D, 30×30)**:
```bash
ros2 launch create_gridworld create_gridworld.launch.py
```

**Specific Configurations**:
```bash
# Small Warehouse (30×30, 6 agents)
ros2 launch create_gridworld create_gridworld.launch.py config_type:=warehouse_small

# Large Warehouse (60×60, 12 agents)
ros2 launch create_gridworld create_gridworld.launch.py config_type:=warehouse_large

# Small 2D World (30×30, good for testing)
ros2 launch create_gridworld create_gridworld.launch.py config_type:=small_2D

# Small 3D World (20×20×5)
ros2 launch create_gridworld create_gridworld.launch.py config_type:=small_3D

# Large 2D Maze (150×150, complex navigation)
ros2 launch create_gridworld create_gridworld.launch.py config_type:=large_2D

# Large 3D Environment (100×100×20, multi-level)
ros2 launch create_gridworld create_gridworld.launch.py config_type:=large_3D
```

**Replay Trajectories**:
```bash
# Default: trajectories.csv at 1 Hz
ros2 launch trajectory_executor trajectory_executor.launch.py

# Custom file and rate
ros2 launch trajectory_executor trajectory_executor.launch.py data_file:=my_traj rate_hz:=5.0
```

## Configuration Options

| Configuration | Dimensions | Agents | Obstacles | Induct | Eject | Charging | Use Case | Publish Rate |
|---------------|------------|--------|-----------|--------|-------|----------|----------|--------------|
| `warehouse_small` | 30×30×1 | 6 | Walls + columns + bays | 8 | 38 | 6 | Small warehouse MAPF | 20 Hz |
| `warehouse_large` | 60×60×1 | 12 | Walls + columns + bays | 12 | 80 | 12 | Large warehouse MAPF | 10 Hz |
| `small_2D` | 30×30×1 | 5 | Borders + 4 internal | 4 | 4 | — | 2D algorithm testing | 20 Hz |
| `small_3D` | 20×20×5 | 3 | 3 block regions | 4 | 4 | — | 3D algorithm testing | 15 Hz |
| `large_2D` | 150×150×1 | 10 | Complex maze structure | 8 | 8 | — | Large-scale 2D navigation | 3 Hz |
| `large_3D` | 100×100×20 | 8 | Multi-level buildings | 10 | 10 | — | Large-scale 3D navigation | 5 Hz |

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
- **Magenta spheres** showing agent positions
- **White text labels** displaying "A_1", "A_2", etc. above each agent
- **Configurable**: Size, color, initial positions

### **Induct Stations**
- **Light blue cylinders** marking entry/pickup points
- **White text labels** displaying "IN_1", "IN_2", etc.
- **Configurable**: Size, color, positions

### **Eject Stations**
- **Orange cylinders** marking exit/dropoff points
- **White text labels** displaying "OUT_1", "OUT_2", etc.
- **Configurable**: Size, color, positions

### **Charging Stations**
- **Green-teal cylinders** marking energy recharge points
- **White text labels** displaying "CHG_1", "CHG_2", etc.
- **Configurable**: Size, color, positions

## Topics

### Published Topics (create_gridworld)
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/gridworld/nodes` | `visualization_msgs/MarkerArray` | Grid node visualization |
| `/gridworld/edges` | `visualization_msgs/MarkerArray` | Connectivity visualization |
| `/gridworld/obstacles` | `visualization_msgs/MarkerArray` | Obstacle visualization |
| `/gridworld/agents` | `visualization_msgs/MarkerArray` | Agent position visualization |
| `/gridworld/induct_stations` | `visualization_msgs/MarkerArray` | Induct station visualization |
| `/gridworld/eject_stations` | `visualization_msgs/MarkerArray` | Eject station visualization |
| `/gridworld/charging_stations` | `visualization_msgs/MarkerArray` | Charging station visualization |

### Subscribed Topics (create_gridworld)
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/agent_position` | `std_msgs/Int64MultiArray` | Individual agent position updates `[agent_id, x, y, z]` |

### Published Topics (trajectory_executor)
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/agent_position` | `std_msgs/Int64MultiArray` | Agent position at each timestep `[agent_id, x, y, z]` |

## Dynamic Agent Control

### Move Individual Agents

Update a specific agent's position in real-time:

```bash
# Move agent 1 to position (5, 8, 2)
ros2 topic pub /agent_position std_msgs/msg/Int64MultiArray "{data: [1, 5, 8, 2]}"

# Move agent 2 to position (10, 15, 1)
ros2 topic pub /agent_position std_msgs/msg/Int64MultiArray "{data: [2, 10, 15, 1]}"

# Add new agent 10 at position (7, 12, 0)
ros2 topic pub /agent_position std_msgs/msg/Int64MultiArray "{data: [10, 7, 12, 0]}"
```

## Trajectory Executor

The `trajectory_executor` package replays multi-agent trajectories from a single CSV file.

### CSV Format

```csv
agent_id,x,y,z,timestep
1,6,4,0,0
2,6,12,0,0
3,6,20,0,0
1,4,4,0,1
2,5,12,0,1
3,5,20,0,1
```

- **`agent_id`**: Matches the agent IDs defined in the gridworld config
- **`x, y, z`**: Grid cell coordinates
- **`timestep`**: Controls simulation ordering (all rows with the same timestep are published together)

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `data_file` | `trajectories` | CSV filename without `.csv` extension (from `data/` dir) |
| `rate_hz` | `1.0` | Playback rate in Hz |

## Custom Configuration

### Creating New Configurations

1. **Create a new YAML file** in `create_gridworld/config/`:
   ```bash
   cp config/gridworld_warehouse_small.yaml config/gridworld_custom.yaml
   ```

2. **Edit the parameters**:
   ```yaml
   create_gridworld_node:
     ros__parameters:
       grid_width: 50
       grid_height: 50
       grid_depth: 1
       grid_resolution: 1.0
       publish_rate: 20.0
       
       # Flattened obstacle regions: [start_x, start_y, start_z, end_x, end_y, end_z, ...]
       obstacle_regions: [10, 10, 0, 15, 15, 0, 20, 20, 0, 25, 25, 0]
       
       # Agent positions: [x, y, z, agent_id, ...]
       agent_positions: [5, 5, 0, 1, 45, 45, 0, 2, 25, 25, 0, 3]
       
       # Induct stations: [x, y, z, station_id, ...]
       induct_stations: [2, 2, 0, 1, 48, 48, 0, 2]
       
       # Eject stations: [x, y, z, station_id, ...]
       eject_stations: [2, 48, 0, 1, 48, 2, 0, 2]
       
       # Charging stations: [x, y, z, station_id, ...]
       charging_stations: [25, 2, 0, 1, 25, 48, 0, 2]
       
       # Visualization settings
       visualization:
         node_scale: 0.15
         node_color: [0.0, 1.0, 0.0, 0.7]  # [R, G, B, Alpha]
         edge_width: 0.05
         edge_color: [0.0, 0.0, 1.0, 0.6]
         obstacle_scale: 0.95
         obstacle_color: [1.0, 0.0, 0.0, 0.9]
         agent_scale: 0.4
         agent_color: [1.0, 0.0, 1.0, 1.0]
         induct_station_scale: 1.0
         induct_station_color: [0.0, 0.5, 1.0, 1.0]
         eject_station_scale: 0.8
         eject_station_color: [1.0, 0.5, 0.0, 1.0]
         charging_station_scale: 0.9
         charging_station_color: [0.0, 1.0, 0.5, 1.0]
       
       frame_id: "map"
       
       topics:
         nodes: "gridworld/nodes"
         edges: "gridworld/edges"
         obstacles: "gridworld/obstacles"
         agents: "gridworld/agents"
         induct_stations: "gridworld/induct_stations"
         eject_stations: "gridworld/eject_stations"
         charging_stations: "gridworld/charging_stations"
   ```

3. **Launch with your configuration**:
   ```bash
   ros2 launch create_gridworld create_gridworld.launch.py config_type:=custom
   ```

### Parameter Descriptions

- **`grid_width/height/depth`**: Grid dimensions in cells
- **`grid_resolution`**: Size of each cell in meters
- **`publish_rate`**: Visualization update frequency in Hz
- **`obstacle_regions`**: Flattened array of obstacle boxes `[x1,y1,z1,x2,y2,z2,...]`
- **`agent_positions`**: Flattened array of agent positions `[x,y,z,agent_id,...]`
- **`induct_stations`**: Flattened array of entry stations `[x,y,z,station_id,...]`
- **`eject_stations`**: Flattened array of exit stations `[x,y,z,station_id,...]`
- **`charging_stations`**: Flattened array of charging docks `[x,y,z,station_id,...]`
- **`visualization.*`**: Color (RGBA), scale, and size parameters per element type
- **`topics.*`**: Custom topic names for each visualization element

## Package Structure

```
gridworld_rviz/
├── create_gridworld/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   ├── gridworld_warehouse_small.yaml
│   │   ├── gridworld_warehouse_large.yaml
│   │   ├── gridworld_small_2D.yaml
│   │   ├── gridworld_small_3D.yaml
│   │   ├── gridworld_large_2D.yaml
│   │   └── gridworld_large_3D.yaml
│   ├── include/create_gridworld/
│   │   ├── config_manager.hpp
│   │   ├── create_gridworld_node.hpp
│   │   ├── grid_world.hpp
│   │   └── visualization_manager.hpp
│   ├── src/
│   │   ├── config_manager.cpp
│   │   ├── create_gridworld_node.cpp
│   │   ├── grid_world.cpp
│   │   └── visualization_manager.cpp
│   ├── launch/
│   │   └── create_gridworld.launch.py
│   └── rviz/
│       └── gridworld_config.rviz
├── trajectory_executor/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── src/
│   │   └── trajectory_executor.cpp
│   ├── launch/
│   │   └── trajectory_executor.launch.py
│   └── data/
│       ├── trajectories.csv
│       └── warehouse_traj_generator.py
└── README.md
```

## License

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](create_gridworld/LICENSE)