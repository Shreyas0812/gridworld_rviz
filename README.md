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

# Large Warehouse (60×60, 18 agents)
ros2 launch create_gridworld create_gridworld.launch.py config_type:=warehouse_large

# Kiva Pod Storage — medium scale (36×36, 100 agents) — Wurman et al. 2008
ros2 launch create_gridworld create_gridworld.launch.py config_type:=kiva

# Kiva Pod Storage — large scale (72×72, 200 agents) — Wurman et al. 2008
ros2 launch create_gridworld create_gridworld.launch.py config_type:=kiva_large

# MAPF Warehouse Benchmark (161×63, 470 agents) — Li et al. 2021 / Sturtevant 2012
ros2 launch create_gridworld create_gridworld.launch.py config_type:=shelf_aisle

# Cross-Dock Sortation Center (44×28, 50 agents) — Boysen et al. 2019
ros2 launch create_gridworld create_gridworld.launch.py config_type:=crossdock

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
| `warehouse_small` | 30×30×1 | 6 | Walls + columns + bays | 8 | 38 | 6 | Small warehouse MAPF (baseline) | 20 Hz |
| `warehouse_large` | 60×60×1 | 18 | Walls + columns + bays | 12 | 80 | 12 | Large warehouse MAPF (scalability) | 10 Hz |
| `kiva` | 36×36×1 | 100 | Pod matrix + maintenance bay | 5 | 10 | 12 | Kiva-style pod storage, medium scale [[1]](#references) | 20 Hz |
| `kiva_large` | 72×72×1 | 200 | Scaled pod matrix + maintenance bay | 5 | 10 | 12 | Kiva-style pod storage, large scale [[1]](#references) | 10 Hz |
| `shelf_aisle` | 161×63×1 | 470 | 10 shelf rows × 5 segments + vertical aisles | 10 | 12 | 18 | MAPF benchmark `warehouse-10-20-10-2-1` [[2,3]](#references) | 5 Hz |
| `crossdock` | 44×28×1 | 50 | Lane dividers + charging corner | 6 | 10 | 8 | Cross-dock sortation center [[4,5]](#references) | 20 Hz |
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

## References

The three literature-based gridworld configurations are derived from the following works:

[1] **Kiva Pod Storage** (`kiva`):
Wurman, P.R., D'Andrea, R., & Mountz, M. (2008). "Coordinating Hundreds of Cooperative, Autonomous Vehicles in Warehouses." *AI Magazine*, 29(1), 9–19.
https://doi.org/10.1609/aimag.v29i1.2082

[2] **MAPF Warehouse Benchmark — layout** (`shelf_aisle`):
Li, J., Tinka, A., Kiesel, S., Durham, J.W., Kumar, T.K.S., & Koenig, S. (2021). "Lifelong Multi-Agent Path Finding in Large-Scale Warehouses." *Proceedings of the 35th AAAI Conference on Artificial Intelligence (AAAI 2021)*.
https://doi.org/10.1609/aaai.v35i13.17344

[3] **MAPF Warehouse Benchmark — map suite** (`shelf_aisle`):
Sturtevant, N.R. (2012). "Benchmarks for Grid-Based Pathfinding." *IEEE Transactions on Computational Intelligence and AI in Games*, 4(2), 144–148.
https://doi.org/10.1109/TCIAIG.2012.2197499

[4] **Cross-Dock Sortation Center — survey** (`crossdock`):
Boysen, N., De Koster, R., & Weidinger, F. (2019). "Warehousing in the e-commerce era: A survey." *European Journal of Operational Research*, 277(2), 396–411.
https://doi.org/10.1016/j.ejor.2018.08.023

[5] **Cross-Dock Sortation Center — robotic fulfillment** (`crossdock`):
Boysen, N., Briskorn, D., & Emde, S. (2017). "Parts-to-picker based order processing in a rack-moving mobile robots environment." *European Journal of Operational Research*, 262(2), 550–562.
https://doi.org/10.1016/j.ejor.2017.03.053

## Package Structure

```
gridworld_rviz/
├── create_gridworld/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   ├── gridworld_warehouse_small.yaml    # Baseline warehouse (30×30)
│   │   ├── gridworld_warehouse_large.yaml    # Scalability warehouse (60×60)
│   │   ├── gridworld_kiva.yaml               # Kiva pod storage, 36×36, 100 agents [1]
│   │   ├── gridworld_kiva_large.yaml         # Kiva pod storage, 72×72, 200 agents [1]
│   │   ├── gridworld_shelf_aisle.yaml        # MAPF benchmark warehouse-10-20-10-2-1, 161×63, 470 agents [2,3]
│   │   ├── gridworld_crossdock.yaml          # Cross-dock sortation center, 44×28, 50 agents [4,5]
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