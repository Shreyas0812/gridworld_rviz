# Gridworld in RViz

Development in Progress

## TO Launch

`Location: gridworld_rviz`

```bash
ros2 launch create_gridworld create_gridworld.launch.py
```

### Launching with Different Configurations:

This is the default one:
```bash
ros2 launch create_gridworld your_launch_file.py config_type:=small_3D
```

```bash
ros2 launch create_gridworld your_launch_file.py config_type:=small_2D
```

```bash
ros2 launch create_gridworld your_launch_file.py config_type:=large_3D
```

```bash
ros2 launch create_gridworld your_launch_file.py config_type:=large_2D
```