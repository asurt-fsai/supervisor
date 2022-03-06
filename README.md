# Supervisor


## Manual Inspection

Inspect any ROS1 *pipeline* (i.e modules or packages); supposedly, the full system.

</br>

### How to use

1. Import the needed classes.

```python
from inspector import Inspector, Module
```

2. Add the modules to be inspected; the order does matter, for they are launched sequentially.

```python
inspector = Inspector([
    Module(pkg="mrpython_pcl", launch="lidar.launch"),
])
```

Moreover, you can add an *accessory* (e.g plotter, rviz, etc.)

```python
inspector = Inspector([
    Module(pkg="mrpython_pcl", launch="lidar.launch",
           accessory_pkg="[pkg]", accessory_launch="[launch-file]",),
])
```

3. Start inspection.

```python
inspector.manual_inspect()
```

4. Launch the supervisor

```zsh
roslaunch supervisor supervisor.launch
```

5. Inspect with actions.

![actions](/graphic/actions_terminal.png)