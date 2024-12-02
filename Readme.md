# BasicSim

A minimal ROS2 robotic simulator, with emphasis on **minimal**. Meant to replace [Stage](https://github.com/rtv/Stage) for applications where you only need extremely basic 2D movement (since Stage in not supported in ROS2).

Does:

- Publishes `/clock`, with the ability to speed up or slow down time.
- Publishes a static `map -> odom` TF for when you don't need to simulate actual localization.
- Publishes `odom -> (robot)_base_link` TF for each robot.
- Publishes `/(robot)/odom` topic with ground truth pose and velocity.
- Listens to `/(robot)/cmd_vel` and applies the velocity commands to move the robot.
- Reads ros2 OccupancyGrid files (same format as map_server) and keeps the robot outside of walls.
- Keeps track of multiple robots in the same simulation session.
- Simulates 2D laser scanners and publishes to `/(robot)/(laser_name)`. You can have multiple ones on the same robot!

Does not do:
- Physics.
- Velocity or acceleration limits.
- Any sensors other than 2D laser scanners.
- Robot shapes (always assumed to be a circle of given radius, defaulting to 0). 
- Robot-robot collision.

## Installation
Simply clone inside or your colcon workspace's source directory and run `colcon build --symlink-install`.

⚠️ This repo has sub-modules! Remember to clone with the `--recurse-submodules` option, or to run `git submodule update --init --recursive`.

## Configuring a simulation

The entire configuration consist on creating a `.yaml` file. For example:

```yaml
map: "occupancy.yaml"
robots:
  - name: "giraff1"
    position: [2, 3, 0]
    angle: 1
    radius: 0.25
    sensors:
      - type: "laser"
        name: "laser_scanner" 
        minAngleRad: -1.9
        maxAngleRad: 1.9
        angleResolutionRad: 0.01 
        minDistance: 0.1
        maxDistance: 4.0

  - name: "giraff2"
    position: [3, 6, 0]
    angle: 2
```

The `map` field holds the path to a ROS-format occupancy map yaml file (such as the ones generated by `map_server`).

`robots` is a list, where each entry defines a separate robotic agent. Inside of each robot's entry, you must specify a name and starting pose. Optionally, you can also attach to each robot a list of sensors. Currently, the only supported sensor type is 2D laser scanners, but who knows what the future might hold.

## Node parameters
Once the simulation is configured with a `yaml` file, you can launch the simulator with the following parameters:

```python
basic_sim = Node(
            package="basic_sim",
            executable="basic_sim",
            parameters=[
                {"deltaTime": 0.1},
                {"speed": 5.0},
                {"worldFile": "[path to file]/simulation.yaml"}
            ],
        )
```
- `deltaTime` is the length of each simulation step (in simulation time). That is, for `deltaTime = 0.1s`, if a robot is told to move at `1m/s`, in one simulation step it will move `0.1m`, independently of how long that simulation step takes to run in real time.

- `speed` is how fast the simulator tries to run, compared to real time. `speed = 1` means one simulation step happens every `deltaTime` seconds. `speed = 10` means it will run 10 faster than real-time. Obviously, at some point processing power becomes a limiting factor (although basicSim is pretty lightweight). 

- `worldfile` is the path to the `yaml` file we looked at in the previous section.
