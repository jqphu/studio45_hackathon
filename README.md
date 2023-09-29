# Studio 45 Hackathon!

## Getting Started

Everything will be based off Ubuntu 22.04.

### ROS2
1. Install [ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)
2. Add: `source /opt/ros/iron/setup.bash` to your `.bashrc` file
3. Check you can run `ros2`

### MoveIt
1. Install [Eclipse Cyclone DDS](https://docs.ros.org/en/iron/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html#install-packages) 
2. Add `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to `.bashrc`
3. Check you can run `ros2` in a new terminal
4. Install [MoveIt2](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

### General Dependencies

We use moveit visulization tools.
```
sudo apt install ros-iron-moveit-visual-tools
```

### XARM Dependencies
We've cloned xarm libraries as a submodule: https://github.com/xArm-Developer/xarm_ros2/tree/rolling

We need to install the dependencies.
Make sure your submodules are updated first and you're on the `rolling` branch.

1. cd dev_ws/src
2. rosdep update
3. rosdep install --from-paths . --ignore-src --rosdistro iron -y


Check you can build.
1. cd dev_ws
2. colcon build

Optional: Add `source <input path to repo>/dev_ws/install/setup.bash` to
`.bashrc`. If you don't add this, remember to source it everytime you need
to use ros.

### Setting Up The Arm
TODO: Need to configure network settings and connect to the local network of the arm. This step is not needed if you're simulating.

## Running

Make sure everything is sourced (ros, and the installation)

1. cd dev_ws
2. colcon build 
3. 

IMPORTANT: Gripper doesn't quite work right now for some reason. It just crashed the node.

```
# 【simulated Lite6】launch xarm_planner_node
ros2 launch xarm_planner lite6_planner_fake.launch.py [add_gripper:=true]
# 【real Lite6】launch xarm_planner_node
ros2 launch xarm_planner lite6_planner_realmove.launch.py robot_ip:=192.168.1.185 [add_gripper:=true]
```

This above command runs the ros2/moveit nodes for us to interact with the arm. Keep this running in a terminal.

4. Run a test (note if you ran realmove, the arm will move. Be careful!)
```
ros2 launch planner planner.launch.py dof:=6 robot_type:=lite
```

If you see the arm move, or the simulation move. You have set this up correctly!

## Developing

There's a basic planner package that you can investigate. Be sure to source
all the required dependencies.

1. cd dev_ws
2. colcon build --packages-up-to planner  

Build the planner package and all of its dependencies

3. ros2 launch planner planner.launch.py dof:=6 robot_type:=lite
