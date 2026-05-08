# Robot Navigation and Arm Control

This workspace contains setup instructions for a system where a Turtlebot is sent to two goal locations based on a map of MULIP. It uses Nav2 to navigate to those locations. Once the Turtlebot arrives at a goal, it publishes to `/turtlebot_goal_index` to indicate which goal it reached. The Kinova arm is subscribed to this topic and uses MoveIt to move to the corresponding location of the Turtlebot.

1. [MoveIt](https://github.com/BurmanT/docker_ros_workspace/blob/main/ur_test/src/move_ur3.py)
2. [Nav2](https://github.com/BurmanT/docker_ros_workspace/blob/main/turtlebot_nav/tanu_workspace_april30/random_navigator.py)

# Kinova Arm

1. Set up connection to Kinova: [ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex)
2. Can use Dockerfile here to set up container: [docker_robots](https://github.com/tufts-ai-robotics-group/docker_robots/tree/main/kinova_kortex)

**In one terminal:**
```bash
xhost +; docker run -it --rm \
  -v ~/docker_ros_workspace:/root/workspace/ros2_kortex_ws/src/tanu_project \
  --device=/dev/bus/usb/ \
  --device=/dev/input \
  --net=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="LIBGL_ALWAYS_SOFTWARE=true" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  kinova:ros2 /bin/bash

export ROS_DOMAIN_ID=5
colcon build --packages-select ur_test --parallel-workers 2
source install/setup.bash
ros2 launch ur_test move_ur3_launch.launch.py use_fake_hardware:=false
```

**Second terminal:**
```bash
docker ps 
docker exec -it <name> 
export ROS_DOMAIN_ID=5
colcon build --packages-select ur_test --parallel-workers 2
source install/setup.bash
ros2 topic echo /turtlebot_goal_index std_msgs/msg/Int32
```

# Turtlebot
**Terminal 1:Set up TF frame**
```bash
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint world &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0.3 0 0 0 base_link laser &
```
**Terminal 2: Navigation**
```bash
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup bringup_launch.py
map:=/home/bee-humble/tanu_workspace/table_map.yaml
params_file:=/home/bee-humble/tanu_workspace/nav2_params.yaml
```
**Terminal 3:Set up cmd_velocity node**
```bash
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
python3 ~/tanu_workspace/cmd_vel_relay.py
```

**Terminal 4:Goal navigation**
```bash
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
python3 ~/tanu_workspace/random_navigator.py
```

**Terminal 5:Rviz**
- From display add TF
- From topic add /map and Transient Local
- From topic add /scan add LaserScan
  - Make 0.1 size
  - Set 2D pose estimate
```bash
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
rviz2
```

**Terminal 6:Check what /turtlebot_goal_index is publishing**
```bash
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
ros2 topic echo /turtlebot_goal_index std_msgs/msg/Int32
```

**Terminal 7:Trigger to send Turtlebot to Goal Location**
```bash
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
ros2 service call /next_goal std_srvs/srv/Trigger {}
```
