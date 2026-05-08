# Robot Navigation and Arm Control

## Overview
The goal of this workspace is to recreate a FetchReach scenario using a Turtlebot and a Kinova arm. The Turtlebot naviates to a location around a table where the Kinova arm is placed and the goal is for the arm to reach above where the Turtlebot has moved to. 

This workspace contains setup instructions for a system where a Turtlebot is sent to two goal locations based on a map of MULIP. It uses Nav2 to navigate to those locations. Once the Turtlebot arrives at a goal, it publishes to `/turtlebot_goal_index` to indicate which goal it reached. The Kinova arm is subscribed to this topic and uses MoveIt to move to the corresponding location of the Turtlebot.

1. [MoveIt](https://github.com/BurmanT/docker_ros_workspace/blob/main/ur_test/src/move_ur3.py): Subscribes to `/turtlebot_goal_index` and based on the goal that the Turtlebot has moved to, uses MoveIt to move arm to specific poses. 
2. [Nav2](https://github.com/BurmanT/docker_ros_workspace/blob/main/turtlebot_nav/tanu_workspace_april30/random_navigator.py): Uses `/next_goal` to send the Turtlebot to navigate to two goal locations in MULIP. Once it has successfully reached the location, publishes to `/turtlebot_goal_index` the specific index of the goal location it reached. 
3. Perception with [Lidar](https://github.com/BurmanT/docker_ros_workspace/blob/main/turtlebot_nav/tanu_workspace_april30/ArmProximity.py) on Turtlebot/ Custom Node: This script is not used in the demo but it is a custom ROS2 node that uses the LiDAR sensor on the Turtlebot to determine if there is an obstacle/arm in front of the Turtlebot. Ideally this would be used to confirm if the arm has reached above the Turtlebot once the Turtlebot arrives at a goal location. However, in the future, a force sensor may be better fit to confirm that the arm reaches above the Turtlebot since the Kinova arm is placed on a table and the Turtlebot is on the ground.  

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
