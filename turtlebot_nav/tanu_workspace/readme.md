Files to have turtlebot move to random goal position near Kinova

Terminal 1:

export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
source ~/workspace/kobuki_tb2/install/setup.bash
ros2 launch kobuki_ros kobuki.launch.py

Terminal 2: 

export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash

# base_footprint → base_link
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_footprint base_link &

# base_link → laser
ros2 run tf2_ros static_transform_publisher \
  0 0 0.3 0 0 0 base_link laser &

Terminal 3 (Navigation):

export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=/home/bee-humble/tanu_workspace/table_map.yaml \
  params_file:=/home/bee-humble/tanu_workspace/nav2_params.yaml

Terminal 4 (cmd vel):

export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
python3 ~/tanu_workspace/cmd_vel_relay.py

Terminal 5: 
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
python3 ~/tanu_workspace/random_navigator.py

Terminal 6 (rviz):
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
rviz2

From display, add TF
From topic add /map and Transient Local 
From topic add /scan add LaserScan ==> make 0.1 size
Set 2D pose estimate


Terminal 7 (trigger):
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
ros2 service call /next_goal std_srvs/srv/Trigger {}





Turtlebot facing towards the kinova arm:
KINOVA_BASE_MAP_X = 0.1783
KINOVA_BASE_MAP_Y = -0.3429

header:
  stamp:
    sec: 1777486431
    nanosec: 46836792
  frame_id: map
pose:
  pose:
    position:
      x: 0.1782848926948161
      y: -0.3429071309093723
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.750996328558762
      w: 0.6603063792598554
  covariance:
  - 0.003382845779243762
  - 0.0008604348651057095
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0008604348651057303
  - 0.007335852457876754
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0067395175590804005
---


goal 1 location (x):

header:
  stamp:
    sec: 1777488270
    nanosec: 509032499
  frame_id: map
pose:
  pose:
    position:
      x: -0.5149344274238572
      y: 0.17220020798923333
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.050212099999552715
      w: 0.9987385769127148
  covariance:
  - 0.006797519610176439
  - 0.00272064942561126
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.00272064942561126
  - 0.004989184701614406
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.006298597880350169
---

x: -0.5149
y:  0.1722
orientation z: -0.0502
orientation w:  0.9987


goal 2 location (next to orange x)

header:
  stamp:
    sec: 1777488418
    nanosec: 231006841
  frame_id: map
pose:
  pose:
    position:
      x: -0.582567482687221
      y: -0.5846127767313181
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.4207014141839362
      w: 0.9071991623142275
  covariance:
  - 0.008497080679244262
  - 0.003147058068677766
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0031470580686777105
  - 0.004978926370489245
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.005402032757312128
---

x: -0.5826
y: -0.5846
orientation z:  0.4207
orientation w:  0.9072