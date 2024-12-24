# Docker Setup for ROS 2 Humble and Robot Operations

## Docker Commands

### Pull ROS 2 Humble Desktop Image
```bash
docker pull ghcr.io/sloretz/ros:humble-desktop
```

### Build Custom Docker Image
```bash
docker build -t ros2_humble_custom .
```

### Run Docker Container
```bash
docker run -it \
    --volume /home/ntlong/har_capstone/turtlebot3_ws/:/root/ros2_ws \
    --network host \
    --device=/dev/ttyACM0 \
    --device=/dev/ttyUSB0 \
    --rm ros2_humble_custom
```

### Access Running Container
```bash
docker exec -it <container_id> bash
```
Replace `<container_id>` with the ID of the running container.

---

## Robot Bringup and Operation

### Launch Robot Bringup
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

### Teleoperate the Robot
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## Flash Firmware for OpenCR

### Flash OpenCR Firmware
```bash
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```
Replace `$OPENCR_PORT` and `$OPENCR_MODEL` with the appropriate port and model information.

---

## SLAM and Mapping

### Run SLAM Toolbox
```bash
ros2 launch slam_toolbox online_async_launch.py
```

### Perform Mapping with Cartographer
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

### Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f <map_name>
```
Replace `<map_name>` with the desired name for your map file.

---

## LiDAR Setup

### Launch LiDAR Node
```bash
cd /home/ntlong/har_capstone/lidar_ws && source install/setup.bash
ros2 launch ldlidar_stl_ros2 ld19.launch.py
```

---

## Navigation

### Launch Navigation Stack
```bash
ros2 launch turtlebot3_navigation2 navigation2_launch.py map:=<map_name>.yaml
```
Replace `<map_name>` with the name of your saved map file.

---

## Monitor UI

### Launch ROS Bridge Server
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Start Monitor UI
```bash
cd /home/ntlong/har_capstone/monitor-ui && npm start
```

---

## Camera Setup

### Run Camera in TensorFlow Virtual Environment
```bash
source tf/bin/activate
```

