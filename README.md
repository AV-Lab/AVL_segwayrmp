# SegwayRMP

## Demo Video

https://github.com/user-attachments/assets/53a53dc9-f08f-44e8-a55d-37d1a7674863

## Package Overview
This repository provides a ROS interface to **Segway Robotics Mobility Platforms (RMPs)**. It consists of several packages:

- **rmp_base** – controls the base and publishes sensor data and status information  
- **rmp_teleop** – joystick interface for controlling the base  
- **rmp_description** – URDF description of the platform(s)  
- **rmp_msgs** – custom messages for RMP communication  

---

## Clone the repo
```
git clone https://github.com/AV-Lab/AVL_segwayrmp/
```

**Install dependencies**
```
rosdep install --from-paths src --ignore-src -r -y
```

**Build & source the workspace**
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Launch the vehicle

**Communication in the vehicle**
```
roslaunch rmp_base rmp440le.launch 
```

**cmd value smoothers**
```
roslaunch cmd_vel_smoother cmd_vel_smoother.launch 
#roslaunch yocs_velocity_smoother standalone.launch # testing this smoother
```

**Control with joystick**
```
roslaunch rmp_teleop joystick.launch 
```

**Launch the sensors (Camera, 2D lidar, 3D lidar)**
```
roslaunch segway_sensors zed2.launch
roslaunch segway_sensors urg_lidar.launch 
roslaunch segway_sensors VLP16_points.launch
```

## Navigation

**launch amcl and move base**
```
roslaunch segway_navigation navigation.launch 
```

**global cost map server**
```
roslaunch segway_navigation start_map_server.launch
```

**Launch gmapping if needed**
```
roslaunch segway_navigation gmapping_demo.launch
```

## Services

**Run services**
```
python run_services.py 1 0 0
```

**Unloader service**
```
python unloader_service.py 
```

**Vocaliser service**
```
python vocaliser_service.py 
```

## Additional Useful commands

**rosbag record**
```
rosbag record /velodyne_points /zed2/zed_node/right/image_rect_color /zed2/zed_node/imu/data
```

**save map**
```
rosrun map_server map_saver -f mymap
```

**view frames**
```
rosrun tf view_frames
or
rosrun tf2_tools view_frames.py
```

**usb perm**
```
sudo chmod a+rw /dev/ttyACM0 
```

**echo clicked points**

```
rostopic echo /clicked_point
```
