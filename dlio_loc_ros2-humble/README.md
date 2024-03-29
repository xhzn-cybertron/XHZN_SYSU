# DLIO(ICRA 2023)-based real-time mapping and localization.
Current development state: Continues-time INS odometry + LiDAR gobal localization
The approach can get a smooth localization with a MCU in a 100m*50m indoor factory environment.
Please note that the difference between two laser scan in state update is set to 0.05 cause the frequency of our LiDAR is set as 20Hz. If your LiDAR is 5Hz/10Hz or else, please check the variable dt in StateUpdate function. IMU is also... We further publish a custom param version.

## Has been worked on this environment
- Ubuntu 22.04 + ROS2 Humble
- g++ 11.2.0
- Eigen 3.4.0
- PCL 1.12.1

Dataset includes a .pcd map and a tiny ros2 bag including LiDAR data(20Hz) and inertial data(100Hz).

# TODO:
Construct a completed framework: DLIO-based odom and map-matching global localization
A stable buffer scheme for sensor data loop back.
Decrease the cpu load(Now avg load is about 30% on 8*2.60GHz cores). The sensor data will loop back and the positioning will be not stable when a new node starts and closes frequently.

# Problems:
iG-LIO reported that DLIO is not stable in dynamic environment and occupy more CPU usage. We will further check it in future.
 
----------------------------------------------------------------------------------------

# Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction

#### [[ IEEE ICRA ](https://ieeexplore.ieee.org/document/10160508)] [[ arXiv ](https://arxiv.org/abs/2203.03749)] [[ Video ](https://www.youtube.com/watch?v=4-oXjG8ow10)] [[ Presentation ](https://www.youtube.com/watch?v=Hmiw66KZ1tU)]

DLIO is a new lightweight LiDAR-inertial odometry algorithm with a novel coarse-to-fine approach in constructing continuous-time trajectories for precise motion correction. It features several algorithmic improvements over its predecessor, [DLO](https://github.com/vectr-ucla/direct_lidar_odometry), and was presented at the IEEE International Conference on Robotics and Automation (ICRA) in London, UK in 2023.

<br>
<p align='center'>
    <img src="./doc/img/dlio.png" alt="drawing" width="720"/>
</p>

## Instructions

### Sensor Setup
DLIO has been extensively tested using a variety of sensor configurations and currently supports Ouster, Velodyne, and Hesai LiDARs. The point cloud should be of input type `sensor_msgs::PointCloud2` and the 6-axis IMU input type of `sensor_msgs::Imu`.

For best performance, extrinsic calibration between the LiDAR/IMU sensors and the robot's center-of-gravity should be inputted into `cfg/dlio.yaml`. If the exact values of these are unavailable, a rough LiDAR-to-IMU extrinsics can also be used (note however that performance will be degraded).

IMU intrinsics are also necessary for best performance, and there are several open-source calibration tools to get these values. These values should also go into `cfg/dlio.yaml`. In practice however, if you are just testing this work, using the default ideal values and performing the initial calibration procedure should be fine.

Also note that the LiDAR and IMU sensors _need_ to be properly time-synchronized, otherwise DLIO will not work. We recommend using a LiDAR with an integrated IMU (such as an Ouster) for simplicity of extrinsics and synchronization.

### Dependencies
The following has been verified to be compatible, although other configurations may work too:

- Ubuntu 20.04
- ROS Noetic (`roscpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `pcl_ros`)
- C++ 14
- CMake >= `3.12.4`
- OpenMP >= `4.5`
- Point Cloud Library >= `1.10.0`
- Eigen >= `3.3.7`

```sh
sudo apt install libomp-dev libpcl-dev libeigen3-dev
```

DLIO currently only supports ROS1, but we welcome any contributions by the community to add ROS2 support!

### Compiling
Compile using the [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/) package via:

```sh
mkdir ws && cd ws && mkdir src && catkin init && cd src
git clone https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git
catkin build
```

### Execution
After compiling, source the workspace and execute via:

```sh
roslaunch direct_lidar_inertial_odometry dlio.launch \
  rviz:={true, false} \
  pointcloud_topic:=/robot/lidar \
  imu_topic:=/robot/imu
```

Be sure to change the topic names to your corresponding topics. Alternatively, edit the launch file directly if desired. If successful, you should see the following output in your terminal:
<br>
<p align='center'>
    <img src="./doc/img/terminal.png" alt="drawing" width="480"/>
</p>

### Services
To save DLIO's generated map into `.pcd` format, call the following service:

```sh
rosservice call /robot/dlio_map/save_pcd LEAF_SIZE SAVE_PATH
```

### Test Data
For your convenience, we provide test data [here](https://drive.google.com/file/d/1Sp_Mph4rekXKY2euxYxv6SD6WIzB-wVU/view?usp=sharing) (1.2GB, 1m 13s, Ouster OS1-32) of an aggressive motion to test our motion correction scheme, and [here](https://drive.google.com/file/d/1HbmF5gTHxCAMqBkEd5PTxDNQvcI8tKXn/view?usp=sharing) (16.5GB, 4m 21s, Ouster OSDome) of a longer trajectory outside with lots of trees. Try these two datasets with both deskewing on and off!

<br>
<p align='center'>
    <img src="./doc/gif/aggressive.gif" alt="drawing" width="720"/>
</p>

## Citation
If you found this work useful, please cite our manuscript:

```bibtex
@article{chen2022dlio,
  title={Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction},
  author={Chen, Kenny and Nemiroff, Ryan and Lopez, Brett T},
  journal={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2023},
  pages={3983-3989},
  doi={10.1109/ICRA48891.2023.10160508}
}
```

## Acknowledgements

We thank the authors of the [FastGICP](https://github.com/SMRT-AIST/fast_gicp) and [NanoFLANN](https://github.com/jlblancoc/nanoflann) open-source packages:

- Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno, “Voxelized GICP for Fast and Accurate 3D Point Cloud Registration,” in _IEEE International Conference on Robotics and Automation (ICRA)_, IEEE, 2021, pp. 11 054–11 059.
- Jose Luis Blanco and Pranjal Kumar Rai, “NanoFLANN: a C++ Header-Only Fork of FLANN, A Library for Nearest Neighbor (NN) with KD-Trees,” https://github.com/jlblancoc/nanoflann, 2014.

We would also like to thank Helene Levy and David Thorne for their help with data collection.

## License
This work is licensed under the terms of the MIT license.

<br>
<p align='center'>
    <img src="./doc/img/ucla.png" alt="drawing" width="720"/>
</p>
<p align='center'>
    <img src="./doc/img/trees.png" alt="drawing" width="720"/>
</p>
