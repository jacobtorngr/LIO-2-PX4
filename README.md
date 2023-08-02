# LIO-2-PX4
ROS package/node for relaying LIDAR odometry from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) to [PX4 Autopilot](https://px4.io/).

<a name="readme-top"></a>


<!-- ABOUT THE PROJECT -->
## About The Project
This is a proof of concept showing that interfacing [LIDAR](https://en.wikipedia.org/wiki/Lidar) with low-level flight flight control is possible. It uses PX4's
[external vision pipeline](https://docs.px4.io/main/en/ros/external_position_estimation.html).
The ```lio_2_px4_node``` node subscribes to LIDAR intertial odometry (LIO, fused IMU and LIDAR data) from the LIO-SAM package on ROS topic ```odometry/imu```, downsamples it to 50Hz 
and publishes it on ```mavros/odometry/out```, where the [MAVROS Odometry plugin](http://docs.ros.org/en/noetic/api/mavros_extras/html/odom_8cpp_source.html)
in theory should handle all coordinate transformations. To get it to work, some adjustments to the coordinate frames are made as can be seen in the installation guide below.

Why LIO and not LIDAR? When PX4 fuses LIO, and LIO-SAM uses the flight controller's internal IMU:s, the IMU data is fused twice. Once by LIO-SAM and then by PX4. 
The reason is that the pure LIDAR odometry is too slow (5Hz on the development setup), while LIO is at 200Hz. PX4 requires 30-50Hz to fuse external odometry. This "double-fusion" has not 
been shown to be a problem in the testing (see <em>Examples</em> below or the article). It seems to work but requires further testing.

### Further reading
* Article
* [PX4 documentation on external odometry](https://docs.px4.io/main/en/ros/external_position_estimation.html)


<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

### Prerequisites 
* [Pixhawk](https://pixhawk.org/) or other flight controller board running [PX4](https://px4.io/)
* LIDAR equipment (developed with [OUSTER](https://ouster.com/) LIDAR)
* [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
* [ROS Noetic](https://docs.px4.io/main/en/ros/mavros_installation.html)
* [MAVROS](https://docs.px4.io/main/en/ros/mavros_installation.html)
* [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
* [QGroundControl](http://qgroundcontrol.com/)

### Installation
1. Clone the repo in your catkin workspace:
   ```sh
   cd <YOUR CATKIN WORKSPACE>/src
   git clone https://github.com/jacobtorngr/LIO-2-PX4.git
   ```
2. Modify px4.launch inside MAVROS (or copy it to create a new launch file):
   ```sh
   roscd mavros
   cd launch
   gedit px4.launch
   ```
   Insert the following line:
   ```xml
   <node pkg="tf" type="static_transform_publisher" name="tf_baseLink_externalPoseChildFrame" args="0 0 0 -1.57079632679 0 0 base_link base_link_px4 1000"/>
   ```
3. Modify px4_config.yaml in the same directory (or copy it to make a new one):
   ```sh
   gedit px4_config.yaml
   ```
   Find the <em>odometry</em> section and chage the following line:
   ```yaml
   odom_child_id_des: "base_link"
   ```
   into:
   ```yaml
   odom_child_id_des: "base_link_px4"
   ```
4. Go back to your catkin workspace and build it:
   ```sh
   cd ~/<YOUR CATKIN WORKSPACE>
   catkin build
   ```
5. Source it:
   ```sh
   source devel/setup.bash
   ```
6. Open QGroundControl and set parameter
   ```SENS_BOARD_ROT```
   to **Yaw 90 deg**.
   The following parameter settings are optional but recommended. Set
   ```EKF2_EV_MASK```
   to **vision position fusion** and **vision yaw fusion** (```EKF2_AID_MASK``` in PX4 v1.13 and earlier). Set
   ```EKF2_MAG_TYPE```
   to **5** (magnetometer disabled).
   
8. Reboot the flight controller.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Run the package
  Simply run the Python script:
   ```sh
   rosrun lio_2_px4 lio_2_px4.py
   ```
<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Tuning
The main tuning parameters are
```EKF2_EV_POS_<X,Y,Z>```
denoting the offset between the PX4 body frame and external odometry frame of reference, and
```EKF2_EV_DELAY```
setting the time delay between the external odometry relative to the internal IMU:s. Further
instructions on how to tune can be found in the [PX4 EV and MoCap guide](https://docs.px4.io/main/en/ros/external_position_estimation.html).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Examples
[This Google Drive folder](https://drive.google.com/drive/folders/1MJeX_GaXWPaaHPx7-lXcfXu0nH8P8apE?usp=sharing) contains
rosbags and px4 logs comparing different configurations, with and without this package. 

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Jacob Törngren - [Linkedin](https://linkedin.com/in/jacobtorngren) - jacob.torngren@hotmail.com

Project Link: [https://github.com/jacobtorngr/LIO-2-PX4](https://github.com/jacobtorngr/LIO-2-PX4)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


