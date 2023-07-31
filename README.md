# LIO-2-PX4
ROS package/node for relaying LIDAR odometry from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) to [PX4 Autopilot](https://px4.io/).

<a name="readme-top"></a>


<!-- ABOUT THE PROJECT -->
## About The Project
This is a proof of concept showing that interfacing [LIDAR](https://en.wikipedia.org/wiki/Lidar) with low-level flight flight control is possible.
Typical flight controllers such as [Pixhawk](https://pixhawk.org/) or [Cube Orange](https://www.cubepilot.com/#/home)
only have accelerometers, gyroscopes, magnetometers and barometers as onboard sensors.
When flying autonomously, these sensors are not enough to accurately estimate position, velocity and
attitude. TODO

### Further reading
Article
[PX4 documentation on external odometry](https://docs.px4.io/main/en/ros/external_position_estimation.html)

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
   ```
   EKF2_AID_MASK
   ```
   to **vision position fusion** and **vision yaw fusion**, and parameter
      ```
   SENS_BOARD_ROT
   ```
   to **Yaw 90 deg**. Reboot the flight controller.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Run the package
  Simply run the Python script:
   ```sh
   rosrun lio_2_px4 lio_2_px4.py
   ```

<!-- CONTACT -->
## Contact

Jacob Törngren - [Linkedin](https://linkedin.com/in/jacobtorngren) - jacob.torngren@hotmail.com

Project Link: [https://github.com/jacobtorngr/LIO-2-PX4](https://github.com/jacobtorngr/LIO-2-PX4)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


