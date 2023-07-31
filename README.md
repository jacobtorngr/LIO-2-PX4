# LIO-2-PX4
ROS package/node for relaying lidar odometry from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) to [PX4 Autopilot](https://px4.io/).

<a name="readme-top"></a>


<!-- ABOUT THE PROJECT -->
## About The Project
This is a proof of concept to show that interfacing LIDAR with PX4 is possible.
Typical flight controllers such as [Pixhawk](https://pixhawk.org/) or [Cube Orange](https://www.cubepilot.com/#/home)
only have accelerometers, gyroscopes, magnetometers and barometers as onboard sensors.
When flying autonomously, these sensors are not enough to accurately estimate position, velocity and
attitude. 
LIO-2-PX4 uses PX4's external odometry pipeline to stream the fused LIDAR and IMU Odometry
messages 

## Further reading
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* npm
  ```sh
  npm install npm@latest -g
  ```

### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
   ```sh
   git clone https://github.com/jacobtorngr/LIO-2-PX4.git
   ```
3. Install NPM packages
   ```sh
   npm install
   ```
4. Enter your API in `config.js`
   ```js
   const API_KEY = 'ENTER YOUR API';
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Jacob Törngren - [Linkedin](https://linkedin.com/in/jacobtorngren) - jacob.torngren@hotmail.com

Project Link: [https://github.com/jacobtorngr/LIO-2-PX4](https://github.com/jacobtorngr/LIO-2-PX4)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


