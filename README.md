# Cooperative Exploration and Mapping Using Dual RGB-D Robots with YOLO-Based Object Detection

## 🗂️ Repository Layout
🔹 **docker_ws/ :** contains all Docker-related files and scripts required to build and manage the development container. This is where the environment setup is defined. <br>
🔹 **ros_ws/ :** this is the ROS 2 workspace where all project packages are developed. <br>
🔹 **chown_me.sh :** a helper script that fixes file ownership issues when Docker creates files as the root user. Run this if you experience permission problems in your workspace. <br>
🔹 **run.sh :** launches the Docker container with the required capabilities, shared volumes, and environment settings. <br>
🔹 **exec.sh :** opens an interactive shell inside a running container. Useful for running ROS commands, building packages, or debugging directly in the development environment.

### ros_ws/src :
- **turtlebot3_gazebo** <br>
  Adapted from [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations), includes robot models and launch files to spawn the TurtleBot3 in a Gazebo simulation and enable communication with ROS.
  
