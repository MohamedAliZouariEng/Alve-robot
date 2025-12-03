# 🤖 Alve-Robot: Automated Industrial Pick-and-Place System

## ⭐ Project Overview

This repository features **Alve-Robot**, a **fully simulated industrial automation system** designed to execute complex **pick-and-place** tasks within a modeled warehouse environment.

The robot **detects and delivers** three types of boxes (Orange, Pizza, Donut) by:

1.  Identifying the box type.
2.  Selecting a specific colored guide line to follow.
3.  Delivering the box to its corresponding conveyor belt.
4.  Returning to its origin using a blob-tracking homing system.

This project serves as the **Capstone Project** for the Industrial Computer Engineer degree at **The National School of Electronics and Telecommunications of Sfax (ENET'Com)**.<a href="https://enetcom.rnu.tn/en" target="_blank">
  <img src="https://scontent.ftun8-1.fna.fbcdn.net/v/t39.30808-6/339283204_136362622577918_8503007294318372148_n.png?_nc_cat=103&ccb=1-7&_nc_sid=6ee11a&_nc_ohc=Ft4DL_NdQtEQ7kNvwHWIPWI&_nc_oc=Adk9O6eSOHMqRlS_gpF5olgxRQvI5wdpRsKos-tUlWz1accbEv0C7UKU3zV6YBR37o4&_nc_zt=23&_nc_ht=scontent.ftun8-1.fna&_nc_gid=NCxeL9Y9LpZUwZ8GENabkg&oh=00_Afg-Q6Zq6LZbnqBJGTJMtSnZX6JU5RXKW-r6GPt60zxnwg&oe=6933C594" alt="ENET'Com Website" height="35" style="vertical-align: middle; margin-left: 5px;">
</a>

  * **Platform:** ROS 2 (Robotics Operating System 2)
  * **Simulation Environment:** Gazebo Sim (Harmonic)
  * **Key Functionality:** Object detection, precise motion planning, and control for a humanoid robot.
-----

## 🛠️ System Architecture & Key Technologies

The Alve-Robot system is built upon modern robotics frameworks to ensure reliability and modularity.

### **Core Components**

| Technology | Role |
| :--- | :--- |
| **ROS 2 Jazzy** | The central communication and application framework. |
| **Gazebo Sim** | High-fidelity 3D simulation of the robot, environment, and physics. |
| **`ros2_control` & `gz_ros2_control`** | Manages the robot's hardware interface and joint-level control. |
| **MoveIt 2** | Advanced motion planning for collision-free and optimal trajectories. |
| **PCL (Point Cloud Library)** | Processes sensor data (e.g., from a simulated depth camera) for 3D object localization. |
| **Ultralytics (YOLO)** | Used within the Python environment for object detection. |

-----

## 💻 Installation & Setup

Follow these steps precisely to set up your environment and run the Alve-Robot simulation.

### 1\. Prerequisites

A high-performance machine is required for the real-time simulation and heavy processing loads of ROS 2, Gazebo, and MoveIt 2.

  * **Operating System:** **Ubuntu 24.04** (Mandatory for ROS 2 Jazzy support).
  * **Minimum Hardware:**
      * **RAM:** 8 GB
      * **CPU:** 8 Cores
      * **GPU:** NVIDIA RTX 2050 4 GB (or equivalent)

### 2\. Install Core ROS 2 Packages

The following packages are essential for the project and must be installed via your system's package manager (`apt`).

| Package/Framework | Installation Command | Official Link |
| :--- | :--- | :--- |
| **ROS 2 Jazzy** | Follow the official guide. | [https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) |
| **`ros_gz_sim`** | `sudo apt-get install ros-jazzy-ros-gz` | [Official GitHub](https://github.com/gazebosim/ros_gz/tree/jazzy) |
| **`ros2_control` & `controllers`** | `sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers` | [Official GitHub](https://github.com/ros-controls/ros2_control/tree/jazzy) |
| **`gz_ros2_control`** | `sudo apt install ros-jazzy-gz-ros2-control` | [Official GitHub](https://github.com/ros-controls/gz_ros2_control/tree/jazzy) |
| **MoveIt 2** | `sudo apt install ros-jazzy-moveit` | [Official Install Guide](https://moveit.ai/install-moveit2/binary/) |
| **Grasping Messages** | `sudo apt install ros-jazzy-grasping-msgs` | [Official GitHub](https://github.com/mikeferguson/grasping_msgs)  |
| **PCL ROS** | `sudo apt install ros-jazzy-pcl-ros` | [Official GitHub](https://github.com/ros-perception/perception_pcl/tree/jazzy) |
| **TRAC-IK Kinematics Solver** | Already in this repository | [Bitbucket Repository](https://bitbucket.org/traclabs/trac_ik/src/jazzy/) |

### 3\. Clone and Setup Workspace

Create your dedicated workspace and move the project files into it.

```bash
# Clone the repository into your home directory
git clone https://github.com/MohamedAliZouariEng/Alve-robot.git

# Create a new ROS 2 workspace structure
mkdir -p ~/ros2_ws/src

# Move all project packages into the 'src' directory
cd ~/Alve-robot
mv * ../ros2_ws/src

# Clean up the now-empty cloned directory
rm -rf ~/Alve-robot
```

### 4\. Setup Python Virtual Environment (Recommended)

This step isolates the required Python dependencies to avoid conflicts with your system's Python installation.

```bash
# Ensure venv is installed
sudo apt install python3.12-venv

# Create and activate the virtual environment
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate

# Install required Python packages
pip install ultralytics # For YOLO object detection
pip install "numpy<2"   # Pin NumPy to a version compatible with many older dependencies
```


### 5\. Build the Workspace

Navigate to your workspace and build the project packages using `colcon`.

```bash
cd ~/ros2_ws
colcon build

# Source the setup file to make the packages visible to ROS 2
source install/setup.bash
```

### 6\. Configure Gazebo Resource Path

Gazebo needs to know where to find the custom robot and environment models. This is crucial for the simulation to load correctly.

```bash
# Set the environment variable for Gazebo models
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/alve_description/urdf:$HOME/ros2_ws/src:$HOME/ros2_ws/src/alve_description/models

# Re-source the workspace setup after setting the environment variable
source install/setup.bash
```

-----

## ▶️ Running the Simulation

Launch the core robot and environment. Then, launch the complete automated pick-and-place scenario.

### 1\. Launch the Base Simulation

This command launches the Gazebo world and the robot, without starting the automation script. It's useful for testing control and viewing the initial setup.

```bash
ros2 launch alve_config alve.launch.py
```

### 2\. Launch the Full Pick-and-Place Scenario

This command launches the simulation, initializes the MoveIt 2 planner, and runs the high-level control script that executes the object detection and the sequence of pick-and-place motions.

```bash
ros2 launch alve_config alve_demo.launch.py
```
# Project Demonstration

<p align="center">
  <a href="https://vimeo.com/user242059085" target="_blank">
    <img src="https://img.shields.io/badge/🎬_Watch_Full_Demo_on_Vimeo-00a2ff?style=for-the-badge&logo=vimeo&logoColor=white&labelColor=0088cc" alt="Watch Demo on Vimeo" style="border-radius: 50px;">
  </a>
</p>

-----
## 🌐 Connect with me

Thank you for your interest in the Alve-Robot project. To explore my other work in robotics and industrial automation, you can connect with me on these professional platforms.

<div align="center">
  <a href="https://www.linkedin.com/in/mohamed-ali-zouari-eng/" target="_blank">
    <img src="https://img.shields.io/badge/-LinkedIn-0077B5?style=flat-square&logo=linkedin&logoColor=white&label=Connect%20on%20LinkedIn" alt="LinkedIn" height="35" style="margin: 0 10px;">
  </a>
  <a href="https://vimeo.com/user242059085" target="_blank">
    <img src="https://img.shields.io/badge/-Vimeo-1AB7EA?style=flat-square&logo=vimeo&logoColor=white&label=Watch%20on%20Vimeo" alt="Vimeo" height="35" style="margin: 0 10px;">
  </a>
</div>

---
