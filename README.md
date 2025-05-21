# 🤖 Arduinobot – ROS 2 Robotic Arm Simulation

This project simulates a modular robotic arm (Arduinobot) using **ROS 2 Foxy**, **Gazebo**, and **ros2_control**. It showcases a complete robot description, control setup, and simulation environment integration — designed to be hardware-ready and extensible for advanced tasks like **additive manufacturing (Direct Energy Deposition)**.

---

## 🛠️ Features

- ✅ **URDF/Xacro-based robot model** with 5 joints (3 for arm, 2 for gripper)
- ✅ Simulated in **Gazebo** using `ros_gz_sim` and `ros_gz_bridge`
- ✅ **ROS 2 Control** integration via `controller_manager` and joint broadcasters
- ✅ Lifecycle-aware ROS 2 launch system with parameterized configs
- ✅ Modular setup with separate launch files for RViz, Gazebo, and control nodes
- ✅ Real-time state feedback, pluginlib debugging, and full system diagnostics
- 🔄 Planned: Extend to simulate **DED (Direct Energy Deposition)** for additive manufacturing workflows

---

## 📁 Package Structure

```bash
├── arduinobot_description
│   ├── urdf/                  # URDF and Xacro models
│   ├── meshes/                # STL files for all robot parts
│   ├── launch/                # Launch files for RViz, Gazebo
│   └── rviz/                  # Display config
├── arduinobot_controller
│   ├── config/                # ros2_control YAMLs
│   └── launch/                # Controller launch files
```

## 🚀 Launch Instructions
Ensure you're using ROS 2 Foxy, have sourced your workspace, and installed all required packages (see dependencies below).

1. Launch in RViz (Robot Model View)
```
ros2 launch arduinobot_description display.launch.py
```
2. Launch in Gazebo Simulation
```
ros2 launch arduinobot_description gazebo.launch.py
```
3. Load and Spawn Controllers
```
ros2 launch arduinobot_controller controller.launch.py
```

📽 Demo
📹 [Add a YouTube link or GIF here once uploaded]

## 🧩 Dependencies
ROS 2 Foxy
ros_gz_sim, ros_gz_bridge
ros2_control, controller_manager
robot_state_publisher, joint_state_publisher_gui
xacro, rviz2, urdf
Install ros_gz_sim if not already available

## 🧠 Future Work
Integrate DED additive manufacturing simulation with task planning nodes
Add moveit2 support for motion planning
Expand URDF to include tool center point sensors and nozzle actuators
Publish a blog post detailing pluginlib debugging and ros2_control setup

## 📚 References
ros2_control Documentation
Gazebo + ROS Integration
URDF Tutorials
ROS 2 Lifecycle

## 🧑‍💻 Maintainer
Raj Manoj Bag
📧 rajbag4321@gmail.com
🔗 LinkedIn
🔗 Portfolio

Built with ❤️ using open-source robotics tools for real-world-ready simulation and future deployment in additive automation.
