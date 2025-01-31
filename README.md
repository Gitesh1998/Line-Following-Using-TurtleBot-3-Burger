
## Line Following Using TurtleBot 3 Burger

The “Line Following Using TurtleBot 3 Burger” project successfully achieved its objectives, including full system integration, which combined line detection, PID control, waypoint navigation, and obstacle avoidance into a unified and operational framework

### Table  of Contents:
1. Motivation
2. Pre-requisite
3. Installation
4. Usage
5. Future Improvement
6. Contact
7. License

### 🏆 Motivation

Autonomous mobile robots play a crucial role in modern robotics applications, from warehouse automation to smart transportation systems. The ability to **follow lines, navigate waypoints, and avoid dynamic obstacles** is fundamental for robots operating in structured and unstructured environments.

This project aims to **develop an efficient and robust TurtleBot 3 Burger system** that can:  
✅ **Follow a predefined path** using real-time vision-based line detection.  
✅ **Navigate to waypoints smoothly** while maintaining stability.  
✅ **Detect and avoid obstacles dynamically** using sensor feedback.  
✅ **Bridge the gap between simulation and hardware** for real-world deployment.

By leveraging **ROS2, OpenCV, and Gazebo Simulator**, this project not only enhances the understanding of robot perception and control but also provides a **practical foundation for real-world autonomous navigation**. The optimizations ensure smooth performance even with limited computational resources, making it an **ideal step toward scalable robotic applications**.


### 📌 Prerequisites

#### 💻 System Requirements

-   **Operating System:** Ubuntu 20.04 or later
-   **Processor:** x86_64 or ARM-based (Jetson Nano, Raspberry Pi, etc.)
-   **Memory:** Minimum 4GB RAM (8GB recommended)
-   **ROS Version:** ROS2 (Humble recommended)
-   **TurtleBot 3 Model:** Burger

### Installation: 

1.  [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)
2.  [OpenCV Installation](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)
3.  [Gazebo Installation](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
4.  [ROS@ Humble installtion on Turtle 3 burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
5.  [USB Camera Driver for ROS2](https://index.ros.org/p/usb-cam/github-ros-drivers-usb-cam/)

📌 Please check the specific versions mentioned earlier before installation to avoid compatibility issues. 🚀

### Usage:   


### 🚀 Usage & How to Run the Code

#### 1️⃣ **Prerequisites**

Ensure that you have the required dependencies installed:

-   ROS2 Humble
-   OpenCV (`pip install opencv-python`)
-   NumPy (`pip install numpy`)
-   A USB camera (or built-in camera) connected to the system
-   A TurtleBot 3 with lidar support

----------

#### 2️⃣ **Set Up ROS2 Environment**

Before running the script, make sure ROS2 is sourced:

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash  # If using a custom ROS2 workspace
export TURTLEBOT3_MODEL=burger

```

----------

#### 3️⃣ **Run the TurtleBot 3 Simulation (Optional, if using Gazebo)**

If you want to test in simulation before running on real hardware, launch the Gazebo environment:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```

Launch the **lidar node** for obstacle detection:

```bash
ros2 launch turtlebot3_bringup robot.launch.py

```

----------

#### 4️⃣ **Run the Line Follower Code**

Navigate to the directory containing the script and execute:

```bash
python3 line_follower.py

```

If using a different camera index (e.g., external USB camera), modify this line in the code:

```python
self.cap = cv2.VideoCapture(0)  # Change 0 to 1 if using an external camera

```

----------

#### 5️⃣ **Stopping the Robot**

-   If running in a terminal, press **Ctrl+C** to stop execution.
-   The robot will automatically stop moving when an obstacle is detected within **30 

### 📌 **Expected Behavior**

-   The robot follows a **black line** using OpenCV.
-   It detects obstacles within **30 cm** using **lidar** and stops movement.
-   The bot will turn **left or right** if the line deviates from the center.
-   If the line is **lost**, the bot stops and waits.


### 🔍 Future Improvements

1️⃣ **Enhance Obstacle Avoidance** – Implement a **more advanced obstacle detection** system using **depth cameras** or **sensor fusion** (lidar + camera) for better decision-making.

2️⃣ **Adaptive PID Tuning** – Develop a **dynamic PID controller** that adjusts parameters in real-time based on speed, surface conditions, and external disturbances.

3️⃣ **Multi-Line Detection & Intersection Handling** – Extend the system to handle **multiple paths, intersections, and turn decisions** for more complex navigation tasks.

4️⃣ **Optimize Computational Efficiency** – Use **hardware acceleration** (e.g., OpenCV with CUDA on Jetson Nano) to process images faster, improving real-time performance.

5️⃣ **ROS2 Navigation Stack Integration** – Replace the custom waypoint system with the **ROS2 Navigation Stack** for more autonomous and optimized path planning.


### Contact:
![https://www.linkedin.com/in/gitesh-pawar-r/](https://cdn-icons-png.flaticon.com/128/3536/3536505.png)
![giteshpawar56@gmail.com](https://cdn-icons-png.flaticon.com/128/732/732200.png)

### License:
Distributed under the GPL License.
