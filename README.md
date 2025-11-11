🤖 Humbot — Autonomous Navigation Robot

🧭 A modular ROS 2-based navigation and motion framework integrating perception, planning, control, and behavior logic for autonomous mobile robots.

🧠 Core Modules Overview
Package	Description
🧩 humbot_bringup	System-level bringup package that launches the entire robot stack (Gazebo simulation, controller, localization, and navigation). Handles parameter loading and node lifecycle management.
⚙️ humbot_controller	Implements velocity command arbitration using Twist Mux with a priority hierarchy:
• 🕹️ Joystick → Highest
• ⌨️ Keyboard teleop → Medium
• 🧭 Local planner (Nav2) → Lowest
Ensures smooth control switching between manual and autonomous modes.
🗺️ humbot_localization	Provides multi-mode localization using IMU + Odometry fusion:
• Default: AMCL (Adaptive Monte Carlo Localization) for global localization against a map.
• Optional: EKF-based localization via robot_localization for sensor fusion.
• Odometry-only motion model mode available for simple simulation setups.
🧭 humbot_planning	Implements A* and Dijkstra algorithms as global planner plugins for Nav2. Each planner is exported as a plugin and managed through the Planner Server for easy algorithm switching.
🦾 humbot_motion	Contains PD-based motion planner and Pure Pursuit controller implementations for local motion control. Both are exported as motion planner plugins, allowing flexible integration within Nav2 or standalone controller servers.
🧬 humbot_navigation	Defines high-level Behavior Trees governing robot autonomy, including task sequencing, recovery behaviors, and smooth transitions between navigation states. Also manages costmap configurations and integrates a Smoother Server for post-planning trajectory refinement.
🌍 humbot_mapping	Handles mapping and SLAM functionality, managing map data, configuration, and visualization through RViz.
🪐 humbot_description	Provides robot models, URDF/Xacro files, Gazebo plugins, meshes, and environment worlds. Used for both visualization and simulation.
💬 humbot_msgs	Contains all custom message, service, and action definitions used across the workspace (e.g., safety stop, motion control actions).
🧰 humbot_utils	Implements a Safety Stop system to monitor robot velocity commands and halt the robot during unsafe conditions or emergency scenarios.
🧩 Key Capabilities

✅ Modular ROS 2 Humble architecture
✅ Simulation-ready in Gazebo + RViz 2
✅ Multi-mode localization (AMCL / EKF / Odometry)
✅ Twist Mux-based input arbitration
✅ PD & Pure Pursuit motion control exported as plugins
✅ Global planners (A*, Dijkstra) via plugin-based Planner Server
✅ Behavior Tree–driven navigation flow
✅ Smooth trajectory refinement using a dedicated Smoother Server
✅ Lifecycle node management for robust bringup and shutdown
✅ Safety Stop mechanism for runtime safety assurance

🧪 Run the Simulation
cd ~/humbot_ws
colcon build
source install/setup.bash
ros2 launch humbot_bringup simulated_robot.launch.py


This launches:

Gazebo world
Controller stack (Twist Mux)
Localization (default AMCL)
Global and local planners
Behavior Tree–based navigation pipeline

🧭 Tech Stack

ROS 2 Humble Hawksbill
Nav2 Core Framework
Gazebo Classic / Fortress
C++ 17
RViz 2 Visualization
Behavior Tree CPP, Twist Mux, robot_localization, amcl


🎥 Demo

🔗 Add your Gazebo / real-world demo video link below:


📂 Workspace Structure
humbot_ws/
├── src/
│   ├── humbot_bringup/
│   ├── humbot_controller/
│   ├── humbot_description/
│   ├── humbot_localization/
│   ├── humbot_mapping/
│   ├── humbot_motion/
│   ├── humbot_msgs/
│   ├── humbot_navigation/
│   ├── humbot_planning/
│   └── humbot_utils/
├── install/
├── build/
└── log/

🚀 Future Roadmap

 Add Model Predictive Path Integral (MPPI) control

 Reinforcement-learning-based adaptive navigation

 Integrate dynamic obstacle tracking and re-planning

 Deploy on physical robot hardware

 Add human-aware navigation and social behavior layers

📄 License

MIT License © 2025 Ishika Saijwal

👩‍💻 Author

Ishika Saijwal
🔗 GitHub
 • LinkedIn
