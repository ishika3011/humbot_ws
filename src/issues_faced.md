ISSUES FACED

1. Odometry-Only Mapping Drift

----Symptoms----

Map in RViz starts fine but becomes distorted after the robot turns or moves a lot.
Appears as starburst rays, double walls, or smeared map data.

----Cause----

The mapping node inserts laser scan data into the occupancy grid directly using odometry pose.
Odometry (odom → base_link) always accumulates error (especially angular drift).
1. Since map is built in the odom frame:
        Any drift in odometry directly shifts the map.
        No correction mechanism exists → distortion increases over time.

2. any small wear/slip (change in wheel size), then also distorted map since change in errors 

----Why it happens----

The node is essentially an occupancy grid inserter, not full SLAM.
It does not perform:
    Scan matching
    Sensor fusion
    Map-to-odom correction (map → odom transform) 




<<------------------------------------------------------------------------------------------------------------------------>>

⚠️ Error explained:

failed to create symbolic link '/home/ishika/humbot_ws/build/humbot_msgs/ament_cmake_python/humbot_msgs/humbot_msgs' because existing path cannot be removed: Is a directory

This means there’s a directory where a symlink should be — ROS2 is trying to make a symbolic link from your build folder to your Python package, but the folder already exists.

💡 Solution: rm -rf build install log

✅ Why this works:
When you use --symlink-install, ROS2 tries to link the source folders directly instead of copying them.
If there’s an existing folder from a previous non-symlink build, it conflicts — so we remove it all and start clean.

<<--------------------------------------------------------------------------------------------------------------------------------->>

⚠️ .rviz files were not loading

💡 Forgot to add in CMakeLists 

✅ install(
        DIRECTORY config launch maps rviz
        DESTINATION share/${PROJECT_NAME})

<<--------------------------------------------------------------------------------------------------------------------------------->>

💡 When you add --symlink-install

Instead of copying, Colcon creates symbolic links (shortcuts) in install/ that point directly to your src/ files.

So:

When you edit a file in src/, it’s immediately visible in install/.

You don’t need to rebuild every time for minor changes like .rviz, .yaml, .launch.py.

✅ Benefits for you (especially in your workflow)
Task	With normal build	With --symlink-install
Edit .rviz or .launch.py	Must rebuild	Instantly applied
Debug/test configs quickly	Slower	Faster
Workspace rebuild size	Copies everything	Small symbolic links
⚙️ When to use it

Always use --symlink-install for development workspaces, especially when:

You’re frequently editing launch/config/rviz files

You’re debugging or iterating quickly

You can skip it only when preparing for a release build (where you want a clean, copy-based install).