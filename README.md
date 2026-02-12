# ROS2-Week-4-Tools-and-Debugging
🎯 Learning Objectives

By the end of this week, you will be able to:

✅ Master ROS 2 CLI tools for debugging

✅ Visualize robot state with RViz2

✅ Analyze system performance with rqt

✅ Record and playback data with rosbag2

✅ Implement effective logging strategies

✅ Debug common ROS 2 issues

✅ Profile and optimize ROS 2 nodes

📚 Theory Content

4.1 ROS 2 Tool Ecosystem

CLI Tools    │  GUI Tools      │  Analysis Tools  

ros2 node    │  rviz2          │  ros2 bag 

ros2 topic   │  rqt_graph      │  ros2 doctor           

ros2 service │  rqt_plot       │  ros2 wtf  

ros2 param   │  rqt_console    │  performance_test

ros2 action  │  rqt_reconfigure│  system_monitor    

ros2 bag     │  plotjuggler    │  trace_analyzer 

4.2 Understanding ROS 2 CLI Tools

Core Command Structure:

    ros2 <command> <subcommand> [arguments] [options]
Command Categories:

Node Management: node, run, pkg

Communication: topic, service, action, param

System: launch, lifecycle, multicast

Data Recording: bag

Diagnostics: doctor, wtf, component

4.3 Visualization with RViz2

RViz2 Architecture:

Displays: Visualization plugins

Tools: Interactive tools (Move Camera, Select, Interact)

Views: Camera viewpoints

Panels: Time panel, tool properties

Common Displays:


Display	   |    Purpose	            |      Topic Type

RobotModel |    Visualize URDF      |     /robot_description

TF	       |    Transform frames    |    /tf, /tf_static

LaserScan  |    LiDAR data	        |    sensor_msgs/LaserScan

Image	   |     Camera feed	    |       sensor_msgs/Image

Marker	   |     Custom visualization  |  visualization_msgs/Marker

Path	   |     Robot path	        |    nav_msgs/Path

Pose	   |     Robot pose	        |    geometry_msgs/Pose

4.4 ROS 2 Logging System

Logging Levels (Increasing Severity):

    DEBUG   # Fine-grained debug information
    INFO    # Normal operational messages
    WARN    # Something unexpected but not critical
    ERROR   # Something failed but system continues
    FATAL   # Critical failure, system cannot continue

Logging Configuration:

    #Set console output format
    
    export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

    #Set logging level for all nodes

    ros2 run my_pkg my_node --ros-args --log-level DEBUG

    #Set logging level for specific node
    ros2 run my_pkg my_node --ros-args --log-level rcl:=DEBUG

    
