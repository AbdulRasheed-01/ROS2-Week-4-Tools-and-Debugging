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

    
4.5 Rosbag2: Data Recording and Playback
Rosbag2 Features:

Record specific topics or all topics

Compressed storage

SQLite3 database format

Splitting large bags by size/time

Converting between formats

Storage Formats:

Format	        |        Compression	|    Use Case

SQLite3	        |        None	        |    Standard recording

MCAP	        |        Optional	    |   Streaming, web

SQLite3 + Compression  | ZSTD/LZ4	    |    Large datasets

4.6 Performance Monitoring

Key Metrics:

Frequency: Topic publication rate

Latency: Message propagation delay

Jitter: Variation in timing

Throughput: Messages per second

CPU/Memory: Node resource usage

Network: DDS traffic analysis

⚙️ Hands-On Setup
Step 1: Install Additional Tools

    #Install rqt tools
    sudo apt install ros-humble-rqt*    
    sudo apt install ros-humble-rqt-graph
    sudo apt install ros-humble-rqt-plot
    sudo apt install ros-humble-rqt-console
    sudo apt install ros-humble-rqt-reconfigure
    sudo apt install ros-humble-rqt-tf-tree

    #Install rosbag2
    sudo apt install ros-humble-rosbag2*
    sudo apt install ros-humble-rosbag2-transport
    sudo apt install ros-humble-rosbag2-storage-default-plugins

    #Install performance tools
    sudo apt install ros-humble-system-metrics-collector
    sudo apt install ros-humble-ros2cli-common-extensions

    #Install PlotJuggler for advanced plotting
    sudo apt install ros-humble-plotjuggler-ros

    #Install additional debugging tools
    sudo apt install ros-humble-demo-nodes-cpp
    sudo apt install ros-humble-demo-nodes-py
    sudo apt install ros-humble-turtlesim

Step 2: Create Debugging Workspace Package

    cd ~/ros2_ws/src
    ros2 pkg create ros2_debugging --build-type ament_python \
        --dependencies rclpy std_msgs sensor_msgs visualization_msgs \
        --description "Week 4: ROS 2 Debugging Tools"

    cd ros2_debugging
    mkdir -p ros2_debugging/{debugging,visualization,performance,rosbag}
    mkdir -p {launch,config}

🔧 Practical Exercises
Exercise 1: Master ROS 2 CLI Tools

1.1 Node Inspection and Management

Create a test node to inspect:

    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32, Int32
    import random

    class TestNode(Node):
        def __init__(self, node_name='test_node'):
            super().__init__(node_name)
        
        # Publishers
            self.pub1 = self.create_publisher(String, 'topic1', 10)
            self.pub2 = self.create_publisher(Float32, 'topic2', 10)
            self.pub3 = self.create_publisher(Int32, 'topic3', 10)
        
        # Subscribers
            self.sub1 = self.create_subscription(
                String, 'feedback', self.feedback_callback, 10)
        
        # Timers
            self.timer1 = self.create_timer(1.0, self.timer1_callback)
            self.timer2 = self.create_timer(0.5, self.timer2_callback)
            self.timer3 = self.create_timer(2.0, self.timer3_callback)
        
        # Parameters
            self.declare_parameter('publish_rate', 1.0)
            self.declare_parameter('node_enabled', True)
            self.declare_parameter('message_text', 'Hello ROS2')
        
            self.count = 0
    
        def timer1_callback(self):
            msg = String()
            msg.data = f"{self.get_parameter('message_text').value} - {self.count}"
            self.pub1.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
            self.count += 1
    
        def timer2_callback(self):
            msg = Float32()
            msg.data = random.uniform(0.0, 100.0)
            self.pub2.publish(msg)
    
        def timer3_callback(self):
            msg = Int32()
            msg.data = random.randint(0, 1000)
            self.pub3.publish(msg)
    
        def feedback_callback(self, msg):
            self.get_logger().info(f'Received feedback: {msg.data}')

    def main(args=None):
        rclpy.init(args=args)
        node = TestNode()
    
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

# Terminal 1: Run the test node
    ros2 run ros2_debugging test_node

# Terminal 2: Practice node commands
    ros2 node list
    ros2 node info /test_node
    ros2 node --help

# Terminal 3: Practice topic commands
    ros2 topic list
    ros2 topic list -t  # Show topic types
    ros2 topic info /topic1
    ros2 topic type /topic1
    ros2 topic hz /topic1
    ros2 topic bw /topic1
    ros2 topic echo /topic1 --once
    ros2 topic echo /topic1 --csv > topic_data.csv

# Terminal 4: Practice service commands
    ros2 service list
    ros2 service list -t
    ros2 service type /test_node/describe_parameters
    ros2 service call /test_node/get_parameters rcl_interfaces/srv/GetParameters "{names: ['publish_rate']}"

# Terminal 5: Practice parameter commands
    ros2 param list
    ros2 param get /test_node publish_rate
    ros2 param set /test_node publish_rate 2.0
    ros2 param dump /test_node > test_node_params.yaml
    ros2 param load /test_node test_node_params.yaml

# Terminal 6: Practice action commands (if actions exist)
    ros2 action list
    ros2 action info /fibonacci

