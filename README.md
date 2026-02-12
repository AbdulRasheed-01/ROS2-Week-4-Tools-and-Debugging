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
Exercise 1.2: Create Interactive CLI Debug Script

Create debugging/cli_debugger.py:

    #!/usr/bin/env python3
    import subprocess
    import sys
    import time

    class ROS2CLIDebugger:
        def __init__(self):
            self.commands = {
                '1': ('List all nodes', 'ros2 node list'),
                '2': ('List all topics', 'ros2 topic list'),
                '3': ('Show topic info', 'ros2 topic info /topic1'),
                '4': ('Monitor topic frequency', 'ros2 topic hz /topic1'),
                '5': ('Show node info', 'ros2 node info /test_node'),
                '6': ('List parameters', 'ros2 param list'),
                '7': ('Show graph', 'ros2 run rqt_graph rqt_graph'),
                '8': ('Record bag', 'ros2 bag record -a -o debug_bag'),
                '9': ('Check system health', 'ros2 doctor'),
                '10': ('View logs', 'ros2 run rqt_console rqt_console'),
                '11': ('Performance test', 'ros2 run performance_test perf_test'),
                '12': ('Exit', 'exit')
            }
    
        def run_command(self, cmd):
            try:
                if cmd.startswith('ros2 run'):
                    subprocess.Popen(cmd.split())
                elif cmd == 'exit':
                    sys.exit(0)
                else:
                    result = subprocess.run(cmd.split(), capture_output=True, text=True)
                    print(result.stdout)
                    if result.stderr:
                        print(f"Error: {result.stderr}")
            except Exception as e:
                print(f"Failed to run command: {e}")
    
    def menu(self):
        while True:
            print("\n" + "="*50)
            print("ROS 2 CLI Debugger")
            print("="*50)
            for key, (desc, cmd) in self.commands.items():
                print(f"{key}. {desc}")
            
            choice = input("\nSelect option: ")
            if choice in self.commands:
                print(f"\nRunning: {self.commands[choice][1]}\n")
                self.run_command(self.commands[choice][1])
                if choice != '12':
                    input("\nPress Enter to continue...")
            else:
                print("Invalid choice")

    def main():
        debugger = ROS2CLIDebugger()
        debugger.menu()

    if __name__ == '__main__':
        main()
Exercise 2: RViz2 Visualization

2.1 Create Visualization Publisher

Create visualization/visualization_publisher.py:

    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point, Pose, Quaternion
    from std_msgs.msg import ColorRGBA
    import math
    import random

    class VisualizationPublisher(Node):
        def __init__(self):
            super().__init__('visualization_publisher')
        
        # Publishers
            self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
            self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # Timers
            self.timer = self.create_timer(0.1, self.publish_markers)
            self.timer2 = self.create_timer(0.5, self.publish_marker_array)
        
            self.counter = 0
        
        # RViz2 configuration
            self.create_timer(2.0, self.print_rviz_config_instructions)
        
            self.get_logger().info("Visualization Publisher started")
    
        def publish_markers(self):
            """Publish various marker types"""
        
        # 1. Arrow Marker (showing direction)
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "map"
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.ns = "arrows"
            arrow_marker.id = 0
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
        
        # Position
            arrow_marker.pose.position.x = 0.0
            arrow_marker.pose.position.y = 0.0
            arrow_marker.pose.position.z = 0.0
        
        # Orientation (pointing in direction of movement)
            angle = self.counter * 0.1
            arrow_marker.pose.orientation.x = 0.0
            arrow_marker.pose.orientation.y = 0.0
            arrow_marker.pose.orientation.z = math.sin(angle / 2)
            arrow_marker.pose.orientation.w = math.cos(angle / 2)
        
        # Scale
            arrow_marker.scale.x = 1.0
            arrow_marker.scale.y = 0.1
            arrow_marker.scale.z = 0.1
        
        # Color
            arrow_marker.color.r = 1.0
            arrow_marker.color.g = 0.0
            arrow_marker.color.b = 0.0
            arrow_marker.color.a = 1.0
        
            arrow_marker.lifetime.sec = 0  # Never expire
        
            self.marker_pub.publish(arrow_marker)
        
        # 2. Sphere Marker (showing robot position)
            sphere_marker = Marker()
            sphere_marker.header.frame_id = "map"
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = "robot_position"
            sphere_marker.id = 1
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
        
        # Position (circular motion)
            sphere_marker.pose.position.x = 5.0 * math.sin(self.counter * 0.05)
            sphere_marker.pose.position.y = 5.0 * math.cos(self.counter * 0.05)
            sphere_marker.pose.position.z = 0.5
        
            sphere_marker.pose.orientation.w = 1.0
            
        # Scale
            sphere_marker.scale.x = 0.5
            sphere_marker.scale.y = 0.5
            sphere_marker.scale.z = 0.5
        
        # Color (gradient based on position)
            sphere_marker.color.r = abs(math.sin(self.counter * 0.1))
            sphere_marker.color.g = abs(math.cos(self.counter * 0.1))
            sphere_marker.color.b = 0.5
            sphere_marker.color.a = 0.8
        
            sphere_marker.lifetime.sec = 0
        
            self.marker_pub.publish(sphere_marker)
        
        # 3. Cube Marker (obstacle)
            cube_marker = Marker()
            cube_marker.header.frame_id = "map"
            cube_marker.header.stamp = self.get_clock().now().to_msg()
            cube_marker.ns = "obstacles"
            cube_marker.id = 2
            cube_marker.type = Marker.CUBE
            cube_marker.action = Marker.ADD
        
            cube_marker.pose.position.x = 3.0
            cube_marker.pose.position.y = 2.0
            cube_marker.pose.position.z = 0.5
            cube_marker.pose.orientation.w = 1.0
        
            cube_marker.scale.x = 0.8
            cube_marker.scale.y = 0.8
            cube_marker.scale.z = 0.8
        
            cube_marker.color.r = 0.0
            cube_marker.color.g = 1.0
            cube_marker.color.b = 0.0
            cube_marker.color.a = 0.5
        
            self.marker_pub.publish(cube_marker)
        
            # 4. Text Marker (display information)
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "text"
            text_marker.id = 3
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
        
            text_marker.pose.position.x = 0.0
            text_marker.pose.position.y = 0.0
            text_marker.pose.position.z = 1.5
        
            text_marker.pose.orientation.w = 1.0
        
            text_marker.scale.x = 0.5
            text_marker.scale.y = 0.5
            text_marker.scale.z = 0.5
        
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
        
            text_marker.text = f"Frame: {self.counter}\nStatus: Running\nTime: {self.get_clock().now().nanoseconds / 1e9:.1f}s"
        
            self.marker_pub.publish(text_marker)
        
            self.counter += 1
    
        def publish_marker_array(self):
            """Publish multiple markers at once"""
            marker_array = MarkerArray()
        
            # Create multiple sphere markers in a grid
            for i in range(5):
                for j in range(5):
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "grid"
                    marker.id = i * 5 + j + 100
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                
                    marker.pose.position.x = float(i * 2)
                    marker.pose.position.y = float(j * 2)
                    marker.pose.position.z = 0.2
                    marker.pose.orientation.w = 1.0
                
                    marker.scale.x = 0.3
                    marker.scale.y = 0.3
                    marker.scale.z = 0.3
                
                    marker.color.r = random.random()
                    marker.color.g = random.random()
                    marker.color.b = random.random()
                    marker.color.a = 0.7
                
                    marker.lifetime.sec = 2
                
                    marker_array.markers.append(marker)
        
            self.marker_array_pub.publish(marker_array)
    
        def print_rviz_config_instructions(self):
            """Print instructions for RViz2 configuration"""
            self.get_logger().info("="*50)
            self.get_logger().info("RViz2 Configuration Instructions:")
            self.get_logger().info("1. Run: rviz2")
            self.get_logger().info("2. Set Fixed Frame: 'map'")
            self.get_logger().info("3. Add displays:")
            self.get_logger().info("   - Marker: Add → Marker")
            self.get_logger().info("   - MarkerArray: Add → MarkerArray")
            self.get_logger().info("   - TF: Add → TF")
            self.get_logger().info("   - Grid: Add → Grid")
            self.get_logger().info("4. Save config to use later")
            self.get_logger().info("="*50)

    def main(args=None):
        rclpy.init(args=args)
        node = VisualizationPublisher()
    
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
2.2 RViz2 Practice Session:

# Terminal 1: Run visualization publisher
    ros2 run ros2_debugging visualization_publisher

# Terminal 2: Start RViz2
    rviz2

# Terminal 3: Practice RViz2 CLI commands
    rviz2 -d your_config.rviz  # Load saved config
    rviz2 --help               # See all options

# Terminal 4: Monitor RViz2 topics
    ros2 topic list | grep visualization
    ros2 topic info /visualization_marker
    ros2 topic hz /visualization_marker
