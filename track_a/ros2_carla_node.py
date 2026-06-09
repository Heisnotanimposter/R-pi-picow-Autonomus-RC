#!/usr/bin/env python3
"""
ROS2 Control Node for Autonomous Vehicles (CARLA Bridge).
Subscribes to Localization Odometry and Planner Path topics,
calculates throttle/brake/steer using PID and Stanley control laws,
and publishes actuator commands to the vehicle simulator.
"""

import math
import sys

# Try importing ROS2 libraries. Output helpful guides if not found (running outside ROS2 env)
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped
    # Using CARLA-ROS-Bridge control format. Alternate fits AckermannDrive/Twist.
    try:
        from carla_msgs.msg import CarlaEgoVehicleControl
    except ImportError:
        # Fallback custom message mock or import bypass
        CarlaEgoVehicleControl = None
except ImportError:
    print("================================================================")
    print("[WARNING] ROS2 Python library ('rclpy') or standard messages not found.")
    print("This is expected if running outside of a ROS2 environment.")
    print("\nTo integrate this file into your ROS2 workspace:")
    print("1. Copy this script to your custom ROS2 package (e.g. src/my_control_pkg/my_control_pkg/carla_control_node.py)")
    print("2. Ensure 'nav_msgs', 'geometry_msgs', and 'carla_msgs' are in package.xml dependencies.")
    print("3. Source your ROS2 setup script (e.g. source /opt/ros/humble/setup.bash) and build.")
    print("================================================================")
    rclpy = None

# Import our custom controller modules
try:
    from pid_controller import PIDController
    from stanley_controller import StanleyController
except ImportError:
    # Handle path offsets if run from root or subdirectory
    from track_a.pid_controller import PIDController
    from track_a.stanley_controller import StanleyController

class CarlaControlNode(Node if rclpy is not None else object):
    def __init__(self):
        if rclpy is None:
            return
            
        super().__init__('carla_control_node')
        
        # 1. Declare ROS2 Parameters for easy tuning during runtime
        self.declare_parameter('kp', 0.80)
        self.declare_parameter('ki', 0.10)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('stanley_k', 1.20)
        self.declare_parameter('stanley_ks', 0.50)
        self.declare_parameter('target_speed', 5.55) # default 20 km/h in m/s
        self.declare_parameter('wheelbase', 2.87)   # Tesla Model 3 wheelbase
        
        # Read parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.target_speed = self.get_parameter('target_speed').value
        
        # 2. Instantiate controllers with parameters
        self.speed_pid = PIDController(
            kp=self.get_parameter('kp').value,
            ki=self.get_parameter('ki').value,
            kd=self.get_parameter('kd').value,
            output_limits=(-1.0, 1.0)
        )
        
        self.lateral_stanley = StanleyController(
            k=self.get_parameter('stanley_k').value,
            ks=self.get_parameter('stanley_ks').value,
            max_steer=0.523 # max 30 degrees
        )
        
        # 3. Create Subscribers
        # Ego localization and velocity state
        self.odom_sub = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odometry_callback,
            10
        )
        
        # Reference waypoint path from planner
        self.path_sub = self.create_subscription(
            Path,
            '/planning/local_trajectory',
            self.path_callback,
            10
        )
        
        # 4. Create Publisher
        # Control command topic to CARLA bridge
        if CarlaEgoVehicleControl is not None:
            self.control_pub = self.create_publisher(
                CarlaEgoVehicleControl,
                '/carla/ego_vehicle/vehicle_control_cmd',
                10
            )
        else:
            self.get_logger().warn("CarlaEgoVehicleControl msg type unavailable. Publishing mock stream.")
            
        # Vehicle States Cache
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.path_x = []
        self.path_y = []
        
        # 5. Create Controller loop timer (20Hz = 50ms)
        self.timer = self.create_timer(0.05, self.control_loop_callback)
        self.get_logger().info("ROS2 Autonomous Vehicle Control Node initialized successfully.")

    def odometry_callback(self, msg):
        """Processes incoming ego vehicle state updates."""
        # Get position
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        
        # Convert quaternion orientation to Euler Yaw angle (Z rotation)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Get forward velocity (3D vector magnitude)
        v = msg.twist.twist.linear
        self.speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def path_callback(self, msg):
        """Processes planning path waypoints."""
        self.path_x = [pose.pose.position.x for pose in msg.poses]
        self.path_y = [pose.pose.position.y for pose in msg.poses]

    def control_loop_callback(self):
        """Fires control ticks at 20Hz to compute and publish commands."""
        # Ensure we have active waypoints
        if not self.path_x or not self.path_y:
            self.get_logger().warn("Waiting for local trajectory path waypoints...", once=True)
            return
            
        # 1. Update parameters (allows dynamic tuning in rqt_reconfigure)
        self.speed_pid.kp = self.get_parameter('kp').value
        self.speed_pid.ki = self.get_parameter('ki').value
        self.speed_pid.kd = self.get_parameter('kd').value
        self.lateral_stanley.k = self.get_parameter('stanley_k').value
        self.lateral_stanley.ks = self.get_parameter('stanley_ks').value
        self.target_speed = self.get_parameter('target_speed').value
        
        # 2. Longitudinal Control (PID speed matching)
        speed_err = self.target_speed - self.speed
        throttle_brake = self.speed_pid.update(speed_err, dt=0.05)
        
        # 3. Lateral Control (Stanley path steering)
        # Shift reference coordinates to front axle center
        x_front = self.pose_x + (self.wheelbase / 2.0) * math.cos(self.yaw)
        y_front = self.pose_y + (self.wheelbase / 2.0) * math.sin(self.yaw)
        
        steer_angle, target_idx, cte = self.lateral_stanley.calculate_steering(
            x_front, y_front, self.yaw, self.speed, self.path_x, self.path_y
        )
        
        # 4. Construct and publish control packet
        if CarlaEgoVehicleControl is not None:
            cmd = CarlaEgoVehicleControl()
            cmd.header.stamp = self.get_clock().now().to_msg()
            
            # Map longitudinal control to throttle vs brake
            if throttle_brake >= 0.0:
                cmd.throttle = throttle_brake
                cmd.brake = 0.0
            else:
                cmd.throttle = 0.0
                cmd.brake = abs(throttle_brake)
                
            # Map lateral control: normalize steering
            cmd.steer = steer_angle / self.lateral_stanley.max_steer
            cmd.hand_brake = False
            cmd.reverse = False
            
            self.control_pub.publish(cmd)
            
        # Log telemetry data
        self.get_logger().info(
            f"[CTRL] Speed: {self.speed:.2f} m/s | Target: {self.target_speed:.2f} m/s | "
            f"CTE: {cte:.2f}m | Steer: {steer_angle:.2f} rad",
            throttle_duration_sec=1.0 # limit logging rate
        )

def main(args=None):
    if rclpy is None:
        print("[ERROR] ROS2 libraries are not sourced/installed. Execution aborted.")
        sys.exit(1)
        
    rclpy.init(args=args)
    node = CarlaControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
