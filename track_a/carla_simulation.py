#!/usr/bin/env python3
"""
CARLA Autonomous Driving Simulation Integration Script.
Connects to the CARLA simulator client, spawns an autonomous vehicle,
extracts location and velocity telemetry, runs the longitudinal PID and 
lateral Stanley controllers, and drives the vehicle autonomously on a route.
"""

import sys
import time
import math

# Try importing CARLA API library. Handle fallback gracefully if not installed.
try:
    import carla
except ImportError:
    print("================================================================")
    print("[WARNING] CARLA Python API module not found ('import carla' failed).")
    print("To run this script in a real CARLA simulator environment:")
    print("1. Install the CARLA simulator (http://carla.org/)")
    print("2. Install the client module: pip install carla")
    print("================================================================")
    carla = None

# Import our custom controller modules
try:
    from pid_controller import PIDController
    from stanley_controller import StanleyController
except ImportError:
    # Handle path offsets if run from root or subdirectory
    from track_a.pid_controller import PIDController
    from track_a.stanley_controller import StanleyController

class CarlaEgoVehicleAgent:
    def __init__(self, world, vehicle_blueprint_name="vehicle.tesla.model3"):
        self.world = world
        self.blueprint_library = world.get_blueprint_library()
        self.map = world.get_map()
        
        # 1. Choose spawn point (usually from map recommended spawn points)
        spawn_points = self.map.get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points found on the current CARLA map!")
        spawn_point = spawn_points[0]
        
        # 2. Spawn the ego-vehicle actor
        blueprint = self.blueprint_library.find(vehicle_blueprint_name)
        blueprint.set_attribute('role_name', 'ego_vehicle')
        self.vehicle = world.spawn_actor(blueprint, spawn_point)
        print(f"[CARLA] Spawned vehicle: '{vehicle_blueprint_name}' at {spawn_point.location}")
        
        # Vehicle specifications (needed for Stanley front-axle calculations)
        self.wheel_base = 2.87 # Tesla Model 3 wheelbase (meters)
        
        # 3. Instantiate controllers
        # Longitudinal speed controller: maps speed error to throttle (0.0 to 1.0) and brake (0.0 to 1.0)
        self.speed_pid = PIDController(kp=0.8, ki=0.1, kd=0.05, output_limits=(-1.0, 1.0))
        
        # Lateral steering controller: maps path alignment to steering angle (-1.0 to 1.0 scaling)
        self.lateral_stanley = StanleyController(k=1.5, ks=0.5, max_steer=0.6) # max 35 deg

    def get_front_axle_position(self, location, yaw_rad):
        """Displaces vehicle location center to front axle coordinate."""
        # Forward displacement: center location + (wheel_base / 2) * heading_vector
        x_front = location.x + (self.wheel_base / 2.0) * math.cos(yaw_rad)
        y_front = location.y + (self.wheel_base / 2.0) * math.sin(yaw_rad)
        return x_front, y_front

    def control_step(self, target_speed_mps, path_x, path_y):
        """
        Executes a single controller step.
        Retrieves vehicle state, runs controllers, and applies vehicle actuators.
        """
        if self.vehicle is None:
            return
            
        # 1. Read vehicle state telemetry
        transform = self.vehicle.get_transform()
        location = transform.location
        yaw_deg = transform.rotation.yaw
        yaw_rad = math.radians(yaw_deg)
        
        velocity = self.vehicle.get_velocity()
        # Speed in meters/sec (3D magnitude)
        current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # 2. Compute longitudinal control (speed)
        speed_error = target_speed_mps - current_speed
        throttle_brake_cmd = self.speed_pid.update(speed_error, dt=0.05) # 20Hz steps
        
        # Convert PID output to CARLA throttle/brake actuators
        control = carla.VehicleControl()
        if throttle_brake_cmd >= 0.0:
            control.throttle = throttle_brake_cmd
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = abs(throttle_brake_cmd) # apply braking pressure
            
        # 3. Compute lateral control (steering)
        x_front, y_front = self.get_front_axle_position(location, yaw_rad)
        
        # Stanley output is target steering wheel angle in radians
        steer_angle_rad, target_idx, cte = self.lateral_stanley.calculate_steering(
            x_front, y_front, yaw_rad, current_speed, path_x, path_y
        )
        
        # CARLA expects normalize steer parameter between -1.0 and 1.0
        # Divide target steering angle by the max steering limit of the vehicle
        normalized_steer = steer_angle_rad / self.lateral_stanley.max_steer
        control.steer = normalized_steer
        
        # 4. Apply actuators to vehicle
        self.vehicle.apply_control(control)
        
        # Print status updates
        print(f"[CTRL] Speed: {current_speed:.1f} m/s | Target: {target_speed_mps:.1f} m/s | "
              f"CTE: {cte:.2f}m | Steer Commanded: {control.steer:.2f}", end='\r')

    def destroy(self):
        """Clean up actor when simulation stops."""
        if self.vehicle is not None:
            self.vehicle.destroy()
            print("\n[CARLA] Ego-vehicle destroyed safely.")
            self.vehicle = None

def main():
    if carla is None:
        print("[ERROR] Cannot run simulation loop without CARLA engine libraries.")
        sys.exit(1)
        
    print("Connecting to CARLA Simulator Server...")
    try:
        # Standard local port is 2000
        client = carla.Client("localhost", 2000)
        client.set_timeout(5.0)
        
        world = client.get_world()
        agent = CarlaEgoVehicleAgent(world)
        
        # Generate a straight waypoint path for demonstration
        # In a real run, you would load waypoints from the HD road map definition
        start_loc = agent.vehicle.get_transform().location
        path_x = [start_loc.x + i * 2.0 for i in range(100)]
        path_y = [start_loc.y for _ in range(100)]
        
        target_speed_mps = 8.33 # ~30 km/h
        
        # Run autonomous control loop at 20Hz (50ms interval)
        print("[CARLA] Autopilot active. Press Ctrl+C to terminate.")
        while True:
            agent.control_step(target_speed_mps, path_x, path_y)
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nExiting simulation.")
    except Exception as e:
        print(f"\nSimulation exception occurred: {e}")
    finally:
        if 'agent' in locals():
            agent.destroy()

if __name__ == "__main__":
    main()
