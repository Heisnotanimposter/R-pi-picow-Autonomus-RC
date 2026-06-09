#!/usr/bin/env python3
"""
Stanley Lateral Controller Module.
Implements the path-tracking Stanley control law (used in the DARPA Grand Challenge
and taught in the University of Toronto course). It references errors from the
front axle of the vehicle.
"""

import numpy as np

class StanleyController:
    def __init__(self, k=1.0, ks=0.5, max_steer=0.523):
        """
        Initializes the Stanley lateral controller.
        
        Parameters:
        - k: Position/Cross-track error gain (higher values steer harder to the line)
        - ks: Softening constant (prevents division by zero at low speeds)
        - max_steer: Maximum steering angle limit (radians) (default ~30 degrees)
        """
        self.k = k
        self.ks = ks
        self.max_steer = max_steer

    def normalize_angle(self, angle):
        """Normalizes an angle to the range [-pi, pi]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def calculate_steering(self, x_front, y_front, yaw, v, path_x, path_y):
        """
        Computes the target steering angle in radians using the Stanley control law.
        
        Parameters:
        - x_front: Vehicle X coordinate at the center of the front axle.
        - y_front: Vehicle Y coordinate at the center of the front axle.
        - yaw: Vehicle heading yaw angle (radians).
        - v: Current forward speed (m/s).
        - path_x: List/Array of path waypoint X coordinates.
        - path_y: List/Array of path waypoint Y coordinates.
        
        Returns:
        - steering_angle: Target steer command in radians (positive = Left, negative = Right)
        - min_idx: Index of the closest waypoint found
        - cte: The cross-track error value (meters)
        """
        if len(path_x) == 0:
            return 0.0, 0, 0.0
            
        # 1. Find the closest waypoint on the path to the vehicle's front axle
        dx = [x_front - px for px in path_x]
        dy = [y_front - py for py in path_y]
        distances = np.hypot(dx, dy)
        min_idx = np.argmin(distances)
        
        # 2. Compute the cross-track error (cte) magnitude
        cte = distances[min_idx]
        
        # 3. Determine the sign of the cross-track error
        # Use vector projection: vector from waypoint to vehicle projected onto the path normal.
        # Path direction vector (tangent at closest waypoint)
        if min_idx < len(path_x) - 1:
            tangent_x = path_x[min_idx + 1] - path_x[min_idx]
            tangent_y = path_y[min_idx + 1] - path_y[min_idx]
        elif min_idx > 0:
            tangent_x = path_x[min_idx] - path_x[min_idx - 1]
            tangent_y = path_y[min_idx] - path_y[min_idx - 1]
        else:
            tangent_x = 1.0
            tangent_y = 0.0
            
        path_yaw = np.arctan2(tangent_y, tangent_x)
        
        # Vector from path waypoint to vehicle front axle
        vec_wp_to_car_x = x_front - path_x[min_idx]
        vec_wp_to_car_y = y_front - path_y[min_idx]
        
        # Cross product to find if vehicle is on the left or right of the path
        # positive = vehicle is to the left of the path, negative = right
        cross_product = tangent_x * vec_wp_to_car_y - tangent_y * vec_wp_to_car_x
        if cross_product < 0:
            cte = -cte # Negative cte means vehicle is to the right of the path

        # 4. Compute heading error (yaw_error)
        # Difference between path orientation (path_yaw) and vehicle heading (yaw)
        # yaw_error = path_yaw - yaw
        yaw_error = self.normalize_angle(path_yaw - yaw)
        
        # 5. Stanley Lateral Control Law formula (University of Toronto/Coursera Formulation):
        # delta(t) = yaw_error(t) + arctan( (k * cte(t)) / (v(t) + ks) )
        #
        # Where:
        # - yaw_error: Align vehicle orientation to path tangent.
        # - arctan((k * cte) / (v + ks)): Steer to eliminate lateral offset.
        # - k: Position feedback gain (determines how aggressively we correct offsets).
        # - ks: Softening constant (prevents division by zero and stabilizes at low speeds).
        #
        # Note: Using np.arctan2(y, x) is equivalent to np.arctan(y/x) but handles division
        # by zero and quadrant normalization robustly.
        c_steer = yaw_error + np.arctan2(self.k * cte, v + self.ks)
        
        # 6. Normalize and clamp steering command to physical limits
        steering_angle = self.normalize_angle(c_steer)
        steering_angle = np.clip(steering_angle, -self.max_steer, self.max_steer)
        
        return steering_angle, min_idx, cte

if __name__ == '__main__':
    # Test Stanley Controller with a straight horizontal path (y = 5.0) heading East (yaw = 0.0)
    print("Testing Stanley Lateral Controller...")
    
    path_x = np.linspace(0.0, 100.0, 101)
    path_y = np.ones_like(path_x) * 5.0 # path along y = 5.0
    
    # Ego vehicle starts at front axle coordinate: (0.0, 4.0), pointing East (yaw = 0.0)
    # This means cross-track error is -1.0 meters (vehicle is 1.0m to the right/below the path).
    x_front = 0.0
    y_front = 4.0
    yaw = 0.0
    v = 5.0 # speed 5 m/s
    
    controller = StanleyController(k=1.2, ks=0.5, max_steer=0.523) # max 30 deg
    
    dt = 0.1
    print(f"{'Time (s)':<8} | {'Pose (x, y)':<15} | {'Yaw (deg)':<10} | {'CTE (m)':<8} | {'Steer (deg)':<10}")
    print("-" * 65)
    
    for step in range(25):
        t = step * dt
        
        # Calculate steering command
        steer_rad, idx, cte = controller.calculate_steering(x_front, y_front, yaw, v, path_x, path_y)
        
        # Log values (convert to degrees for printing)
        yaw_deg = np.degrees(yaw)
        steer_deg = np.degrees(steer_rad)
        print(f"{t:<8.1f} | ({x_front:<5.2f}, {y_front:<5.2f}) | {yaw_deg:<10.1f} | {cte:<8.2f} | {steer_deg:<10.1f}")
        
        # Simple vehicle kinematic model update (bicycle model approximation)
        # x_front = x_front + v * cos(yaw + steer) * dt
        # y_front = y_front + v * sin(yaw + steer) * dt
        # yaw = yaw + (v / wheel_base) * sin(steer) * dt
        wheel_base = 2.5
        x_front += v * np.cos(yaw + steer_rad) * dt
        y_front += v * np.sin(yaw + steer_rad) * dt
        yaw += (v / wheel_base) * np.sin(steer_rad) * dt
        yaw = controller.normalize_angle(yaw)
