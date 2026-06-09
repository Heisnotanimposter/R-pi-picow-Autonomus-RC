#!/usr/bin/env python3
"""
PID Controller Module for Autonomous Vehicle Guidance.
Provides feedback control for longitudinal tracking (speed/acceleration)
or lateral path alignment. Includes anti-windup and derivative filtering.
"""

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None), alpha=1.0):
        """
        Initializes the PID controller.
        
        Parameters:
        - kp: Proportional gain
        - ki: Integral gain
        - kd: Derivative gain
        - output_limits: Tuple (min_val, max_val) to clamp control commands
        - alpha: Derivative low-pass filter coefficient (0.0 to 1.0)
                 1.0 = raw derivative, < 1.0 = smoother (filters out sensor jitter)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.alpha = alpha
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        
        # Max value to bound the integral term (limits accumulation error)
        self.integral_limit = 50.0

    def update(self, error, dt):
        """
        Computes the PID control variable.
        
        Parameters:
        - error: The current error value (target - measured)
        - dt: Time step (seconds)
        """
        if dt <= 0.0:
            return 0.0
            
        # 1. Proportional term
        p_term = self.kp * error
        
        # 2. Integral term with anti-windup clamping
        self.integral += error * dt
        # Clamp integral term to prevent runaway windup
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        i_term = self.ki * self.integral
        
        # 3. Derivative term with low-pass filter
        raw_derivative = (error - self.prev_error) / dt
        # Low-pass filter: y[t] = alpha * x[t] + (1 - alpha) * y[t-1]
        filtered_derivative = self.alpha * raw_derivative + (1.0 - self.alpha) * self.prev_derivative
        d_term = self.kd * filtered_derivative
        
        # Keep track of states for next calculation
        self.prev_error = error
        self.prev_derivative = filtered_derivative
        
        # Sum terms
        output = p_term + i_term + d_term
        
        # 4. Clamp output to control limits
        min_limit, max_limit = self.output_limits
        if min_limit is not None:
            output = max(min_limit, output)
        if max_limit is not None:
            output = min(max_limit, output)
            
        return output

    def reset(self):
        """Resets controller states (integral and derivative terms)."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0

if __name__ == '__main__':
    # Simple simulation testing the PID speed controller
    print("Testing Speed PID Controller simulation...")
    
    # Target speed: 10 m/s. Initial speed: 0 m/s
    target_speed = 10.0
    current_speed = 0.0
    
    # Controller: Kp = 1.5, Ki = 0.5, Kd = 0.1, limits = (-1.0, 1.0) (throttle/brake command)
    controller = PIDController(kp=1.5, ki=0.5, kd=0.1, output_limits=(-1.0, 1.0))
    
    dt = 0.1 # 100ms ticks
    sim_time = 3.0 # simulate 3 seconds
    steps = int(sim_time / dt)
    
    print(f"{'Time (s)':<10} | {'Current Speed (m/s)':<22} | {'Control Output':<15}")
    print("-" * 55)
    
    for step in range(steps):
        t = step * dt
        error = target_speed - current_speed
        
        # Compute command
        control_action = controller.update(error, dt)
        
        # Simple vehicle physics model: acceleration proportional to control command
        # v[t+1] = v[t] + acceleration * dt
        accel = control_action * 8.0 - (0.1 * current_speed) # add drag friction
        current_speed += accel * dt
        
        print(f"{t:<10.1f} | {current_speed:<22.4f} | {control_action:<15.4f}")
