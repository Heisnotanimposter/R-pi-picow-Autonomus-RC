/**
 * SDC (Self-Driving Car) Modular Software Stack
 * Implements the 5 modules discussed in the professor's lecture:
 * 1. Environment Perception
 * 2. Environment Mapping
 * 3. Motion Planning
 * 4. Vehicle Control
 * 5. System Supervisor
 */

// --- 1. ENVIRONMENT PERCEPTION MODULE ---
class PerceptionModule {
  constructor() {
    this.sensorBlocked = false;
    this.sensorNoise = 0.05; // Simulate GPS/IMU drift noise

    // Extended Kalman Filter (EKF) State Variables
    this.x = 0.0;     // Estimated X position (meters)
    this.z = 0.0;     // Estimated Z position (meters)
    this.theta = 0.0; // Estimated Yaw angle (radians)
    this.lastGtYaw = 0.0;
    this.initialized = false;

    // EKF Covariance Matrix P (3x3)
    this.P = [
      [0.5, 0.0, 0.0],
      [0.0, 0.5, 0.0],
      [0.0, 0.0, 0.1]
    ];

    // Process Noise Covariance Q (3x3)
    this.Q = [
      [0.02, 0.0, 0.0],
      [0.0, 0.02, 0.0],
      [0.0, 0.0, 0.005]
    ];

    // Measurement Noise Covariance R (2x2) - GPS noise characteristics
    this.R = [
      [1.5, 0.0],
      [0.0, 1.5]
    ];

    this.gpsTimer = 0.0;
    this.gpsUpdateInterval = 1.0; // GPS correction every 1.0s (1Hz)
  }

  update(carPhysics, rawDistanceCm, dt) {
    if (!this.initialized) {
      this.x = carPhysics.position.x;
      this.z = carPhysics.position.z;
      this.theta = carPhysics.angle;
      this.lastGtYaw = carPhysics.angle;
      this.initialized = true;
    }

    // --- 1. EKF PREDICTION STEP ---
    // Simulate encoder velocity measurement with noise
    const v_noise = (Math.random() - 0.5) * 0.1;
    const speedMeas = carPhysics.speed + v_noise;

    // Simulate IMU gyro yaw rate measurement with noise
    let yawRate = 0.0;
    if (dt > 0.0) {
      let diff = carPhysics.angle - this.lastGtYaw;
      diff = Math.atan2(Math.sin(diff), Math.cos(diff)); // unwrap
      yawRate = diff / dt;
    }
    this.lastGtYaw = carPhysics.angle;
    const w_noise = (Math.random() - 0.5) * 0.03;
    const yawRateMeas = yawRate + w_noise;

    // Propagate kinematic motion model:
    // x(t) = x(t-1) + v * sin(theta) * dt
    // z(t) = z(t-1) + v * cos(theta) * dt
    // theta(t) = theta(t-1) + w * dt
    this.x += speedMeas * Math.sin(this.theta) * dt;
    this.z += speedMeas * Math.cos(this.theta) * dt;
    this.theta += yawRateMeas * dt;
    this.theta = Math.atan2(Math.sin(this.theta), Math.cos(this.theta)); // normalize

    // Motion Model Jacobian Fx (3x3):
    // Fx = [ 1  0  v * cos(theta) * dt ]
    //      [ 0  1 -v * sin(theta) * dt ]
    //      [ 0  0  1                   ]
    const a = speedMeas * Math.cos(this.theta) * dt;
    const b = -speedMeas * Math.sin(this.theta) * dt;

    // Predict Covariance: P = Fx * P * Fx^T + Q
    const P00 = this.P[0][0]; const P01 = this.P[0][1]; const P02 = this.P[0][2];
    const P10 = this.P[1][0]; const P11 = this.P[1][1]; const P12 = this.P[1][2];
    const P20 = this.P[2][0]; const P21 = this.P[2][1]; const P22 = this.P[2][2];

    // M = Fx * P
    const M00 = P00 + a * P20; const M01 = P01 + a * P21; const M02 = P02 + a * P22;
    const M10 = P10 + b * P20; const M11 = P11 + b * P21; const M12 = P12 + b * P22;
    const M20 = P20;           const M21 = P21;           const M22 = P22;

    // N = M * Fx^T
    const N00 = M00 + a * M02; const N01 = M01 + b * M02; const N02 = M02;
    const N10 = M10 + a * M12; const N11 = M11 + b * M12; const N12 = M12;
    const N20 = M20 + a * M22; const N21 = M21 + b * M22; const N22 = M22;

    // P = N + Q
    this.P[0][0] = N00 + this.Q[0][0]; this.P[0][1] = N01 + this.Q[0][1]; this.P[0][2] = N02 + this.Q[0][2];
    this.P[1][0] = N10 + this.Q[1][0]; this.P[1][1] = N11 + this.Q[1][1]; this.P[1][2] = N12 + this.Q[1][2];
    this.P[2][0] = N20 + this.Q[2][0]; this.P[2][1] = N21 + this.Q[2][1]; this.P[2][2] = N22 + this.Q[2][2];

    // --- 2. EKF CORRECTION STEP (GPS Update) ---
    this.gpsTimer += dt;
    let gpsUpdated = false;
    let gpsMeas = null;

    if (this.gpsTimer >= this.gpsUpdateInterval) {
      this.gpsTimer = 0.0;
      gpsUpdated = true;

      // Simulate noisy GPS measurement
      const gpsNoiseX = (Math.random() - 0.5) * 1.5;
      const gpsNoiseZ = (Math.random() - 0.5) * 1.5;
      const zx = carPhysics.position.x + gpsNoiseX;
      const zz = carPhysics.position.z + gpsNoiseZ;
      gpsMeas = { x: zx, z: zz };

      // Innovation s = z - H * x  (H = [1 0 0; 0 1 0])
      const s0 = zx - this.x;
      const s1 = zz - this.z;

      // Innovation Covariance S = H * P * H^T + R
      const S00 = this.P[0][0] + this.R[0][0];
      const S01 = this.P[0][1] + this.R[0][1];
      const S10 = this.P[1][0] + this.R[1][0];
      const S11 = this.P[1][1] + this.R[1][1];

      // Det S
      const detS = S00 * S11 - S01 * S10;
      if (Math.abs(detS) > 1e-6) {
        // S_inv
        const Sinv00 = S11 / detS;
        const Sinv01 = -S01 / detS;
        const Sinv10 = -S10 / detS;
        const Sinv11 = S00 / detS;

        // Kalman Gain K = P * H^T * S_inv
        const K00 = this.P[0][0] * Sinv00 + this.P[0][1] * Sinv10;
        const K01 = this.P[0][0] * Sinv01 + this.P[0][1] * Sinv11;
        const K10 = this.P[1][0] * Sinv00 + this.P[1][1] * Sinv10;
        const K11 = this.P[1][0] * Sinv01 + this.P[1][1] * Sinv11;
        const K20 = this.P[2][0] * Sinv00 + this.P[2][1] * Sinv10;
        const K21 = this.P[2][0] * Sinv01 + this.P[2][1] * Sinv11;

        // Update State
        this.x += K00 * s0 + K01 * s1;
        this.z += K10 * s0 + K11 * s1;
        this.theta += K20 * s0 + K21 * s1;
        this.theta = Math.atan2(Math.sin(this.theta), Math.cos(this.theta));

        // Update Covariance P = (I - K * H) * P
        const W00 = 1.0 - K00; const W01 = -K01;
        const W10 = -K10;       const W11 = 1.0 - K11;
        const W20 = -K20;       const W21 = -K21;

        const uP00 = W00 * this.P[0][0] + W01 * this.P[1][0];
        const uP01 = W00 * this.P[0][1] + W01 * this.P[1][1];
        const uP02 = W00 * this.P[0][2] + W01 * this.P[1][2];

        const uP10 = W10 * this.P[0][0] + W11 * this.P[1][0];
        const uP11 = W10 * this.P[0][1] + W11 * this.P[1][1];
        const uP12 = W10 * this.P[0][2] + W11 * this.P[1][2];

        const uP20 = W20 * this.P[0][0] + W21 * this.P[1][0] + this.P[2][0];
        const uP21 = W20 * this.P[0][1] + W21 * this.P[1][1] + this.P[2][1];
        const uP22 = W20 * this.P[0][2] + W21 * this.P[1][2] + this.P[2][2];

        this.P[0][0] = uP00; this.P[0][1] = uP01; this.P[0][2] = uP02;
        this.P[1][0] = uP10; this.P[1][1] = uP11; this.P[1][2] = uP12;
        this.P[2][0] = uP20; this.P[2][1] = uP21; this.P[2][2] = uP22;
      }
    }

    // --- 3. OBSTACLE PERCEPTION ---
    // Obstacle Detection (processes raw ultrasonic sensor returns using ESTIMATED state)
    let detectedObstacle = null;
    if (this.sensorBlocked) {
      detectedObstacle = null;
    } else if (rawDistanceCm < 400.0) {
      const distMeters = rawDistanceCm / 100.0;
      
      // Project ray using Estimated Position & Heading
      const hitX = this.x + Math.sin(this.theta) * distMeters;
      const hitZ = this.z + Math.cos(this.theta) * distMeters;

      detectedObstacle = {
        x: hitX,
        z: hitZ,
        distanceCm: rawDistanceCm,
        type: rawDistanceCm < 100.0 ? 'dynamic' : 'static'
      };
    }

    return {
      position: { x: this.x, y: carPhysics.position.y, z: this.z },
      yaw: this.theta,
      speed: speedMeas,
      detectedObstacle,
      status: this.sensorBlocked ? 'DEGRADED' : 'OK',
      P: this.P,
      gpsUpdated,
      gpsMeas
    };
  }
}

// --- 2. ENVIRONMENT MAPPING MODULE ---
class MappingModule {
  constructor() {
    // Detailed Road Map Waypoints (Loop around KNUT Chungju campus)
    this.waypoints = {
      'knut-chungju': [
        { x: 0, z: 80 },    // E1 Main Gate Start
        { x: 35, z: 20 },   // E6 Student Center
        { x: 35, z: -20 },  // E8 Administration
        { x: 40, z: -60 },  // E17 Smart ICT Hall
        { x: 0, z: 15 },    // Central Intersection
        { x: -40, z: -60 }, // W20 Central Library
        { x: -35, z: -20 }, // W16 IT Building
        { x: -40, z: 15 },  // W10 Dorms
      ],
      'knut-uiwang': [
        { x: 0, z: 60 },
        { x: 30, z: 30 },
        { x: 30, z: -30 },
        { x: 0, z: -60 },
        { x: -30, z: -30 },
        { x: -30, z: 30 },
      ],
      'obstacle-course': [
        { x: 0, z: 60 },
        { x: 60, z: 60 },
        { x: 60, z: -60 },
        { x: -60, z: -60 },
        { x: -60, z: 60 },
      ]
    };

    this.currentMapName = 'knut-chungju';
    this.currentWaypointIndex = 0;

    // 16x16 Occupancy Grid centered on vehicle (each cell is 2m x 2m)
    this.gridSize = 16;
    this.cellSize = 2.0; // meters per cell
    this.occupancyGrid = Array(this.gridSize).fill(0).map(() => Array(this.gridSize).fill(0.0));
    // Log-Odds grid representation: 0.0 corresponds to prior probability 0.5 (unknown)
    this.logOddsGrid = Array(this.gridSize).fill(0).map(() => Array(this.gridSize).fill(0.0));
  }

  getWaypoints() {
    return this.waypoints[this.currentMapName];
  }

  getCurrentWaypoint() {
    const list = this.getWaypoints();
    return list[this.currentWaypointIndex % list.length];
  }

  // Bresenham's line tracing algorithm for raycast updates
  traceRay(x0, y0, x1, y1) {
    const cells = [];
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;
    let err = dx - dy;

    let x = x0;
    let y = y0;

    while (true) {
      if (x === x1 && y === y1) {
        break; // skip endpoint (obstacle itself)
      }
      cells.push({ r: y, c: x });

      const e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x += sx;
      }
      if (e2 < dx) {
        err += dx;
        y += sy;
      }
    }
    return cells;
  }

  update(carPosition, detectedObstacle) {
    // 1. Decelerate/decay the old grid log-odds values slowly towards 0 (representing drift/unknown)
    for (let r = 0; r < this.gridSize; r++) {
      for (let c = 0; c < this.gridSize; c++) {
        this.logOddsGrid[r][c] *= 0.85;
      }
    }

    const halfSize = this.gridSize / 2;

    // 2. Inverse Sensor Model Log-Odds Updates
    if (detectedObstacle) {
      const relX = detectedObstacle.x - carPosition.x;
      const relZ = detectedObstacle.z - carPosition.z;

      // Translate to relative grid coordinates
      const col = Math.round(halfSize + relX / this.cellSize);
      const row = Math.round(halfSize + relZ / this.cellSize);

      // Verify boundary
      if (row >= 0 && row < this.gridSize && col >= 0 && col < this.gridSize) {
        // Trace line from center (vehicle bumper/origin) to obstacle cell
        const freeCells = this.traceRay(halfSize, halfSize, col, row);

        // Update traversed cells as free (subtract log-odds)
        freeCells.forEach(cell => {
          if (cell.r >= 0 && cell.r < this.gridSize && cell.c >= 0 && cell.c < this.gridSize) {
            this.logOddsGrid[cell.r][cell.c] = Math.max(-5.0, this.logOddsGrid[cell.r][cell.c] - 0.4);
          }
        });

        // Update hit cell as occupied (add log-odds)
        this.logOddsGrid[row][col] = Math.min(5.0, this.logOddsGrid[row][col] + 1.5);
      }
    }

    // 3. Map log-odds back to probability values P = 1 - 1/(1 + exp(L))
    for (let r = 0; r < this.gridSize; r++) {
      for (let c = 0; c < this.gridSize; c++) {
        const lo = this.logOddsGrid[r][c];
        const p = 1.0 - 1.0 / (1.0 + Math.exp(lo));
        
        // Output visualization: map [0.5, 1.0] -> [0.0, 1.0] for occupied rendering
        this.occupancyGrid[r][c] = Math.max(0.0, (p - 0.5) * 2.0);
      }
    }
  }

  changeMap(mapName) {
    if (this.waypoints[mapName]) {
      this.currentMapName = mapName;
      this.currentWaypointIndex = 0;
      // Reset Grids
      this.occupancyGrid = Array(this.gridSize).fill(0).map(() => Array(this.gridSize).fill(0.0));
      this.logOddsGrid = Array(this.gridSize).fill(0).map(() => Array(this.gridSize).fill(0.0));
    }
  }
}

// --- 3. MOTION PLANNING MODULE ---
class PlanningModule {
  constructor() {
    this.behaviorState = 'ROUTE_FOLLOWING'; // ROUTE_FOLLOWING, OBSTACLE_AVOIDANCE, REVERSE_RECOVERY, ESTOP_FAULT
    this.waypointThreshold = 8.0; // target waypoint radius (m)
    this.avoidanceTimer = 0;
  }

  update(perception, mapping, dt) {
    if (perception.status === 'DEGRADED') {
      this.behaviorState = 'ESTOP_FAULT';
      return { targetSpeed: 0, targetAngle: perception.yaw, state: this.behaviorState };
    }

    const carPos = perception.position;
    const targetWp = mapping.getCurrentWaypoint();
    const distanceToWp = Math.sqrt(
      Math.pow(targetWp.x - carPos.x, 2) + Math.pow(targetWp.z - carPos.z, 2)
    );

    // Mission Planner: Increment waypoint if close enough
    if (distanceToWp < this.waypointThreshold) {
      mapping.currentWaypointIndex = (mapping.currentWaypointIndex + 1) % mapping.getWaypoints().length;
    }

    const obst = perception.detectedObstacle;
    const obstDist = obst ? obst.distanceCm : 999.0;

    // --- GRID-BASED LOOK-AHEAD PLANNER CHECK ---
    // Project search cells directly in front of the vehicle along its estimated heading
    const halfSize = mapping.gridSize / 2;
    const yaw = perception.yaw;
    let gridObstacleDetected = false;

    for (let step = 1; step <= 3; step++) {
      const col = Math.round(halfSize + Math.sin(yaw) * step);
      const row = Math.round(halfSize + Math.cos(yaw) * step);
      if (row >= 0 && row < mapping.gridSize && col >= 0 && col < mapping.gridSize) {
        if (mapping.occupancyGrid[row][col] > 0.35) {
          gridObstacleDetected = true;
          break;
        }
      }
    }

    // Behavior Planner: State Transitions
    switch (this.behaviorState) {
      case 'ROUTE_FOLLOWING':
        if (gridObstacleDetected || obstDist < 120) {
          if (obstDist < 50) {
            this.behaviorState = 'REVERSE_RECOVERY';
            this.avoidanceTimer = 1.5; // reverse for 1.5s
          } else {
            this.behaviorState = 'OBSTACLE_AVOIDANCE';
            this.avoidanceTimer = 1.2;
          }
        }
        break;

      case 'OBSTACLE_AVOIDANCE':
        this.avoidanceTimer -= dt;
        if (obstDist < 50) {
          this.behaviorState = 'REVERSE_RECOVERY';
          this.avoidanceTimer = 1.5;
        } else if (this.avoidanceTimer <= 0 && !gridObstacleDetected && obstDist >= 150) {
          this.behaviorState = 'ROUTE_FOLLOWING';
        }
        break;

      case 'REVERSE_RECOVERY':
        this.avoidanceTimer -= dt;
        if (this.avoidanceTimer <= 0) {
          // If clear, follow route. Else steer around
          this.behaviorState = (gridObstacleDetected || obstDist < 120) ? 'OBSTACLE_AVOIDANCE' : 'ROUTE_FOLLOWING';
          this.avoidanceTimer = 1.0;
        }
        break;

      case 'ESTOP_FAULT':
        if (perception.status === 'OK') {
          this.behaviorState = 'ROUTE_FOLLOWING';
        }
        break;
    }

    // Local Planner: Output trajectory targets based on state
    let targetSpeed = 8.0; // default m/s
    let targetAngle = Math.atan2(targetWp.x - carPos.x, targetWp.z - carPos.z);

    if (this.behaviorState === 'OBSTACLE_AVOIDANCE') {
      targetSpeed = 3.5;
      // Steer hard to the side (skid turn)
      targetAngle = perception.yaw + Math.PI / 2.2; // turn left relative to current direction
    } else if (this.behaviorState === 'REVERSE_RECOVERY') {
      targetSpeed = -4.0; // reverse speed
      targetAngle = perception.yaw - Math.PI / 6; // reverse turning
    } else if (this.behaviorState === 'ESTOP_FAULT') {
      targetSpeed = 0;
      targetAngle = perception.yaw;
    } else {
      // Route Following: Slow down slightly in sharp curves
      const headingDiff = Math.abs(targetAngle - perception.yaw);
      if (headingDiff > 0.5) {
        targetSpeed = 5.0; // curve speed limit
      }
    }

    return {
      targetSpeed,
      targetAngle,
      state: this.behaviorState
    };
  }
}

// --- 4. VEHICLE CONTROL MODULE ---
class ControlModule {
  constructor() {
    this.kp_yaw = 2.5; // proportional steering gain
    this.kp_speed = 0.6; // proportional speed gain
    this.lastCommand = '/stop?';
  }

  update(perception, targets) {
    const currentSpeed = perception.speed;
    const currentYaw = perception.yaw;
    const targetSpeed = targets.targetSpeed;
    const targetAngle = targets.targetAngle;

    // 1. Longitudinal Controller (proportional velocity error feedback)
    const speedError = targetSpeed - currentSpeed;
    
    // PWM duty cycle calculations (regulate speed)
    let targetPwm = 70; // baseline
    if (targetSpeed !== 0) {
      // Boost PWM to reduce error on acceleration, scale down when speed matches
      targetPwm = Math.min(100, Math.max(30, Math.round(50 + Math.abs(speedError) * 10)));
    }

    // 2. Lateral Controller (calculates steering yaw corrections)
    let yawError = targetAngle - currentYaw;
    // Normalize yaw error between -PI and PI
    yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));

    // Determine H-Bridge Socket command matching physical hardware
    let command = '/stop?';

    if (targetSpeed === 0) {
      command = '/stop?';
    } else if (targetSpeed < 0) {
      command = '/back?'; // reversing
    } else {
      // Forward path control: Steer based on heading offset
      // Threshold: if angle error is significant (> 15 degrees)
      if (yawError > 0.26) {
        command = '/left?'; // steer left
      } else if (yawError < -0.26) {
        command = '/right?'; // steer right
      } else {
        command = '/forward?'; // stay straight
      }
    }

    this.lastCommand = command;

    return {
      command,
      pwmPercent: targetPwm,
      yawError,
      speedError
    };
  }
}

// --- 5. SYSTEM SUPERVISOR MODULE ---
class SupervisorModule {
  constructor() {
    this.watchdogFreq = 10; // Expected loops per second (10Hz)
    this.tickCount = 0;
    this.elapsedTime = 0;
    this.hardwareFaults = [];
  }

  update(perception, dt) {
    this.elapsedTime += dt;
    this.tickCount++;

    const statusCheck = {
      IMU: 'OK',
      GPS: 'OK',
      Ultrasonic: 'OK',
      CPU_Rate: '10Hz',
      ActiveWarnings: 'NONE'
    };

    this.hardwareFaults = [];

    // Check sensor status
    if (perception.status === 'DEGRADED') {
      this.hardwareFaults.push('ULTRASONIC_SENSOR_BLOCKED');
      statusCheck.Ultrasonic = 'FAULT';
      statusCheck.ActiveWarnings = 'SENSOR_BLOCKAGE';
    }

    // Check watchdog timer
    if (this.elapsedTime >= 1.0) {
      const actualFreq = this.tickCount / this.elapsedTime;
      if (actualFreq < this.watchdogFreq * 0.8) {
        this.hardwareFaults.push('CPU_FREQUENCY_LAG');
        statusCheck.CPU_Rate = `${actualFreq.toFixed(1)}Hz (LAG)`;
      }
      this.tickCount = 0;
      this.elapsedTime = 0;
    }

    return {
      status: this.hardwareFaults.length > 0 ? 'CRITICAL_FAULT' : 'SYSTEM_HEALTHY',
      faults: this.hardwareFaults,
      checklist: statusCheck
    };
  }
}

// --- SDC FULL SOFTWARE STACK ORCHESTRATOR ---
class SDCSystem {
  constructor(picoVM) {
    this.picoVM = picoVM;
    
    // Instantiate modular components
    this.perception = new PerceptionModule();
    this.mapping = new MappingModule();
    this.planner = new PlanningModule();
    this.controller = new ControlModule();
    this.supervisor = new SupervisorModule();

    // SDC telemetry cache
    this.state = {
      active: false,
      behavior: 'ROUTE_FOLLOWING',
      cte: 0.0,
      targetSpeed: 0.0,
      commandSent: '/stop?',
      supervisorStatus: 'SYSTEM_HEALTHY',
      checklist: {
        IMU: 'OK',
        GPS: 'OK',
        Ultrasonic: 'OK',
        CPU_Rate: '10Hz',
        ActiveWarnings: 'NONE'
      },
      ekfP: null,
      ekfPos: null,
      gpsMeas: null,
      gpsUpdated: false
    };
  }

  // Trigger SDC stack execution loop
  tick(carPhysics, rawDistanceCm, dt) {
    if (!this.state.active) return;

    // 1. ENVIRONMENT PERCEPTION (Pass actual DT for EKF kinematics)
    const perceptOutput = this.perception.update(carPhysics, rawDistanceCm, dt);

    // 2. ENVIRONMENT MAPPING
    this.mapping.update(perceptOutput.position, perceptOutput.detectedObstacle);

    // 3. MOTION PLANNING
    const plannerOutput = this.planner.update(perceptOutput, this.mapping, dt);

    // 4. VEHICLE CONTROL
    const controlOutput = this.controller.update(perceptOutput, plannerOutput);

    // 5. SYSTEM SUPERVISOR
    const supervisorOutput = this.supervisor.update(perceptOutput, dt);

    // Apply control decisions to virtual Pico VM (triggers GPIO lights & motor logic)
    if (this.state.active) {
      // Regulate PWM speed
      this.picoVM.setPWM(controlOutput.pwmPercent);
      // Send socket command
      this.picoVM.receiveRequest(controlOutput.command);
    }

    // Save telemetry logs
    this.state.behavior = plannerOutput.state;
    this.state.cte = controlOutput.yawError; // Cross-track alignment error
    this.state.targetSpeed = plannerOutput.targetSpeed;
    this.state.commandSent = controlOutput.command;
    this.state.supervisorStatus = supervisorOutput.status;
    this.state.checklist = supervisorOutput.checklist;
    this.state.ekfP = perceptOutput.P;
    this.state.ekfPos = perceptOutput.position;
    this.state.gpsMeas = perceptOutput.gpsMeas;
    this.state.gpsUpdated = perceptOutput.gpsUpdated;
  }

  setSensorFault(isBlocked) {
    this.perception.sensorBlocked = isBlocked;
    if (isBlocked) {
      this.picoVM.log('[수퍼바이저 경고] 비주얼 센서가 차단됨! 시스템 차단 모드 동작.', 'error');
    } else {
      this.picoVM.log('[수퍼바이저 정보] 비주얼 센서 차단이 해제되었습니다.', 'success');
    }
  }

  changeMap(mapName) {
    this.mapping.changeMap(mapName);
  }
}

// Export for browser usage
window.SDCSystem = SDCSystem;
