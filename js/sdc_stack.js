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
  }

  update(carPhysics, rawDistanceCm) {
    // Localization: Fuse raw physics position with simulated sensor noise
    const noiseX = (Math.random() - 0.5) * this.sensorNoise;
    const noiseZ = (Math.random() - 0.5) * this.sensorNoise;

    const estimatedPos = {
      x: carPhysics.position.x + noiseX,
      y: carPhysics.position.y,
      z: carPhysics.position.z + noiseZ
    };

    const estimatedYaw = carPhysics.angle + (Math.random() - 0.5) * 0.01;
    const estimatedSpeed = carPhysics.speed + (Math.random() - 0.5) * 0.02;

    // Obstacle Detection (processes raw ultrasonic sensor returns)
    let detectedObstacle = null;
    if (this.sensorBlocked) {
      // Degraded sensor signal
      detectedObstacle = null;
    } else if (rawDistanceCm < 400.0) {
      // Map ray hit point back to global coordinates
      const distMeters = rawDistanceCm / 100.0;
      const angle = carPhysics.angle;
      
      const hitX = carPhysics.position.x + Math.sin(angle) * distMeters;
      const hitZ = carPhysics.position.z + Math.cos(angle) * distMeters;

      detectedObstacle = {
        x: hitX,
        z: hitZ,
        distanceCm: rawDistanceCm,
        type: rawDistanceCm < 100.0 ? 'dynamic' : 'static' // label close obstacles as dynamic threats
      };
    }

    return {
      position: estimatedPos,
      yaw: estimatedYaw,
      speed: estimatedSpeed,
      detectedObstacle,
      status: this.sensorBlocked ? 'DEGRADED' : 'OK'
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
    // 0.0 = free, 1.0 = occupied
    this.gridSize = 16;
    this.cellSize = 2.0; // meters per cell
    this.occupancyGrid = Array(this.gridSize).fill(0).map(() => Array(this.gridSize).fill(0.0));
  }

  getWaypoints() {
    return this.waypoints[this.currentMapName];
  }

  getCurrentWaypoint() {
    const list = this.getWaypoints();
    return list[this.currentWaypointIndex % list.length];
  }

  update(carPosition, detectedObstacle) {
    // 1. Decay the old grid occupancy values slowly
    for (let r = 0; r < this.gridSize; r++) {
      for (let c = 0; c < this.gridSize; c++) {
        this.occupancyGrid[r][c] = Math.max(0.0, this.occupancyGrid[r][c] - 0.05);
      }
    }

    // 2. If an obstacle is detected, map its global coordinate to the local relative grid
    if (detectedObstacle) {
      const relX = detectedObstacle.x - carPosition.x;
      const relZ = detectedObstacle.z - carPosition.z;

      // Translate to grid cell index (car is centered at index gridSize/2)
      const halfSize = this.gridSize / 2;
      const col = Math.round(halfSize + relX / this.cellSize);
      const row = Math.round(halfSize + relZ / this.cellSize);

      // Verify bounds
      if (row >= 0 && row < this.gridSize && col >= 0 && col < this.gridSize) {
        // Increment occupancy probability
        this.occupancyGrid[row][col] = Math.min(1.0, this.occupancyGrid[row][col] + 0.6);
      }
    }
  }

  changeMap(mapName) {
    if (this.waypoints[mapName]) {
      this.currentMapName = mapName;
      this.currentWaypointIndex = 0;
      // Reset Grid
      this.occupancyGrid = Array(this.gridSize).fill(0).map(() => Array(this.gridSize).fill(0.0));
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

    // Obstacle evaluation
    const obst = perception.detectedObstacle;
    const obstDist = obst ? obst.distanceCm : 999.0;

    // Behavior Planner: State Transitions
    switch (this.behaviorState) {
      case 'ROUTE_FOLLOWING':
        if (obstDist < 50) {
          this.behaviorState = 'REVERSE_RECOVERY';
          this.avoidanceTimer = 1.5; // reverse for 1.5s
        } else if (obstDist < 120) {
          this.behaviorState = 'OBSTACLE_AVOIDANCE';
          this.avoidanceTimer = 1.0;
        }
        break;

      case 'OBSTACLE_AVOIDANCE':
        this.avoidanceTimer -= dt;
        if (obstDist < 50) {
          this.behaviorState = 'REVERSE_RECOVERY';
          this.avoidanceTimer = 1.5;
        } else if (this.avoidanceTimer <= 0 && obstDist >= 150) {
          this.behaviorState = 'ROUTE_FOLLOWING';
        }
        break;

      case 'REVERSE_RECOVERY':
        this.avoidanceTimer -= dt;
        if (this.avoidanceTimer <= 0) {
          // If clear, follow route. Else steer around
          this.behaviorState = obstDist < 120 ? 'OBSTACLE_AVOIDANCE' : 'ROUTE_FOLLOWING';
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
      }
    };
  }

  // Trigger SDC stack execution loop
  tick(carPhysics, rawDistanceCm, dt) {
    if (!this.state.active) return;

    // 1. ENVIRONMENT PERCEPTION
    const perceptOutput = this.perception.update(carPhysics, rawDistanceCm);

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
