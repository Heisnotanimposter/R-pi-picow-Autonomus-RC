/**
 * Main UI Orchestration and Glue Layer
 * Handles bindings, inputs, telemetry rendering, and autonomous loops.
 */
document.addEventListener('DOMContentLoaded', () => {
  
  // Elements
  const picoConsole = document.getElementById('pico-console');
  const btnClearConsole = document.getElementById('btn-clear-console');
  const mapSelect = document.getElementById('map-select');
  const picoIpSpan = document.getElementById('pico-ip');
  
  // Telemetry Elements
  const valSpeed = document.getElementById('val-speed');
  const valSteer = document.getElementById('val-steer');
  const valDistance = document.getElementById('val-distance');
  const valHeading = document.getElementById('val-heading');
  const valCoords = document.getElementById('val-coords');
  
  // Control Panel Elements
  const btnForward = document.getElementById('ctrl-forward');
  const btnLeft = document.getElementById('ctrl-left');
  const btnStop = document.getElementById('ctrl-stop');
  const btnRight = document.getElementById('ctrl-right');
  const btnBack = document.getElementById('ctrl-back');
  const sliderPwm = document.getElementById('slider-pwm');
  const labelPwmVal = document.getElementById('pwm-value');
  const chkAutonomous = document.getElementById('chk-autonomous');
  
  // Breadboard Elements
  const ledGP2 = document.getElementById('led-gp2');
  const ledGP3 = document.getElementById('led-gp3');
  const ledGP7 = document.getElementById('led-gp7');
  const ledGP8 = document.getElementById('led-gp8');
  const ledGP5 = document.getElementById('led-gp5');
  const ledGP9 = document.getElementById('led-gp9');
  const pwmBarA = document.getElementById('pwm-bar-a');
  const pwmBarB = document.getElementById('pwm-bar-b');
  const txtMotorA = document.getElementById('txt-motor-a');
  const txtMotorB = document.getElementById('txt-motor-b');

  // Camera Buttons
  const btnCamOrbit = document.getElementById('btn-cam-orbit');
  const btnCamFpv = document.getElementById('btn-cam-fpv');
  const btnCamChase = document.getElementById('btn-cam-chase');
  const btnCamTop = document.getElementById('btn-cam-top');
  
  // Environment Controls
  const btnEnvDay = document.getElementById('btn-env-day');
  const btnEnvSunset = document.getElementById('btn-env-sunset');
  const btnEnvNight = document.getElementById('btn-env-night');
  const chkHeadlights = document.getElementById('chk-headlights');
  const chkLaser = document.getElementById('chk-laser');
  const sliderFriction = document.getElementById('slider-friction');

  // GPS Reference Coordinates
  const COORDINATES = {
    'knut-chungju': { lat: 36.969722, lon: 127.871389 }, // Main campus
    'knut-uiwang': { lat: 37.316667, lon: 126.949722 },  // Railroad campus
    'obstacle-course': { lat: 37.5665, lon: 126.9780 }   // Seoul center grid
  };

  // State Variables
  let autoNavTimer = null;
  let activeKeys = { w: false, a: false, s: false, d: false, ArrowUp: false, ArrowDown: false, ArrowLeft: false, ArrowRight: false };

  // 1. Initialize Pico Virtual Machine
  const picoVM = new PicoVM(
    // Logger Callback
    (msg, type) => {
      const line = document.createElement('div');
      line.className = `log-line ${type}`;
      line.innerText = msg;
      picoConsole.appendChild(line);
      picoConsole.scrollTop = picoConsole.scrollHeight; // Auto-scroll
    },
    // GPIO Callback
    (gpio) => {
      updateBreadboardUI(gpio);
    }
  );

  // 2. Initialize 3D Simulation Engine
  const simulation = new SimulationEngine(
    'main-3d-viewport',
    'fpv-viewport',
    picoVM,
    // Telemetry Callback
    (telemetry) => {
      updateTelemetryUI(telemetry);
    }
  );

  // 2.5 Initialize SDC Modular Software Stack
  const sdc = new SDCSystem(picoVM);

  // Start virtual board connection boot sequence and draw waypoints
  picoVM.boot().then(() => {
    picoIpSpan.innerText = picoVM.ip;
    
    // Sync initial map waypoints
    sdc.changeMap(mapSelect.value);
    simulation.drawWaypoints(sdc.mapping.getWaypoints());
  });

  // 3. UI Bindings - Controls
  const sendCommand = (path) => {
    picoVM.receiveRequest(path);
  };

  btnForward.addEventListener('mousedown', () => sendCommand('/forward?'));
  btnBack.addEventListener('mousedown', () => sendCommand('/back?'));
  btnLeft.addEventListener('mousedown', () => sendCommand('/left?'));
  btnRight.addEventListener('mousedown', () => sendCommand('/right?'));
  btnStop.addEventListener('mousedown', () => sendCommand('/stop?'));

  // Reset to stop on mouseup
  const stopOnRelease = (e) => {
    if (e.target.classList.contains('btn-ctrl') && e.target.id !== 'ctrl-stop') {
      sendCommand('/stop?');
    }
  };
  document.addEventListener('mouseup', stopOnRelease);

  // PWM slider
  sliderPwm.addEventListener('input', (e) => {
    const val = e.target.value;
    labelPwmVal.innerText = `${val}%`;
    picoVM.setPWM(val);
  });

  // Map dropdown switcher
  mapSelect.addEventListener('change', (e) => {
    const mapName = e.target.value;
    simulation.loadMap(mapName);
    
    // Update SDC waypoints
    sdc.changeMap(mapName);
    simulation.drawWaypoints(sdc.mapping.getWaypoints());
    
    picoVM.log(`[시스템] 맵 변경됨: ${mapName.toUpperCase()}`, 'info');
  });

  // Clear Console logs
  btnClearConsole.addEventListener('click', () => {
    picoConsole.innerHTML = '<div class="log-line text-muted">[시스템] 로그 콘솔이 초기화되었습니다.</div>';
  });

  // 4. Keyboard Controls Handler (Mimicking network frames)
  document.addEventListener('keydown', (e) => {
    if (e.repeat) return; // Prevent fire on hold
    
    // Ignore keyboard input when active in input text fields if any
    if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'SELECT') {
      return;
    }

    if (e.key === 'w' || e.key === 'ArrowUp') {
      activeKeys.w = true;
      sendCommand('/forward?');
      highlightBtn(btnForward);
    } else if (e.key === 's' || e.key === 'ArrowDown') {
      activeKeys.s = true;
      sendCommand('/back?');
      highlightBtn(btnBack);
    } else if (e.key === 'a' || e.key === 'ArrowLeft') {
      activeKeys.a = true;
      sendCommand('/left?');
      highlightBtn(btnLeft);
    } else if (e.key === 'd' || e.key === 'ArrowRight') {
      activeKeys.d = true;
      sendCommand('/right?');
      highlightBtn(btnRight);
    } else if (e.key === ' ') {
      sendCommand('/stop?');
      highlightBtn(btnStop);
    }
  });

  document.addEventListener('keyup', (e) => {
    if (e.key === 'w' || e.key === 'ArrowUp') {
      activeKeys.w = false;
      dehighlightBtn(btnForward);
    } else if (e.key === 's' || e.key === 'ArrowDown') {
      activeKeys.s = false;
      dehighlightBtn(btnBack);
    } else if (e.key === 'a' || e.key === 'ArrowLeft') {
      activeKeys.a = false;
      dehighlightBtn(btnLeft);
    } else if (e.key === 'd' || e.key === 'ArrowRight') {
      activeKeys.d = false;
      dehighlightBtn(btnRight);
    } else if (e.key === ' ') {
      dehighlightBtn(btnStop);
    }

    // Stop if no keys are pressed
    if (!activeKeys.w && !activeKeys.a && !activeKeys.s && !activeKeys.d) {
      sendCommand('/stop?');
    }
  });

  const highlightBtn = (btn) => {
    btn.classList.add('active');
  };
  const dehighlightBtn = (btn) => {
    btn.classList.remove('active');
  };

  // 5. Autonomous Navigation (SDC Modular Software Stack loop)
  chkAutonomous.addEventListener('change', (e) => {
    const isChecked = e.target.checked;
    const autoHudIndicator = document.getElementById('indicator-auto').querySelector('.dot');
    
    if (isChecked) {
      autoHudIndicator.classList.add('active');
      picoVM.log('[시스템] 자율주행 모듈 소프트웨어 스택 활성화', 'success');
      startAutonomousMode();
    } else {
      autoHudIndicator.classList.remove('active');
      picoVM.log('[시스템] 자율주행 모듈 소프트웨어 스택 비활성화', 'info');
      stopAutonomousMode();
    }
  });

  // Render 2D Occupancy Grid Radar
  const drawOccupancyGrid = (grid) => {
    const canvas = document.getElementById('occupancy-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const size = canvas.width;
    const gridCount = grid.length;
    const cellSize = size / gridCount;

    ctx.clearRect(0, 0, size, size);

    // Draw grid cells with obstacle probability
    for (let r = 0; r < gridCount; r++) {
      for (let c = 0; c < gridCount; c++) {
        const prob = grid[r][c];
        if (prob > 0.0) {
          // Glow cyan
          ctx.fillStyle = `rgba(0, 242, 254, ${prob * 0.85})`;
          ctx.fillRect(c * cellSize, r * cellSize, cellSize, cellSize);
        }
        
        // Faint border lines
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
        ctx.strokeRect(c * cellSize, r * cellSize, cellSize, cellSize);
      }
    }

    // Draw robot in the center
    const center = size / 2;
    ctx.fillStyle = '#ff4757'; // Red dot for robot position
    ctx.beginPath();
    ctx.arc(center, center, 3.5, 0, Math.PI * 2);
    ctx.fill();

    // Draw heading sweep arrow
    const heading = simulation.car.angle;
    ctx.strokeStyle = '#ff4757';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(center, center);
    ctx.lineTo(center + Math.sin(heading) * 12, center + Math.cos(heading) * 12);
    ctx.stroke();
  };

  const startAutonomousMode = () => {
    sdc.state.active = true;
    
    // UI state badge
    const badge = document.getElementById('sdc-state-badge');
    badge.innerText = 'ACTIVE';
    badge.className = 'badge badge-green';

    // 10Hz SDC execution tick (100ms interval)
    autoNavTimer = setInterval(() => {
      // Execute 5 SDC Modules tick
      sdc.tick(simulation.car, simulation.sensorDistance, 0.1);

      // Render 2D occupancy grid sweep
      drawOccupancyGrid(sdc.mapping.occupancyGrid);

      // Update SDC status readouts
      document.getElementById('sdc-behavior-state').innerText = sdc.state.behavior;
      
      const cteSpan = document.getElementById('sdc-cte');
      cteSpan.innerText = sdc.state.cte.toFixed(2);
      if (Math.abs(sdc.state.cte) > 0.4) {
        cteSpan.className = 'font-mono text-danger'; // high error
      } else {
        cteSpan.className = 'font-mono text-success';
      }

      document.getElementById('sdc-target-speed').innerText = `${sdc.state.targetSpeed.toFixed(1)} m/s`;

      // Update System Supervisor checklists
      const checklist = sdc.state.checklist;
      toggleChecklistDot('chk-gps', checklist.GPS === 'OK');
      toggleChecklistDot('chk-imu', checklist.IMU === 'OK');
      toggleChecklistDot('chk-ultra', checklist.Ultrasonic === 'OK');
      toggleChecklistDot('chk-cpu', checklist.CPU_Rate.includes('10Hz'));
      
    }, 100);
  };

  const toggleChecklistDot = (elementId, isHealthy) => {
    const dot = document.getElementById(elementId);
    if (!dot) return;
    if (isHealthy) {
      dot.classList.add('active');
    } else {
      dot.classList.remove('active');
    }
  };

  const stopAutonomousMode = () => {
    sdc.state.active = false;
    
    if (autoNavTimer) {
      clearInterval(autoNavTimer);
      autoNavTimer = null;
    }

    // Reset SDC elements
    const badge = document.getElementById('sdc-state-badge');
    badge.innerText = 'INACTIVE';
    badge.className = 'badge';
    
    document.getElementById('sdc-behavior-state').innerText = 'INACTIVE';
    document.getElementById('sdc-cte').innerText = '0.00';
    document.getElementById('sdc-cte').className = 'font-mono text-info';
    document.getElementById('sdc-target-speed').innerText = '0.0 m/s';

    // Clear Canvas
    const canvas = document.getElementById('occupancy-canvas');
    if (canvas) {
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);
    }

    // Clear Supervisor dot checklists
    ['chk-gps', 'chk-imu', 'chk-ultra', 'chk-cpu'].forEach(id => {
      const dot = document.getElementById(id);
      if (dot) dot.classList.add('active'); // show green as default when idle
    });

    // Return VM speed control to slider value
    picoVM.setPWM(parseInt(sliderPwm.value));
    sendCommand('/stop?');
  };

  // Bind Trigger Sensor Blockage button
  const btnFaultTrigger = document.getElementById('btn-trigger-fault');
  btnFaultTrigger.addEventListener('click', () => {
    const isCurrentlyBlocked = sdc.perception.sensorBlocked;
    const shouldBlock = !isCurrentlyBlocked;
    
    sdc.setSensorFault(shouldBlock);

    if (shouldBlock) {
      btnFaultTrigger.innerText = 'CLEAR SENSOR BLOCKAGE';
      btnFaultTrigger.className = 'btn-small text-success';
    } else {
      btnFaultTrigger.innerText = 'TRIGGER SENSOR BLOCKAGE';
      btnFaultTrigger.className = 'btn-small text-danger';
    }
  });

  // 6. Camera Viewport Swapping
  const selectCamMode = (mode, activeBtn) => {
    simulation.cameraMode = mode;
    
    // UI class updates
    [btnCamOrbit, btnCamFpv, btnCamChase, btnCamTop].forEach(btn => {
      btn.classList.remove('active');
    });
    activeBtn.classList.add('active');
  };

  btnCamOrbit.addEventListener('click', () => selectCamMode('orbit', btnCamOrbit));
  btnCamFpv.addEventListener('click', () => selectCamMode('fpv', btnCamFpv));
  btnCamChase.addEventListener('click', () => selectCamMode('chase', btnCamChase));
  btnCamTop.addEventListener('click', () => selectCamMode('top', btnCamTop));

  // 7. Environment & Settings Switchers
  btnEnvDay.addEventListener('click', () => {
    simulation.updateEnvironmentTime('day');
    btnEnvDay.classList.add('active');
    btnEnvSunset.classList.remove('active');
    btnEnvNight.classList.remove('active');
  });
  btnEnvSunset.addEventListener('click', () => {
    simulation.updateEnvironmentTime('sunset');
    btnEnvDay.classList.remove('active');
    btnEnvSunset.classList.add('active');
    btnEnvNight.classList.remove('active');
  });
  btnEnvNight.addEventListener('click', () => {
    simulation.updateEnvironmentTime('night');
    btnEnvDay.classList.remove('active');
    btnEnvSunset.classList.remove('active');
    btnEnvNight.classList.add('active');
  });

  chkHeadlights.addEventListener('change', (e) => {
    simulation.toggleHeadlightsState(e.target.checked);
  });

  chkLaser.addEventListener('change', (e) => {
    simulation.showLaser = e.target.checked;
    const hudIndicator = document.getElementById('indicator-laser');
    if (e.target.checked) {
      hudIndicator.querySelector('.dot').classList.add('active');
    } else {
      hudIndicator.querySelector('.dot').classList.remove('active');
    }
  });

  sliderFriction.addEventListener('input', (e) => {
    // scale range from 0.1 to 1.5
    const coeff = e.target.value / 5.0; // 5 is base
    simulation.frictionCoeff = coeff;
  });

  // 8. Update Breadboard / GPIO Pin visualizer
  const updateBreadboardUI = (gpio) => {
    // GP2/3/7/8 direction status (0 = low, 1 = high)
    gpio.GP2 === 1 ? ledGP2.classList.add('active') : ledGP2.classList.remove('active');
    gpio.GP3 === 1 ? ledGP3.classList.add('active') : ledGP3.classList.remove('active');
    gpio.GP7 === 1 ? ledGP7.classList.add('active') : ledGP7.classList.remove('active');
    gpio.GP8 === 1 ? ledGP8.classList.add('active') : ledGP8.classList.remove('active');

    // GP5/9 (Trig/Echo) visual flashing
    gpio.GP5 === 1 ? ledGP5.classList.add('active-trig') : ledGP5.classList.remove('active-trig');
    gpio.GP9 === 1 ? ledGP9.classList.add('active-echo') : ledGP9.classList.remove('active-echo');

    // PWM bar widths (GP4/GP6) representing duty cycle percentage
    const widthA = `${Math.round((gpio.GP4 / 65535) * 100)}%`;
    const widthB = `${Math.round((gpio.GP6 / 65535) * 100)}%`;
    pwmBarA.style.width = widthA;
    pwmBarB.style.width = widthB;

    // L298N Driver status updates based on logic pins
    // Motor A (Left)
    if (gpio.GP3 === 1 && gpio.GP2 === 0) {
      txtMotorA.innerText = 'FORWARD';
      txtMotorA.className = 'status forward';
    } else if (gpio.GP3 === 0 && gpio.GP2 === 1) {
      txtMotorA.innerText = 'REVERSE';
      txtMotorA.className = 'status backward';
    } else {
      txtMotorA.innerText = 'STOP';
      txtMotorA.className = 'status text-muted';
    }

    // Motor B (Right)
    if (gpio.GP8 === 1 && gpio.GP7 === 0) {
      txtMotorB.innerText = 'FORWARD';
      txtMotorB.className = 'status forward';
    } else if (gpio.GP8 === 0 && gpio.GP7 === 1) {
      txtMotorB.innerText = 'REVERSE';
      txtMotorB.className = 'status backward';
    } else {
      txtMotorB.innerText = 'STOP';
      txtMotorB.className = 'status text-muted';
    }
  };

  // 9. Update Telemetry Panel UI & GPS calculations
  const updateTelemetryUI = (telemetry) => {
    valSpeed.innerText = `${telemetry.speed} m/s`;
    
    // Display steer angle
    const steerVal = parseInt(telemetry.steer);
    valSteer.innerText = steerVal === 0 ? '0°' : (steerVal > 0 ? `L ${steerVal}°` : `R ${Math.abs(steerVal)}°`);

    // Distance rendering (LiDAR is in meters for Track A, Ultrasonic is in cm for Track B)
    if (telemetry.trackAActive) {
      const distM = (telemetry.distance / 100.0);
      valDistance.innerText = `${distM.toFixed(1)} m`;
      if (distM < 5.0) {
        valDistance.className = 'value text-danger font-mono';
      } else {
        valDistance.className = 'value text-success font-mono';
      }
    } else {
      if (telemetry.distance >= 400) {
        valDistance.innerText = 'Out of Range';
        valDistance.className = 'value text-muted font-mono';
      } else if (telemetry.distance < 50) {
        valDistance.innerText = `${telemetry.distance} cm`;
        valDistance.className = 'value text-danger font-mono';
      } else {
        valDistance.innerText = `${telemetry.distance} cm`;
        valDistance.className = 'value text-warning font-mono';
      }
    }

    valHeading.innerText = `${telemetry.heading}°`;

    // GPS Latitude/Longitude displacement calculations
    const ref = COORDINATES[mapSelect.value];
    
    // 1 meter offset is approx 0.000009 deg Lat, 0.000011 deg Lon (in South Korea coordinates)
    const latOffset = telemetry.z * -0.000009; // South in ThreeJS is +z, so north is -z
    const lonOffset = telemetry.x * 0.000011;
    
    const currentLat = ref.lat + latOffset;
    const currentLon = ref.lon + lonOffset;

    valCoords.innerText = `${currentLat.toFixed(6)}°N, ${currentLon.toFixed(6)}°E`;
  };

  // --- TRACK A DYNAMIC UI GLUE AND INTERACTIVE LOOPS ---
  const tabTrackB = document.getElementById('tab-track-b');
  const tabTrackA = document.getElementById('tab-track-a');
  
  const trackBSdcBody = document.getElementById('track-b-sdc-body');
  const trackASdcBody = document.getElementById('track-a-sdc-body');
  
  const ros2Console = document.getElementById('ros2-console');
  const consoleTitle = document.getElementById('console-title');
  
  const trackBControlsBody = document.getElementById('track-b-controls-body');
  const trackAControlsBody = document.getElementById('track-a-controls-body');
  const controlsTitle = document.getElementById('controls-title');
  
  const trackBHardwareBody = document.getElementById('track-b-hardware-body');
  const trackAHardwareBody = document.getElementById('track-a-hardware-body');
  const hardwareTitle = document.getElementById('hardware-title');
  const hardwareBadge = document.getElementById('hardware-badge');

  // Track A Controllers and parameter references
  const chkRos2Autonomous = document.getElementById('chk-ros2-autonomous');
  const ctrlLawSelect = document.getElementById('ctrl-law-select');
  const stanleyGains = document.getElementById('stanley-gains');
  const pidGains = document.getElementById('pid-gains');
  
  const sliderStanleyK = document.getElementById('slider-stanley-k');
  const labelStanleyK = document.getElementById('stanley-k-val');
  const sliderStanleyKs = document.getElementById('slider-stanley-ks');
  const labelStanleyKs = document.getElementById('stanley-ks-val');
  
  const sliderPidKp = document.getElementById('slider-pid-kp');
  const labelPidKp = document.getElementById('pid-kp-val');
  const sliderPidKi = document.getElementById('slider-pid-ki');
  const labelPidKi = document.getElementById('pid-ki-val');
  const sliderPidKd = document.getElementById('slider-pid-kd');
  const labelPidKd = document.getElementById('pid-kd-val');
  
  const sliderTargetSpeed = document.getElementById('slider-target-speed');
  const labelTargetSpeed = document.getElementById('target-speed-val');

  // Track A EDA Dataset components
  const btnEdaNext = document.getElementById('btn-eda-next');
  const btnEdaRun = document.getElementById('btn-eda-run');
  const edaFrameNum = document.getElementById('eda-frame-num');
  const edaActiveClass = document.getElementById('eda-active-class');
  const edaActiveConf = document.getElementById('eda-active-conf');
  const edaActiveDist = document.getElementById('eda-active-dist');

  let ros2LogTimer = null;
  const ros2Logs = [
    "[INFO] [carla_simulator]: Vehicle state ticked. Telemetry active.",
    "[INFO] [perception_node]: Bounding box clustering: 3 objects in Lidar sensor field.",
    "[INFO] [localization_node]: Sensor fusion running. GPS/IMU pose updated.",
    "[INFO] [planning_node]: Waypoints spline trajectory calculated.",
    "[INFO] [control_node]: Stanley lateral error: CTE = 0.12m, Heading error = 0.04 rad.",
    "[INFO] [control_node]: PID speed matching throttle value: 0.45"
  ];
  let logIdx = 0;

  const logROS2 = (msg, type = 'info') => {
    const line = document.createElement('div');
    line.className = `log-line ${type}`;
    line.innerText = msg;
    ros2Console.appendChild(line);
    ros2Console.scrollTop = ros2Console.scrollHeight;
  };

  const startROS2Logger = () => {
    if (ros2LogTimer) return;
    logROS2("[ROS2] Initializing rclpy nodes...", "success");
    logROS2("[INFO] [perception_node]: Subscribed to /carla/ego_vehicle/lidar", "info");
    logROS2("[INFO] [planning_node]: Subscribed to /planning/global_route", "info");
    logROS2("[INFO] [control_node]: Subscribed to /carla/ego_vehicle/odometry", "info");

    ros2LogTimer = setInterval(() => {
      let logMsg = ros2Logs[logIdx % ros2Logs.length];
      
      // Inject real values in simulation
      if (logMsg.includes("Stanley lateral error")) {
        logMsg = `[INFO] [control_node]: Steering lateral error: CTE = ${simulation.trackACTE.toFixed(2)}m, Heading error = ${(simulation.trackAHeadingError * 180 / Math.PI).toFixed(1)} deg.`;
      } else if (logMsg.includes("PID speed matching")) {
        const speedErr = simulation.trackATargetSpeed - simulation.car.speed;
        logMsg = `[INFO] [control_node]: Speed control throttle value: ${(speedErr > 0.1 ? 0.35 + Math.random()*0.3 : 0.05).toFixed(2)}`;
      }
      
      logROS2(logMsg);
      logIdx++;
    }, 1800);
  };

  const stopROS2Autopilot = () => {
    chkRos2Autonomous.checked = false;
    const hudIndicator = document.getElementById('indicator-auto').querySelector('.dot');
    hudIndicator.classList.remove('active');
    
    simulation.speedPIDSim.reset();
    simulation.steerPIDSim.reset();
    if (ros2LogTimer) {
      clearInterval(ros2LogTimer);
      ros2LogTimer = null;
    }
  };

  // Switch Track Tabs
  tabTrackB.addEventListener('click', () => {
    tabTrackB.classList.add('active');
    tabTrackA.classList.remove('active');
    
    trackBSdcBody.classList.remove('hidden');
    trackASdcBody.classList.add('hidden');
    
    picoConsole.classList.remove('hidden');
    ros2Console.classList.add('hidden');
    consoleTitle.innerText = "VIRTUAL PICO W LOG CONSOLE";
    
    trackBControlsBody.classList.remove('hidden');
    trackAControlsBody.classList.add('hidden');
    controlsTitle.innerText = "ROBOT DIRECT CONTROLLER";
    
    trackBHardwareBody.classList.remove('hidden');
    trackAHardwareBody.classList.add('hidden');
    hardwareTitle.innerText = "PICO W & BREADBOARD EMULATOR";
    hardwareBadge.innerText = "GPIO STATUS";
    hardwareBadge.className = "badge badge-green";
    
    simulation.setTrackMode('B');
    stopROS2Autopilot();
  });

  tabTrackA.addEventListener('click', () => {
    tabTrackB.classList.remove('active');
    tabTrackA.classList.add('active');
    
    trackBSdcBody.classList.add('hidden');
    trackASdcBody.classList.remove('hidden');
    
    picoConsole.classList.add('hidden');
    ros2Console.classList.remove('hidden');
    consoleTitle.innerText = "ROS2 TOPIC LOG CONSOLE";
    
    trackBControlsBody.classList.add('hidden');
    trackAControlsBody.classList.remove('hidden');
    controlsTitle.innerText = "CARLA AUTOPILOT TUNER";
    
    trackBHardwareBody.classList.add('hidden');
    trackAHardwareBody.classList.remove('hidden');
    hardwareTitle.innerText = "ROS2 MIDDLEWARE NODE GRAPH";
    hardwareBadge.innerText = "20Hz TOPICS";
    hardwareBadge.className = "badge";
    
    // Stop Track B Autopilot if running
    chkAutonomous.checked = false;
    stopAutonomousMode();
    
    simulation.setTrackMode('A');
    simulation.drawWaypoints(sdc.mapping.getWaypoints());
  });

  // Slider bindings for Track A
  chkRos2Autonomous.addEventListener('change', (e) => {
    const isChecked = e.target.checked;
    const hudIndicator = document.getElementById('indicator-auto').querySelector('.dot');
    
    if (isChecked) {
      hudIndicator.classList.add('active');
      simulation.speedPIDSim.reset();
      simulation.steerPIDSim.reset();
      startROS2Logger();
      logROS2("[ROS2] Autonomous autopilot engaged.", "success");
    } else {
      hudIndicator.classList.remove('active');
      if (ros2LogTimer) {
        clearInterval(ros2LogTimer);
        ros2LogTimer = null;
      }
      logROS2("[ROS2] Autonomous autopilot disengaged.", "error");
    }
  });

  ctrlLawSelect.addEventListener('change', (e) => {
    const law = e.target.value;
    simulation.trackAController = law;
    if (law === 'stanley') {
      stanleyGains.classList.remove('hidden');
      pidGains.classList.add('hidden');
      logROS2("[ROS2] Switched steering node to Stanley Lateral Kinematic Law.");
    } else {
      stanleyGains.classList.add('hidden');
      pidGains.classList.remove('hidden');
      logROS2("[ROS2] Switched steering node to PID Closed-Loop feedback.");
    }
  });

  // Stanley gains
  sliderStanleyK.addEventListener('input', (e) => {
    const val = e.target.value / 10.0;
    labelStanleyK.innerText = val.toFixed(1);
    simulation.trackAStanleyGains.k = val;
  });
  sliderStanleyKs.addEventListener('input', (e) => {
    const val = e.target.value / 10.0;
    labelStanleyKs.innerText = val.toFixed(1);
    simulation.trackAStanleyGains.ks = val;
  });

  // PID steering gains
  sliderPidKp.addEventListener('input', (e) => {
    const val = e.target.value / 10.0;
    labelPidKp.innerText = val.toFixed(1);
    simulation.trackAPIDGains.kp = val;
  });
  sliderPidKi.addEventListener('input', (e) => {
    const val = e.target.value / 10.0;
    labelPidKi.innerText = val.toFixed(1);
    simulation.trackAPIDGains.ki = val;
  });
  sliderPidKd.addEventListener('input', (e) => {
    const val = e.target.value / 10.0;
    labelPidKd.innerText = val.toFixed(1);
    simulation.trackAPIDGains.kd = val;
  });

  // Speed
  sliderTargetSpeed.addEventListener('input', (e) => {
    const val = e.target.value / 10.0;
    labelTargetSpeed.innerText = `${val.toFixed(1)} m/s`;
    simulation.trackATargetSpeed = val;
  });

  // --- ROS2 NODE GRAPH CANVAS ANIMATION ---
  const graphCanvas = document.getElementById('ros2-graph-canvas');
  const graphCtx = graphCanvas.getContext('2d');

  const nodes = [
    { name: '/carla_sim', x: 50, y: 130, r: 21, label: 'CARLA Sim' },
    { name: '/perception', x: 50, y: 50, r: 21, label: 'Perception' },
    { name: '/localization', x: 160, y: 50, r: 21, label: 'Localize' },
    { name: '/planning', x: 265, y: 90, r: 21, label: 'Planning' },
    { name: '/control', x: 160, y: 130, r: 21, label: 'Control' }
  ];

  const connections = [
    { from: 0, to: 1, label: '/lidar_scan', pos: 0.0, speed: 0.015 },
    { from: 0, to: 2, label: '/imu_odom', pos: 0.3, speed: 0.02 },
    { from: 1, to: 3, label: '/obstacles', pos: 0.5, speed: 0.012 },
    { from: 2, to: 3, label: '/pose_state', pos: 0.1, speed: 0.018 },
    { from: 3, to: 4, label: '/local_path', pos: 0.7, speed: 0.01 },
    { from: 4, to: 0, label: '/vehicle_cmd', pos: 0.0, speed: 0.025 }
  ];

  const animateGraph = () => {
    if (!simulation.trackAActive) {
      requestAnimationFrame(animateGraph);
      return;
    }

    const w = graphCanvas.width;
    const h = graphCanvas.height;
    graphCtx.clearRect(0, 0, w, h);

    const isEngaged = chkRos2Autonomous.checked;

    // Draw connection channels
    connections.forEach(conn => {
      const from = nodes[conn.from];
      const to = nodes[conn.to];

      graphCtx.strokeStyle = isEngaged ? 'rgba(0, 242, 254, 0.25)' : 'rgba(255, 255, 255, 0.05)';
      graphCtx.lineWidth = 1.5;
      graphCtx.beginPath();
      graphCtx.moveTo(from.x, from.y);
      graphCtx.lineTo(to.x, to.y);
      graphCtx.stroke();

      const midX = (from.x + to.x) / 2;
      const midY = (from.y + to.y) / 2;
      graphCtx.fillStyle = isEngaged ? '#64748b' : '#334155';
      graphCtx.font = '6.5px Share Tech Mono';
      graphCtx.textAlign = 'center';
      graphCtx.fillText(conn.label, midX, midY - 3);

      if (isEngaged) {
        conn.pos += conn.speed;
        if (conn.pos > 1.0) conn.pos = 0;

        const px = from.x + (to.x - from.x) * conn.pos;
        const py = from.y + (to.y - from.y) * conn.pos;

        graphCtx.fillStyle = '#00f2fe';
        graphCtx.beginPath();
        graphCtx.arc(px, py, 3.0, 0, Math.PI * 2);
        graphCtx.fill();
      }
    });

    // Draw node bubbles
    nodes.forEach(node => {
      graphCtx.strokeStyle = isEngaged ? '#00e676' : '#64748b'; // active green, idle gray
      graphCtx.lineWidth = 1.5;
      graphCtx.fillStyle = '#0a0b16';
      graphCtx.beginPath();
      graphCtx.arc(node.x, node.y, node.r, 0, Math.PI * 2);
      graphCtx.fill();
      graphCtx.stroke();

      graphCtx.fillStyle = '#e2e8f0';
      graphCtx.font = 'bold 7px Space Grotesk';
      graphCtx.textAlign = 'center';
      graphCtx.textBaseline = 'middle';
      graphCtx.fillText(node.label, node.x, node.y);
    });

    requestAnimationFrame(animateGraph);
  };

  // --- NUSCENES EDA INTERACTIVE COMPONENT ---
  const edaCanvas = document.getElementById('eda-radar-canvas');
  const edaCtx = edaCanvas.getContext('2d');

  let edaFrameIdx = 0;
  let edaCleaned = false;

  const edaFrames = [
    {
      frameNum: 'FRAME 042',
      objects: [
        { class: 'Car', x: 20, y: 15, conf: 0.94, dist: 25.0, w: 10, h: 6 },
        { class: 'Car', x: -15, y: -25, conf: 0.88, dist: 29.1, w: 9, h: 5 },
        { class: 'Pedestrian', x: 10, y: -8, conf: 0.72, dist: 12.8, w: 4, h: 4 }
      ]
    },
    {
      frameNum: 'FRAME 043',
      objects: [
        { class: 'Truck', x: 28, y: -6, conf: 0.91, dist: 28.6, w: 14, h: 7 },
        { class: 'Car', x: -10, y: 18, conf: 0.96, dist: 20.6, w: 10, h: 6 }
      ]
    },
    {
      frameNum: 'FRAME 044',
      objects: [
        { class: 'Car', x: 5, y: 32, conf: 0.92, dist: 32.4, w: 10, h: 6 },
        { class: 'Bicycle', x: -12, y: -10, conf: 0.65, dist: 15.6, w: 5, h: 4 },
        { class: 'Pedestrian', x: 22, y: 20, conf: 0.78, dist: 29.7, w: 4, h: 4 }
      ]
    }
  ];

  const drawEDAScan = () => {
    const w = edaCanvas.width;
    const h = edaCanvas.height;
    edaCtx.clearRect(0, 0, w, h);

    const center = w / 2;

    // Draw range rings
    edaCtx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
    edaCtx.lineWidth = 0.8;
    for (let r = 18; r <= 48; r += 15) {
      edaCtx.beginPath();
      edaCtx.arc(center, center, r, 0, Math.PI * 2);
      edaCtx.stroke();
    }

    // Ego dot
    edaCtx.fillStyle = '#ff4757';
    edaCtx.beginPath();
    edaCtx.arc(center, center, 3, 0, Math.PI * 2);
    edaCtx.fill();

    const frame = edaFrames[edaFrameIdx];
    edaFrameNum.innerText = frame.frameNum;

    // Seeded random points generator
    const pts = [];
    const rand = (seed) => {
      const x = Math.sin(seed) * 10000;
      return x - Math.floor(x);
    };

    frame.objects.forEach((obj, objIdx) => {
      const scale = 1.3;
      const ox = center + obj.x * scale;
      const oy = center - obj.y * scale;

      // Draw clusters
      for (let p = 0; p < 15; p++) {
        const seed = objIdx * 40 + p * 12;
        const px = ox + (rand(seed) - 0.5) * 10;
        const py = oy + (rand(seed + 1) - 0.5) * 10;
        pts.push({ x: px, y: py, isNoise: false });
      }
    });

    // Generate static noise points
    for (let n = 0; n < 30; n++) {
      const seed = 300 + n * 8 + edaFrameIdx * 150;
      const px = rand(seed) * w;
      const py = rand(seed + 2) * h;
      pts.push({ x: px, y: py, isNoise: true });
    }

    // Draw points
    pts.forEach(pt => {
      if (edaCleaned) {
        if (pt.isNoise) {
          edaCtx.fillStyle = 'rgba(255, 255, 255, 0.04)';
        } else {
          edaCtx.fillStyle = '#00e676'; // bright green cleaned
        }
      } else {
        edaCtx.fillStyle = 'rgba(0, 242, 254, 0.4)'; // raw cyan
      }
      edaCtx.beginPath();
      edaCtx.arc(pt.x, pt.y, 1.2, 0, Math.PI * 2);
      edaCtx.fill();
    });

    // Draw bounds if cleaned
    if (edaCleaned) {
      frame.objects.forEach(obj => {
        const scale = 1.3;
        const ox = center + obj.x * scale;
        const oy = center - obj.y * scale;

        edaCtx.strokeStyle = '#ff9f43';
        edaCtx.lineWidth = 1.0;
        edaCtx.strokeRect(ox - obj.w / 2, oy - obj.h / 2, obj.w, obj.h);

        edaCtx.fillStyle = '#ff9f43';
        edaCtx.font = '6px Space Grotesk';
        edaCtx.fillText(obj.class, ox - obj.w / 2, oy - obj.h / 2 - 2);
      });

      const primary = frame.objects[0];
      edaActiveClass.innerText = primary.class.toUpperCase();
      edaActiveConf.innerText = primary.conf.toFixed(2);
      edaActiveDist.innerText = `${primary.dist.toFixed(1)}m`;
      edaActiveClass.className = "font-mono text-warning";
    } else {
      edaActiveClass.innerText = "RAW DATA";
      edaActiveConf.innerText = "UNFILTERED";
      edaActiveDist.innerText = "NOISY";
      edaActiveClass.className = "font-mono text-muted";
    }
  };

  btnEdaNext.addEventListener('click', () => {
    edaFrameIdx = (edaFrameIdx + 1) % edaFrames.length;
    edaCleaned = false;
    drawEDAScan();
    logROS2(`[INFO] [perception_node]: advanced nuScenes dataset stream to FRAME 04${edaFrameIdx + 2}.`);
  });

  btnEdaRun.addEventListener('click', () => {
    edaCleaned = true;
    drawEDAScan();
    logROS2("[INFO] [perception_node]: executing Pandas dataset cleaning algorithm...");
    logROS2("[INFO] [perception_node]: filtered outliers and Lidar background noise. Bounding boxes locked.");
  });

  // Initial draw and trigger graph animation
  drawEDAScan();
  animateGraph();

});
