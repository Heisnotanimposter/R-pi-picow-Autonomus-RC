/**
 * Real-time 3D Simulation Engine
 * Handles rendering, maps, physics, collisions, and sensor simulations.
 */

// Internal helper for simulation PID
class JS_PIDController {
  constructor(kp, ki, kd, minLim, maxLim) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.minLim = minLim;
    this.maxLim = maxLim;
    this.integral = 0;
    this.prevError = 0;
    this.integralLimit = 5.0;
  }
  update(error, dt) {
    this.integral += error * dt;
    this.integral = Math.max(-this.integralLimit, Math.min(this.integral, this.integralLimit));
    const derivative = (error - this.prevError) / dt;
    this.prevError = error;
    let output = this.kp * error + this.ki * this.integral + this.kd * derivative;
    return Math.max(this.minLim, Math.min(this.maxLim, output));
  }
  reset() {
    this.integral = 0;
    this.prevError = 0;
  }
}

class SimulationEngine {
  constructor(mainContainerId, fpvContainerId, picoVM, onTelemetryUpdate) {
    this.mainContainer = document.getElementById(mainContainerId);
    this.fpvContainer = document.getElementById(fpvContainerId);
    this.picoVM = picoVM;
    this.onTelemetryUpdate = onTelemetryUpdate || (() => {});

    // Simulation states
    this.currentMapName = 'knut-chungju';
    this.timeOfDay = 'day'; // day, sunset, night
    this.showLaser = true;
    this.showHeadlights = true;
    this.frictionCoeff = 0.5;
    
    // Physics states
    this.car = {
      position: new THREE.Vector3(0, 0.5, 80), // Start near E1 Main Gate
      velocity: new THREE.Vector3(0, 0, 0),
      speed: 0,
      maxSpeed: 15.0,
      acceleration: 22.0,
      deceleration: 8.0,
      friction: 4.0,
      angle: Math.PI, // Face north initially
      rotationSpeed: 2.2, // Radians per sec
      width: 2.4,
      length: 3.5,
      height: 1.2
    };

    this.collidables = [];
    this.buildings = [];
    this.wheels = [];
    this.sensorDistance = 400.0; // Max HC-SR04 range is 400cm
    this.cameraMode = 'orbit'; // orbit, fpv, chase, top

    // Track A states
    this.trackAActive = false;
    this.trackAController = 'stanley'; // 'stanley' or 'pid'
    this.trackATargetSpeed = 8.0;
    this.trackAStanleyGains = { k: 1.2, ks: 0.5 };
    this.trackAPIDGains = { kp: 1.5, ki: 0.1, kd: 0.2 };
    
    // Instantiated internal JS controllers for simulation
    this.speedPIDSim = new JS_PIDController(1.0, 0.1, 0.05, -1.0, 1.0);
    this.steerPIDSim = new JS_PIDController(1.5, 0.1, 0.2, -0.52, 0.52);
    
    this.trackASteeringAngle = 0.0;
    this.trackASpeedError = 0.0;
    this.trackACTE = 0.0;
    this.trackAHeadingError = 0.0;
    this.currentWaypoints = [];
    this.lidarAngle = 0.0;

    // Track C RL states
    this.trackCActive = false;
    this.trackCSpeedup = 1;
    this.rlTrainingActive = false;
    this.hasCollidedThisTick = false;

    // Setup Scenes
    this.initThree();
    this.loadMap(this.currentMapName);
    this.createRobotCar();
    this.createSedanCar();
    this.createGhostRobot();
    this.setupLighting();

    // Start loop
    this.clock = new THREE.Clock();
    this.animate();

    // Resize handler
    window.addEventListener('resize', () => this.onWindowResize());
  }

  // Initialize Three.js renderers, scene, cameras
  initThree() {
    // 1. Main Viewport
    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.FogExp2(0x0a0c16, 0.002);

    this.camera = new THREE.PerspectiveCamera(60, this.mainContainer.clientWidth / this.mainContainer.clientHeight, 0.1, 1000);
    this.camera.position.set(0, 40, 120);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(this.mainContainer.clientWidth, this.mainContainer.clientHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.setClearColor(0x0a0c16, 1);
    this.mainContainer.appendChild(this.renderer.domElement);

    // Orbit Controls
    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.05;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.05; // Don't go below ground
    this.controls.minDistance = 5;
    this.controls.maxDistance = 300;

    // 2. FPV Camera and Viewport
    this.fpvScene = new THREE.Scene();
    this.fpvCamera = new THREE.PerspectiveCamera(75, 4 / 3, 0.1, 150); // 320x240 aspect

    this.fpvRenderer = new THREE.WebGLRenderer({ antialias: true });
    this.fpvRenderer.setSize(this.fpvContainer.clientWidth, this.fpvContainer.clientHeight);
    this.fpvRenderer.shadowMap.enabled = false;
    this.fpvContainer.appendChild(this.fpvRenderer.domElement);
  }

  // Handle window resizing
  onWindowResize() {
    this.camera.aspect = this.mainContainer.clientWidth / this.mainContainer.clientHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(this.mainContainer.clientWidth, this.mainContainer.clientHeight);

    this.fpvRenderer.setSize(this.fpvContainer.clientWidth, this.fpvContainer.clientHeight);
  }

  // Generate / Load Map
  loadMap(mapName) {
    this.currentMapName = mapName;
    
    // Clear old map meshes
    if (this.mapGroup) this.scene.remove(this.mapGroup);
    if (this.waypointGroup) this.scene.remove(this.waypointGroup);
    
    this.collidables = [];
    this.buildings = [];

    this.mapGroup = new THREE.Group();
    this.scene.add(this.mapGroup);
    
    this.waypointGroup = new THREE.Group();
    this.scene.add(this.waypointGroup);

    if (mapName === 'knut-chungju') {
      this.generateKNUTChungjuMap();
    } else if (mapName === 'knut-uiwang') {
      this.generateKNUTUiwangMap();
    } else {
      this.generateObstacleCourse();
    }

    // Adapt car start position
    if (mapName === 'obstacle-course') {
      this.car.position.set(0, 0.5, 40);
      this.car.angle = Math.PI;
    } else {
      this.car.position.set(0, 0.5, 80);
      this.car.angle = Math.PI;
    }
    this.car.velocity.set(0,0,0);
    this.car.speed = 0;

    // Reset Track A variables
    if (this.speedPIDSim) this.speedPIDSim.reset();
    if (this.steerPIDSim) this.steerPIDSim.reset();
    this.trackASteeringAngle = 0.0;
    this.trackASpeedError = 0.0;
    this.trackACTE = 0.0;
    this.trackAHeadingError = 0.0;
  }

  // Custom Elevation calculator representing Chungju Campus slopes
  getTerrainHeight(x, z) {
    if (this.currentMapName === 'obstacle-course') return 0; // Flat
    
    // KNUT Chungju main campus rises as we go north (z decreases) and east (x increases)
    // Add a gentle base slope + some soft hills
    const baseSlope = (80 - z) * 0.05 + (x * 0.02);
    const hill1 = Math.sin(x * 0.05) * Math.cos(z * 0.05) * 1.5;
    const hill2 = Math.sin(x * 0.01) * 3;
    
    return Math.max(-2, baseSlope + hill1 + hill2);
  }

  // Create Terrain mesh
  createTerrainMesh(width, depth, colorHex) {
    const segments = 60;
    const geometry = new THREE.PlaneGeometry(width, depth, segments, segments);
    geometry.rotateX(-Math.PI / 2);

    const positions = geometry.attributes.position;
    for (let i = 0; i < positions.count; i++) {
      const x = positions.getX(i);
      const z = positions.getZ(i);
      positions.setY(i, this.getTerrainHeight(x, z));
    }
    geometry.computeVertexNormals();

    const material = new THREE.MeshStandardMaterial({
      color: colorHex,
      roughness: 0.8,
      metalness: 0.1,
      flatShading: true
    });

    const mesh = new THREE.Mesh(geometry, material);
    mesh.receiveShadow = true;
    this.mapGroup.add(mesh);
  }

  // Add building to map
  addBuilding(name, x, z, width, depth, height, colorHex, heading = 0) {
    const y = this.getTerrainHeight(x, z) + height / 2;
    const geometry = new THREE.BoxGeometry(width, height, depth);
    
    const material = new THREE.MeshStandardMaterial({
      color: colorHex,
      roughness: 0.4,
      metalness: 0.3,
      transparent: true,
      opacity: 0.85
    });
    
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(x, y, z);
    mesh.rotation.y = heading;
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    
    this.mapGroup.add(mesh);
    this.buildings.push({ name, mesh, radius: Math.max(width, depth) / 1.7, height });
    this.collidables.push(mesh);

    // Build neon outline/glow
    const edges = new THREE.EdgesGeometry(geometry);
    const lineMat = new THREE.LineBasicMaterial({ color: 0x00f2fe, linewidth: 2 });
    const wireframe = new THREE.LineSegments(edges, lineMat);
    wireframe.position.copy(mesh.position);
    wireframe.rotation.y = heading;
    this.mapGroup.add(wireframe);

    // Dynamic 3D text labels above buildings
    this.createLabelSprite(name, x, y + height / 2 + 3);
  }

  createLabelSprite(text, x, y, z) {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');
    
    ctx.fillStyle = 'rgba(0,0,0,0.6)';
    ctx.fillRect(0, 0, 256, 64);
    ctx.strokeStyle = '#00f2fe';
    ctx.lineWidth = 2;
    ctx.strokeRect(0, 0, 256, 64);
    
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 20px Space Grotesk';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 128, 32);

    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ map: texture, transparent: true });
    const sprite = new THREE.Sprite(material);
    sprite.position.set(x, y, z || x); // z handles correct displacement
    sprite.position.z = z !== undefined ? z : y; // fix mapping logic
    sprite.position.y = y;
    sprite.position.x = x;
    sprite.scale.set(12, 3, 1);
    
    this.mapGroup.add(sprite);
  }

  // Generate Korea National University of Transportation (KNUT) Chungju Campus Map
  generateKNUTChungjuMap() {
    // 1. Terrain Grass
    this.createTerrainMesh(250, 250, 0x091410);

    // 2. Roads (Main center road & East/West arterials)
    // Draw using slightly elevated line/planes following terrain
    const roadWidth = 8.0;
    const roadPoints = [
      new THREE.Vector3(0, 0, 100),   // E1 Main Gate
      new THREE.Vector3(0, 0, 20),    // Center crossroad
      new THREE.Vector3(-40, 0, 10),  // West fork
      new THREE.Vector3(40, 0, 10),   // East fork
      new THREE.Vector3(0, 0, -80)    // North library area
    ];

    roadPoints.forEach(pt => {
      pt.y = this.getTerrainHeight(pt.x, pt.z) + 0.05;
    });

    // Create styled road paths using custom geometries (represented simply here)
    const mainRoadGeo = new THREE.BoxGeometry(roadWidth, 0.1, 180);
    const mainRoadMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.9 });
    const mainRoad = new THREE.Mesh(mainRoadGeo, mainRoadMat);
    mainRoad.position.set(0, this.getTerrainHeight(0, 10) + 0.05, 10);
    this.mapGroup.add(mainRoad);

    const crossRoadGeo = new THREE.BoxGeometry(160, 0.1, roadWidth);
    const crossRoad = new THREE.Mesh(crossRoadGeo, mainRoadMat);
    crossRoad.position.set(0, this.getTerrainHeight(0, 15) + 0.05, 15);
    this.mapGroup.add(crossRoad);

    // 3. Buildings matching real campus layout
    // E (East) Zone - Right Side
    this.addBuilding('E1 Main Gate', 0, 90, 8, 2, 4, 0x475569);
    this.addBuilding('E3 Dormitories', 45, 55, 15, 12, 12, 0x1e293b);
    this.addBuilding('E6 Student Center', 40, 20, 20, 14, 15, 0x3b82f6);
    this.addBuilding('E8 Administration', 40, -20, 24, 14, 20, 0x0f172a);
    this.addBuilding('E17 Smart ICT Hall', 45, -60, 18, 12, 16, 0x10b981);

    // W (West) Zone - Left Side
    this.addBuilding('W19 Stadium Field', -50, 50, 30, 20, 0.2, 0x15803d);
    this.addBuilding('W10 Dormitories', -45, 15, 16, 12, 12, 0x1e293b);
    this.addBuilding('W16 IT Building', -40, -20, 18, 14, 16, 0x10b981);
    this.addBuilding('W20 Central Library', -45, -60, 25, 20, 22, 0x6366f1);

    // 4. Foliage & Obstacles (Trees)
    for (let i = 0; i < 45; i++) {
      const tx = (Math.random() - 0.5) * 200;
      const tz = (Math.random() - 0.5) * 200;
      
      // Keep trees off roads
      if (Math.abs(tx) > 8 && Math.abs(tz - 15) > 8) {
        this.addTree(tx, tz);
      }
    }
  }

  // Generate Korea National University of Transportation (KNUT) Uiwang Campus Map (Railroad theme)
  generateKNUTUiwangMap() {
    this.createTerrainMesh(250, 250, 0x0f121b);

    // Model a central railway track since it's the railroad campus
    const trackGeo = new THREE.BoxGeometry(4, 0.3, 200);
    const trackMat = new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.8 });
    const track = new THREE.Mesh(trackGeo, trackMat);
    track.position.set(10, this.getTerrainHeight(10, 0) + 0.15, 0);
    this.mapGroup.add(track);

    // Railroad ties (wooden sleep structures)
    for (let z = -90; z <= 90; z += 6) {
      const tieGeo = new THREE.BoxGeometry(7, 0.1, 1.2);
      const tieMat = new THREE.MeshStandardMaterial({ color: 0x271707 });
      const tie = new THREE.Mesh(tieGeo, tieMat);
      tie.position.set(10, this.getTerrainHeight(10, z) + 0.05, z);
      this.mapGroup.add(tie);
    }

    // Uiwang Buildings
    this.addBuilding('W18 Future Science Hall', -40, 40, 18, 15, 18, 0x334155);
    this.addBuilding('College of Railroads', -35, -10, 25, 15, 22, 0x1e3a8a);
    this.addBuilding('Railroad Museum', 40, -40, 20, 20, 12, 0xb91c1c);
    this.addBuilding('Railway Training Yard', 45, 30, 20, 15, 6, 0x3f3f46);

    // Trees
    for (let i = 0; i < 30; i++) {
      const tx = (Math.random() - 0.5) * 200;
      const tz = (Math.random() - 0.5) * 200;
      if (Math.abs(tx - 10) > 10) {
        this.addTree(tx, tz);
      }
    }
  }

  // Generate Obstacle Course Map
  generateObstacleCourse() {
    // 1. Dark Grid Floor
    const gridHelper = new THREE.GridHelper(200, 50, 0x00f2fe, 0x1e293b);
    gridHelper.position.y = 0.01;
    this.mapGroup.add(gridHelper);

    this.createTerrainMesh(200, 200, 0x050508);

    // 2. Surrounding Walls
    this.addWall(0, -100, 200, 1, 8);
    this.addWall(0, 100, 200, 1, 8);
    this.addWall(-100, 0, 1, 200, 8);
    this.addWall(100, 0, 1, 200, 8);

    // 3. Inner Obstacles: Pillars and Corridors
    // Center column maze
    this.addBuilding('Pillar A', -30, 30, 8, 8, 15, 0x1e293b);
    this.addBuilding('Pillar B', 30, 30, 8, 8, 15, 0x1e293b);
    this.addBuilding('Pillar C', -30, -30, 8, 8, 15, 0x1e293b);
    this.addBuilding('Pillar D', 30, -30, 8, 8, 15, 0x1e293b);

    // Crosswalls creating corridors
    this.addBuilding('Wall Center-Left', -50, 0, 40, 4, 8, 0x334155);
    this.addBuilding('Wall Center-Right', 50, 0, 40, 4, 8, 0x334155);
    this.addBuilding('Wall North-Choke', 0, -50, 4, 30, 8, 0x334155);
    this.addBuilding('Wall South-Choke', 0, 50, 4, 30, 8, 0x334155);

    // Benches, barrels, barriers
    for (let i = 0; i < 15; i++) {
      const bx = (Math.random() - 0.5) * 140;
      const bz = (Math.random() - 0.5) * 140;
      
      // Avoid spawn zone
      if (Math.abs(bx) > 10 || Math.abs(bz - 40) > 10) {
        this.addObstacleBlock(bx, bz);
      }
    }
  }

  // Helper for adding tree meshes
  addTree(x, z) {
    const y = this.getTerrainHeight(x, z);
    const treeGroup = new THREE.Group();
    
    // Trunk
    const trunkGeo = new THREE.CylinderGeometry(0.4, 0.6, 5, 8);
    const trunkMat = new THREE.MeshStandardMaterial({ color: 0x451a03, roughness: 0.9 });
    const trunk = new THREE.Mesh(trunkGeo, trunkMat);
    trunk.position.y = 2.5;
    trunk.castShadow = true;
    treeGroup.add(trunk);

    // Leaves
    const leavesGeo = new THREE.ConeGeometry(3, 8, 8);
    const leavesMat = new THREE.MeshStandardMaterial({ color: 0x065f46, roughness: 0.6 });
    const leaves = new THREE.Mesh(leavesGeo, leavesMat);
    leaves.position.y = 7;
    leaves.castShadow = true;
    treeGroup.add(leaves);

    treeGroup.position.set(x, y, z);
    this.mapGroup.add(treeGroup);
    
    // Add collision boundary
    this.buildings.push({ 
      name: 'Tree', 
      mesh: trunk, 
      radius: 1.0, 
      height: 10 
    });
    this.collidables.push(trunk);
  }

  // Helper for adding wall barriers
  addWall(x, z, w, d, h) {
    const geo = new THREE.BoxGeometry(w, h, d);
    const mat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.8 });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(x, h/2, z);
    mesh.receiveShadow = true;
    this.mapGroup.add(mesh);
    this.collidables.push(mesh);
    this.buildings.push({ name: 'Border Wall', mesh, radius: Math.max(w, d) / 2, height: h });
  }

  // Small obstacle block
  addObstacleBlock(x, z) {
    const h = 2 + Math.random() * 2;
    const w = 2.5;
    const d = 2.5;
    const geo = new THREE.BoxGeometry(w, h, d);
    const mat = new THREE.MeshStandardMaterial({ color: 0xef4444, roughness: 0.5 });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(x, h/2, z);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    this.mapGroup.add(mesh);
    this.collidables.push(mesh);
    this.buildings.push({ name: 'Obstacle Block', mesh, radius: 1.8, height: h });
    
    // Wire glow
    const edges = new THREE.EdgesGeometry(geo);
    const lineMat = new THREE.LineBasicMaterial({ color: 0xff3366 });
    const wireframe = new THREE.LineSegments(edges, lineMat);
    wireframe.position.copy(mesh.position);
    this.mapGroup.add(wireframe);
  }

  // Create Robot Car Mesh matching original Zumo/4WD styling
  createRobotCar() {
    this.robotGroup = new THREE.Group();
    this.robotGroup.position.copy(this.car.position);
    this.scene.add(this.robotGroup);

    // Chassis Box
    const chassisGeo = new THREE.BoxGeometry(this.car.width, this.car.height - 0.4, this.car.length);
    const chassisMat = new THREE.MeshStandardMaterial({ color: 0x111827, roughness: 0.5, metalness: 0.6 });
    const chassis = new THREE.Mesh(chassisGeo, chassisMat);
    chassis.position.y = 0.5;
    chassis.castShadow = true;
    this.robotGroup.add(chassis);

    // Cover Plate (acrylic yellow, matching the PDF screenshot!)
    const coverGeo = new THREE.BoxGeometry(this.car.width + 0.1, 0.1, this.car.length + 0.1);
    const coverMat = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.2, metalness: 0.1 });
    const cover = new THREE.Mesh(coverGeo, coverMat);
    cover.position.y = 0.9;
    cover.castShadow = true;
    this.robotGroup.add(cover);

    // Wheels (4WD)
    const wheelGeo = new THREE.CylinderGeometry(0.6, 0.6, 0.5, 12);
    wheelGeo.rotateZ(Math.PI / 2);
    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x000000, roughness: 0.9 });
    
    // Positions relative to chassis center
    const wheelOffsets = [
      { x: -1.3, z: 1.1 },  // Front Left
      { x: 1.3, z: 1.1 },   // Front Right
      { x: -1.3, z: -1.1 }, // Rear Left
      { x: 1.3, z: -1.1 }   // Rear Right
    ];

    this.wheels = [];
    wheelOffsets.forEach((offset, idx) => {
      const wheel = new THREE.Mesh(wheelGeo, wheelMat);
      wheel.position.set(offset.x, 0.4, offset.z);
      wheel.castShadow = true;
      this.robotGroup.add(wheel);
      this.wheels.push(wheel);
    });

    // Simulated Headlights
    this.headlights = [];
    const lightColor = 0xffffff;
    
    const headlightOffsets = [-0.8, 0.8];
    headlightOffsets.forEach(offsetX => {
      // 3D Visual Mesh bulb
      const bulbGeo = new THREE.SphereGeometry(0.15, 8, 8);
      const bulbMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
      const bulb = new THREE.Mesh(bulbGeo, bulbMat);
      bulb.position.set(offsetX, 0.5, 1.7);
      this.robotGroup.add(bulb);

      // SpotLight
      const spotlight = new THREE.SpotLight(lightColor, 8, 40, Math.PI / 6, 0.5, 1);
      spotlight.position.set(offsetX, 0.5, 1.8);
      spotlight.castShadow = true;
      spotlight.shadow.mapSize.width = 512;
      spotlight.shadow.mapSize.height = 512;
      
      // Target in front of car
      const targetObj = new THREE.Object3D();
      targetObj.position.set(offsetX, 0.5, 10);
      this.robotGroup.add(targetObj);
      spotlight.target = targetObj;

      this.robotGroup.add(spotlight);
      this.headlights.push(spotlight);
    });

    // Simulated HC-SR04 Ultrasonic Sensor on front bumper
    const sensorGroup = new THREE.Group();
    sensorGroup.position.set(0, 0.6, 1.8); // Front Center
    this.robotGroup.add(sensorGroup);

    // Sensor Board backplate
    const pcbGeo = new THREE.BoxGeometry(0.8, 0.4, 0.1);
    const pcbMat = new THREE.MeshStandardMaterial({ color: 0x1e3a8a }); // Blue PCB
    const pcb = new THREE.Mesh(pcbGeo, pcbMat);
    sensorGroup.add(pcb);

    // Two ultrasonic transducer "eyes"
    const eyeGeo = new THREE.CylinderGeometry(0.15, 0.15, 0.25, 8);
    eyeGeo.rotateX(Math.PI / 2);
    const eyeMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.8 });
    
    const eyeLeft = new THREE.Mesh(eyeGeo, eyeMat);
    eyeLeft.position.set(-0.22, 0, 0.1);
    const eyeRight = new THREE.Mesh(eyeGeo, eyeMat);
    eyeRight.position.set(0.22, 0, 0.1);
    
    sensorGroup.add(eyeLeft);
    sensorGroup.add(eyeRight);

    // Sensor Ray projection indicator line
    const points = [new THREE.Vector3(0, 0.6, 1.8), new THREE.Vector3(0, 0.6, 20)];
    const lineGeo = new THREE.BufferGeometry().setFromPoints(points);
    this.laserMaterial = new THREE.LineBasicMaterial({ color: 0xff3366, transparent: true, opacity: 0.8 });
    this.laserLine = new THREE.Line(lineGeo, this.laserMaterial);
    this.scene.add(this.laserLine);

    // Laser target circle
    const targetRingGeo = new THREE.RingGeometry(0.4, 0.5, 16);
    targetRingGeo.rotateX(-Math.PI / 2);
    const targetRingMat = new THREE.MeshBasicMaterial({ color: 0xff3366, side: THREE.DoubleSide });
    this.laserTargetRing = new THREE.Mesh(targetRingGeo, targetRingMat);
    this.scene.add(this.laserTargetRing);
  }

  // Create full-sized autonomous sedan for Track A
  createSedanCar() {
    this.sedanGroup = new THREE.Group();
    this.sedanGroup.position.copy(this.car.position);
    this.scene.add(this.sedanGroup);
    
    // Lower body (chassis)
    const bodyGeo = new THREE.BoxGeometry(2.4, 0.7, 4.4);
    const bodyMat = new THREE.MeshStandardMaterial({ 
      color: 0x00d2ff, // Glossy cyan/blue electric
      roughness: 0.15, 
      metalness: 0.85 
    });
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    body.position.y = 0.5;
    body.castShadow = true;
    body.receiveShadow = true;
    this.sedanGroup.add(body);
    
    // Upper body (cabin)
    const cabinGeo = new THREE.BoxGeometry(2.0, 0.7, 2.4);
    const cabinMat = new THREE.MeshStandardMaterial({ 
      color: 0x0b0d1e, // Dark windows
      roughness: 0.05, 
      metalness: 0.9,
      transparent: true,
      opacity: 0.9
    });
    const cabin = new THREE.Mesh(cabinGeo, cabinMat);
    cabin.position.set(0, 1.1, -0.3); // slightly back
    cabin.castShadow = true;
    this.sedanGroup.add(cabin);
    
    // Front windshield / rear window slope details
    // Headlights (3D Mesh)
    const bulbGeo = new THREE.SphereGeometry(0.12, 8, 8);
    const bulbMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const headlightL = new THREE.Mesh(bulbGeo, bulbMat);
    headlightL.position.set(-0.9, 0.45, 2.2);
    const headlightR = new THREE.Mesh(bulbGeo, bulbMat);
    headlightR.position.set(0.9, 0.45, 2.2);
    this.sedanGroup.add(headlightL);
    this.sedanGroup.add(headlightR);
    
    // Taillights
    const tailMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    const taillightL = new THREE.Mesh(bulbGeo, tailMat);
    taillightL.position.set(-0.9, 0.5, -2.2);
    const taillightR = new THREE.Mesh(bulbGeo, tailMat);
    taillightR.position.set(0.9, 0.5, -2.2);
    this.sedanGroup.add(taillightL);
    this.sedanGroup.add(taillightR);
    
    // Wheels (4)
    const wheelGeo = new THREE.CylinderGeometry(0.6, 0.6, 0.4, 16);
    wheelGeo.rotateZ(Math.PI / 2);
    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x111115, roughness: 0.8 });
    const wheelOffsets = [
      { x: -1.25, z: 1.3 }, // FL
      { x: 1.25, z: 1.3 },  // FR
      { x: -1.25, z: -1.3 },// RL
      { x: 1.25, z: -1.3 }  // RR
    ];
    this.sedanWheels = [];
    wheelOffsets.forEach(offset => {
      const wheel = new THREE.Mesh(wheelGeo, wheelMat);
      wheel.position.set(offset.x, 0.4, offset.z);
      wheel.castShadow = true;
      this.sedanGroup.add(wheel);
      this.sedanWheels.push(wheel);
    });
    
    // Lidar Turret
    const turretGeo = new THREE.CylinderGeometry(0.3, 0.3, 0.25, 12);
    const turretMat = new THREE.MeshStandardMaterial({ color: 0x222533, metalness: 0.8 });
    const turret = new THREE.Mesh(turretGeo, turretMat);
    turret.position.set(0, 1.55, -0.3); // Top of cabin
    this.sedanGroup.add(turret);
    
    // Lidar spinning indicator disk
    const diskGeo = new THREE.CylinderGeometry(0.28, 0.28, 0.05, 12);
    const diskMat = new THREE.MeshBasicMaterial({ color: 0x00f2fe });
    this.lidarDisk = new THREE.Mesh(diskGeo, diskMat);
    this.lidarDisk.position.set(0, 1.7, -0.3);
    this.sedanGroup.add(this.lidarDisk);

    // Dynamic headlights spotlight (night driving)
    this.sedanHeadlights = [];
    const lightColor = 0xffffff;
    const headlightOffsets = [-0.9, 0.9];
    headlightOffsets.forEach(offsetX => {
      const spotlight = new THREE.SpotLight(lightColor, 12, 50, Math.PI / 5, 0.5, 1);
      spotlight.position.set(offsetX, 0.5, 2.2);
      spotlight.castShadow = true;
      spotlight.shadow.mapSize.width = 512;
      spotlight.shadow.mapSize.height = 512;
      
      const targetObj = new THREE.Object3D();
      targetObj.position.set(offsetX, 0.5, 12);
      this.sedanGroup.add(targetObj);
      spotlight.target = targetObj;
      
      this.sedanGroup.add(spotlight);
      this.sedanHeadlights.push(spotlight);
    });

    // Hide by default
    this.sedanGroup.visible = false;
  }

  // Create glowing cyan wireframe ghost robot representing EKF estimated pose
  createGhostRobot() {
    this.ghostGroup = new THREE.Group();
    
    // Wireframe Box matching robot dimensions
    const boxGeo = new THREE.BoxGeometry(this.car.width, this.car.height - 0.4, this.car.length);
    const boxMat = new THREE.MeshBasicMaterial({ 
      color: 0x00f2fe, 
      wireframe: true, 
      transparent: true, 
      opacity: 0.35 
    });
    const box = new THREE.Mesh(boxGeo, boxMat);
    box.position.y = 0.5;
    this.ghostGroup.add(box);

    // Directional pointer cone
    const coneGeo = new THREE.ConeGeometry(0.3, 0.9, 4);
    coneGeo.rotateX(Math.PI / 2);
    const coneMat = new THREE.MeshBasicMaterial({ 
      color: 0x00f2fe, 
      transparent: true, 
      opacity: 0.6 
    });
    const cone = new THREE.Mesh(coneGeo, coneMat);
    cone.position.set(0, 0.9, 1.2);
    this.ghostGroup.add(cone);

    this.scene.add(this.ghostGroup);
    this.ghostGroup.visible = false;
  }

  setEstimatedPose(x, z, yaw) {
    if (this.ghostGroup) {
      this.ghostGroup.position.x = x;
      this.ghostGroup.position.z = z;
      this.ghostGroup.position.y = this.getTerrainHeight(x, z) + 0.35;
      this.ghostGroup.rotation.y = yaw;
      this.ghostGroup.visible = true;
    }
  }

  clearEstimatedPose() {
    if (this.ghostGroup) {
      this.ghostGroup.visible = false;
    }
  }

  setTrackMode(mode) {
    this.trackCActive = false;
    if (mode === 'A') {
      this.trackAActive = true;
      this.robotGroup.visible = false;
      this.sedanGroup.visible = true;
      
      // Toggle headlights
      this.headlights.forEach(l => l.visible = false);
      this.sedanHeadlights.forEach(l => l.visible = this.showHeadlights);
      
      // Reset position
      this.loadMap(this.currentMapName);
      
      this.picoVM.log("[시스템] TRACK A (ROS2 자율주행 시뮬레이션) 전환 완료.", "success");
    } else if (mode === 'C') {
      this.trackAActive = false;
      this.trackCActive = true;
      this.robotGroup.visible = false;
      this.sedanGroup.visible = true;
      
      // Toggle headlights
      this.headlights.forEach(l => l.visible = false);
      this.sedanHeadlights.forEach(l => l.visible = this.showHeadlights);
      
      // Reset position
      this.loadMap(this.currentMapName);
      
      this.picoVM.log("[시스템] TRACK C (RL 강화학습 시뮬레이션) 전환 완료.", "success");
    } else {
      this.trackAActive = false;
      this.robotGroup.visible = true;
      this.sedanGroup.visible = false;
      
      // Toggle headlights
      this.headlights.forEach(l => l.visible = this.showHeadlights);
      this.sedanHeadlights.forEach(l => l.visible = false);
      
      // Reset position
      this.loadMap(this.currentMapName);
      
      this.picoVM.log("[시스템] TRACK B (Pico 임베디드 제어) 전환 완료.", "info");
    }
  }

  // Setup Ambient/Directional Lighting
  setupLighting() {
    this.ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(this.ambientLight);

    this.dirLight = new THREE.DirectionalLight(0xffffff, 1.0);
    this.dirLight.position.set(80, 150, 80);
    this.dirLight.castShadow = true;
    this.dirLight.shadow.mapSize.width = 2048;
    this.dirLight.shadow.mapSize.height = 2048;
    this.dirLight.shadow.camera.near = 0.5;
    this.dirLight.shadow.camera.far = 500;
    
    const d = 150;
    this.dirLight.shadow.camera.left = -d;
    this.dirLight.shadow.camera.right = d;
    this.dirLight.shadow.camera.top = d;
    this.dirLight.shadow.camera.bottom = -d;
    this.dirLight.shadow.bias = -0.0005;

    this.scene.add(this.dirLight);

    // Set active time of day defaults
    this.updateEnvironmentTime(this.timeOfDay);
  }

  // Update light colors, shadows, and fog based on environment settings
  updateEnvironmentTime(time) {
    this.timeOfDay = time;
    
    // Clear/Fog adjustments
    if (time === 'day') {
      this.scene.fog.color.setHex(0xe0f2fe);
      this.renderer.setClearColor(0xe0f2fe, 1);
      this.ambientLight.color.setHex(0xffffff);
      this.ambientLight.intensity = 0.7;
      
      this.dirLight.color.setHex(0xfef08a); // Warm sun
      this.dirLight.intensity = 1.0;
      this.dirLight.position.set(80, 150, 40);
      
      // Disable robot headlights visual bulb/light
      this.toggleHeadlightsState(false);
    } else if (time === 'sunset') {
      this.scene.fog.color.setHex(0x2d1a3c);
      this.renderer.setClearColor(0x2d1a3c, 1);
      this.ambientLight.color.setHex(0xf43f5e);
      this.ambientLight.intensity = 0.4;
      
      this.dirLight.color.setHex(0xfb923c); // Orange light
      this.dirLight.intensity = 0.6;
      this.dirLight.position.set(-60, 50, -40);
      
      this.toggleHeadlightsState(true);
    } else if (time === 'night') {
      this.scene.fog.color.setHex(0x060713);
      this.renderer.setClearColor(0x060713, 1);
      this.ambientLight.color.setHex(0x1e1b4b);
      this.ambientLight.intensity = 0.15;
      
      this.dirLight.color.setHex(0x38bdf8); // Moonlight
      this.dirLight.intensity = 0.25;
      this.dirLight.position.set(40, 120, -20);
      
      this.toggleHeadlightsState(true);
    }
  }

  toggleHeadlightsState(isActive) {
    this.showHeadlights = isActive;
    this.headlights.forEach(light => {
      light.visible = isActive;
    });
  }

  updatePhysics(dt) {
    if (this.trackAActive || this.trackCActive) {
      this.updateTrackAPhysics(dt);
      return;
    }
    // 1. Convert GPIO outputs to force variables
    // skid steering logic:
    // Left side active Fwd (GP3=1, GP2=0) => drives left side forward
    // Left side active Back (GP3=0, GP2=1) => drives left side backward
    // Right side active Fwd (GP8=1, GP7=0) => drives right side forward
    // Right side active Back (GP8=0, GP7=1) => drives right side backward
    // Enable pins GP4/GP6 duty cycles act as scaling factors (0-65535)

    const gpio = this.picoVM.gpio;
    const dutyA = gpio.GP4 / 65535.0; // Left motor power (0.0 - 1.0)
    const dutyB = gpio.GP6 / 65535.0; // Right motor power (0.0 - 1.0)

    let leftMotor = 0; // -1 to 1
    if (gpio.GP3 === 1 && gpio.GP2 === 0) leftMotor = 1;
    else if (gpio.GP3 === 0 && gpio.GP2 === 1) leftMotor = -1;

    let rightMotor = 0; // -1 to 1
    if (gpio.GP8 === 1 && gpio.GP7 === 0) rightMotor = 1;
    else if (gpio.GP8 === 0 && gpio.GP7 === 1) rightMotor = -1;

    // Scale by PWM duty cycles
    leftMotor *= dutyA;
    rightMotor *= dutyB;

    // Convert skid steering to Forward Speed & Yaw Rotation
    // If both motor directions are identical: translation speed.
    // If opposite: rotation.
    const forwardForce = (leftMotor + rightMotor) * 0.5 * this.car.acceleration;
    const turnForce = (leftMotor - rightMotor) * this.car.rotationSpeed; // Positive leftMotor turns car right, negative leftMotor turns left

    // Apply forces
    if (Math.abs(forwardForce) > 0.1) {
      this.car.speed += forwardForce * dt;
      // Clamp speed
      const clampSpeed = this.car.maxSpeed * Math.max(dutyA, dutyB);
      this.car.speed = Math.max(-clampSpeed * 0.6, Math.min(clampSpeed, this.car.speed));
    } else {
      // Natural Deceleration & Friction
      const f = this.car.friction * this.frictionCoeff;
      if (this.car.speed > 0) {
        this.car.speed = Math.max(0, this.car.speed - f * dt);
      } else if (this.car.speed < 0) {
        this.car.speed = Math.min(0, this.car.speed + f * dt);
      }
    }

    // Apply turn force (skid steer yaw angle)
    if (Math.abs(turnForce) > 0.1) {
      this.car.angle += turnForce * dt;
      
      // Animate wheels rotating in opposite directions for skid steer
      this.animateWheels(leftMotor, rightMotor, dt);
    } else {
      // Rotate wheels standard based on translation speed
      this.animateWheels(this.car.speed, this.car.speed, dt);
    }

    // Translate coordinates
    const dx = Math.sin(this.car.angle) * this.car.speed * dt;
    const dz = Math.cos(this.car.angle) * this.car.speed * dt;

    this.car.position.x += dx;
    this.car.position.z += dz;

    // Bounds checking (keep within campus grid boundaries)
    this.car.position.x = Math.max(-120, Math.min(120, this.car.position.x));
    this.car.position.z = Math.max(-120, Math.min(120, this.car.position.z));

    // Adapt Y position to terrain slope
    const targetY = this.getTerrainHeight(this.car.position.x, this.car.position.z) + 0.35;
    this.car.position.y = THREE.MathUtils.lerp(this.car.position.y, targetY, 0.2); // Smooth elevation changes

    // Update 3D Object transform
    this.robotGroup.position.copy(this.car.position);
    this.robotGroup.rotation.y = this.car.angle;

    // Check building collisions
    this.checkCollisions();
  }

  // Track A Kinematic Bicycle Model + Stanley/PID control loops
  updateTrackAPhysics(dt) {
    const wps = this.currentWaypoints;
    if (!wps || wps.length === 0) {
      // Free deceleration if no waypoints
      const f = this.car.friction * this.frictionCoeff;
      if (this.car.speed > 0) this.car.speed = Math.max(0, this.car.speed - f * dt);
      else if (this.car.speed < 0) this.car.speed = Math.min(0, this.car.speed + f * dt);
      
      // Translate
      const dx = Math.sin(this.car.angle) * this.car.speed * dt;
      const dz = Math.cos(this.car.angle) * this.car.speed * dt;
      this.car.position.x += dx;
      this.car.position.z += dz;
      
      this.sedanGroup.position.copy(this.car.position);
      this.sedanGroup.rotation.y = this.car.angle;
      return;
    }

    const yaw = this.car.angle;
    const speed = this.car.speed;
    const targetSpeed = this.trackATargetSpeed;

    // 1. FRONT AXLE LOCALIZATION (Stanley is referenced to front axle)
    const wheelbase = 2.8;
    const x_front = this.car.position.x + Math.sin(yaw) * (wheelbase / 2.0);
    const z_front = this.car.position.z + Math.cos(yaw) * (wheelbase / 2.0);

    // 2. FIND CLOSEST WAYPOINT
    let minIdx = 0;
    let minD = 9999.0;
    for (let i = 0; i < wps.length; i++) {
      const d = Math.hypot(x_front - wps[i].x, z_front - wps[i].z);
      if (d < minD) {
        minD = d;
        minIdx = i;
      }
    }

    // Determine path tangent angle at closest waypoint
    const nextIdx = (minIdx + 1) % wps.length;
    const tangentX = wps[nextIdx].x - wps[minIdx].x;
    const tangentZ = wps[nextIdx].z - wps[minIdx].z;
    const pathYaw = Math.atan2(tangentX, tangentZ);

    // Calculate signed Cross-Track Error (cte)
    let cte = minD;
    const vecX = x_front - wps[minIdx].x;
    const vecZ = z_front - wps[minIdx].z;
    // Cross product tangent x vec
    const cross = tangentX * vecZ - tangentZ * vecX;
    if (cross < 0) {
      cte = -cte; // vehicle is to the right of the path
    }

    // Heading error normalized to [-PI, PI]
    let yawError = pathYaw - yaw;
    yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));

    // 3. LATERAL CONTROL LAW SELECTION
    let steer = 0.0;
    if (this.trackCActive) {
      steer = this.trackASteeringAngle;
    } else if (this.trackAController === 'stanley') {
      const k = this.trackAStanleyGains.k;
      const ks = this.trackAStanleyGains.ks;
      // Stanley Law
      steer = yawError + Math.atan2(k * cte, speed + ks);
    } else {
      // Look-ahead PID path tracking (Pure Pursuit style target heading)
      const lookaheadIdx = (minIdx + 4) % wps.length;
      const lookPt = wps[lookaheadIdx];
      const lookYaw = Math.atan2(lookPt.x - this.car.position.x, lookPt.z - this.car.position.z);
      let lookYawError = lookYaw - yaw;
      lookYawError = Math.atan2(Math.sin(lookYawError), Math.cos(lookYawError));
      
      // Update Steer PID (gains linked to slider inputs)
      this.steerPIDSim.kp = this.trackAPIDGains.kp;
      this.steerPIDSim.ki = this.trackAPIDGains.ki;
      this.steerPIDSim.kd = this.trackAPIDGains.kd;
      steer = this.steerPIDSim.update(lookYawError, dt);
    }

    // Clamp steering to maximum steering angle (30 degrees)
    const maxSteer = 0.523;
    steer = Math.max(-maxSteer, Math.min(maxSteer, steer));

    // 4. LONGITUDINAL CONTROL (Speed PID)
    const effectiveTargetSpeed = this.trackCActive ? 5.0 : targetSpeed;
    const speedError = effectiveTargetSpeed - speed;
    const throttleBrake = this.speedPIDSim.update(speedError, dt);

    // Kinematic model acceleration and friction
    let accel = throttleBrake * 18.0;
    // Add simple aerodynamic drag/friction
    accel -= 0.3 * speed;
    
    // Update velocity
    this.car.speed += accel * dt;
    // Clamp forward speed
    const maxAllowedSpeed = this.trackCActive ? 5.0 : 22.0;
    this.car.speed = Math.max(-2.0, Math.min(this.car.speed, maxAllowedSpeed));

    // Update yaw heading based on steer angle (Bicycle Kinematics: dYaw = (V/L) * sin(Steer))
    const yawRate = (this.car.speed / wheelbase) * Math.sin(steer);
    this.car.angle += yawRate * dt;
    this.car.angle = Math.atan2(Math.sin(this.car.angle), Math.cos(this.car.angle)); // normalize

    // 5. TRANSLATION (Coordinates displacement)
    const dx = Math.sin(this.car.angle) * this.car.speed * dt;
    const dz = Math.cos(this.car.angle) * this.car.speed * dt;

    this.car.position.x += dx;
    this.car.position.z += dz;

    // Bounds checking
    this.car.position.x = Math.max(-120, Math.min(120, this.car.position.x));
    this.car.position.z = Math.max(-120, Math.min(120, this.car.position.z));

    // Elevation terrain matching
    const targetY = this.getTerrainHeight(this.car.position.x, this.car.position.z) + 0.35;
    this.car.position.y = THREE.MathUtils.lerp(this.car.position.y, targetY, 0.2);

    // Update visual chassis transforms
    this.sedanGroup.position.copy(this.car.position);
    this.sedanGroup.rotation.y = this.car.angle;

    // Animate wheels rotation
    const roll = this.car.speed * dt / 0.6; // angle roll = dx / radius
    this.sedanWheels.forEach(w => w.rotation.x += roll);
    // Apply front visual steering angles
    this.sedanWheels[0].rotation.y = steer;
    this.sedanWheels[1].rotation.y = steer;

    // Save states for telemetry
    this.trackASteeringAngle = steer;
    this.trackASpeedError = speedError;
    this.trackACTE = cte;
    this.trackAHeadingError = yawError;

    // Collisions
    this.checkCollisions();
  }

  // Skid steer wheel animation
  animateWheels(leftForce, rightForce, dt) {
    const rotSpeed = 15.0; // visual speed scale
    
    // Front Left (wheel 0) and Rear Left (wheel 2)
    this.wheels[0].rotation.x += leftForce * rotSpeed * dt;
    this.wheels[2].rotation.x += leftForce * rotSpeed * dt;
    
    // Front Right (wheel 1) and Rear Right (wheel 3)
    this.wheels[1].rotation.x += rightForce * rotSpeed * dt;
    this.wheels[3].rotation.x += rightForce * rotSpeed * dt;

    // Add visual wheel steering wiggle just for aesthetic realism when turning
    const steerWiggle = (leftForce - rightForce) * 0.25;
    this.wheels[0].rotation.y = steerWiggle;
    this.wheels[1].rotation.y = steerWiggle;
  }

  // Simple bounding cylinder collision detection
  checkCollisions() {
    for (let i = 0; i < this.buildings.length; i++) {
      const b = this.buildings[i];
      const dist = this.car.position.distanceTo(b.mesh.position);
      
      // Calculate buffer distance based on building boundary + car radius
      const collisionThreshold = b.radius + 1.8;
      
      if (dist < collisionThreshold) {
        // Collision occured! Push car back
        const pushDir = new THREE.Vector3().subVectors(this.car.position, b.mesh.position).normalize();
        pushDir.y = 0; // Ground only
        
        // Push slightly outside boundary
        this.car.position.addScaledVector(pushDir, (collisionThreshold - dist) + 0.2);
        
        // Kill momentum
        this.car.speed = -this.car.speed * 0.3; // bounce slightly
        
        this.hasCollidedThisTick = true;

        // Log to Virtual Pico VM
        this.picoVM.log(`[시스템 경고] ${b.name} 충돌 발생! 차량 충돌 보호 구동 정지.`, 'error');
      }
    }
  }

  updateUltrasonicSensor() {
    if (this.trackAActive || this.trackCActive) {
      this.updateLidarSensor();
      return;
    }
    // 1. Compute raycast starting from front bumper pointing forward
    // Direction vector of the car
    const dir = new THREE.Vector3(Math.sin(this.car.angle), 0, Math.cos(this.car.angle)).normalize();
    const origin = this.car.position.clone().addScaledVector(dir, 1.85); // bumper edge
    origin.y += 0.25; // height offset matching sensor mount

    // 2. Perform intersection raycasting manually against building meshes
    let closestDist = 400.0; // 400cm max range
    let hitPoint = origin.clone().addScaledVector(dir, closestDist);
    let hitMesh = null;

    this.buildings.forEach(b => {
      // Represent buildings as cylinders or axis-aligned bounding boxes for fast distance checks
      // Simple ray-to-cylinder math
      const toB = new THREE.Vector3().subVectors(b.mesh.position, origin);
      const projection = toB.dot(dir);
      
      if (projection > 0 && projection < closestDist) {
        // Find closest point on ray to building center
        const closestPtOnRay = origin.clone().addScaledVector(dir, projection);
        const distToCenter = closestPtOnRay.distanceTo(b.mesh.position);
        
        if (distToCenter < b.radius) {
          // It hits! Calculate hit distance from origin to boundary
          const offset = Math.sqrt(b.radius * b.radius - distToCenter * distToCenter);
          const hitDist = projection - offset;
          
          if (hitDist > 0 && hitDist < closestDist) {
            closestDist = hitDist;
            hitMesh = b.name;
          }
        }
      }
    });

    // Translate metric units to simulated centimeters (1 Three.js meter = 100 cm)
    // Map bounds check
    const mapLimitDist = 120.0; // map border limit
    // Check if ray hits map edge
    const distToBorderX = Math.abs((dir.x > 0 ? 120 : -120) - origin.x) / (Math.abs(dir.x) || 0.0001);
    const distToBorderZ = Math.abs((dir.z > 0 ? 120 : -120) - origin.z) / (Math.abs(dir.z) || 0.0001);
    const distToBorder = Math.min(distToBorderX, distToBorderZ);
    
    if (distToBorder < closestDist) {
      closestDist = distToBorder;
    }

    this.sensorDistance = Math.round(closestDist * 100); // meters to cm

    // 3. Render visual indicator laser and targets
    if (this.showLaser) {
      this.laserLine.visible = true;
      this.laserTargetRing.visible = true;

      // Update positions
      const actualHitPoint = origin.clone().addScaledVector(dir, closestDist);
      const linePositions = this.laserLine.geometry.attributes.position;
      linePositions.setXYZ(0, origin.x, origin.y, origin.z);
      linePositions.setXYZ(1, actualHitPoint.x, actualHitPoint.y, actualHitPoint.z);
      linePositions.needsUpdate = true;

      this.laserTargetRing.position.copy(actualHitPoint);
      this.laserTargetRing.position.y = this.getTerrainHeight(actualHitPoint.x, actualHitPoint.z) + 0.1;
      
      // Animate color based on danger: Green (safe) -> Yellow (close) -> Red (extremely close)
      if (this.sensorDistance < 50) {
        this.laserMaterial.color.setHex(0xff3366); // red
        this.laserTargetRing.material.color.setHex(0xff3366);
      } else if (this.sensorDistance < 120) {
        this.laserMaterial.color.setHex(0xff9f43); // yellow/orange
        this.laserTargetRing.material.color.setHex(0xff9f43);
      } else {
        this.laserMaterial.color.setHex(0x00f2fe); // cyan
        this.laserTargetRing.material.color.setHex(0x00f2fe);
      }
    } else {
      this.laserLine.visible = false;
      this.laserTargetRing.visible = false;
    }

    // Trigger VM callback to simulate Echo/Trig hardware pin waveforms
    this.picoVM.measureDistance(this.sensorDistance);
  }

  // Track A spinning LiDAR scan simulator
  updateLidarSensor() {
    // 1. Spin Lidar raycast direction around 360 degrees
    this.lidarAngle += 0.12; // speed of rotation (radians per tick)
    if (this.lidarAngle > Math.PI * 2) this.lidarAngle -= Math.PI * 2;

    const currentYaw = this.car.angle;
    const origin = this.car.position.clone();
    origin.y += 1.6; // height of Lidar mount on top of roof

    // Lidar ray heading direction
    const sweepYaw = currentYaw + this.lidarAngle;
    const dir = new THREE.Vector3(Math.sin(sweepYaw), 0, Math.cos(sweepYaw)).normalize();

    // 2. Perform intersection checks
    let closestDist = 80.0; // 80m max LiDAR range
    this.buildings.forEach(b => {
      const toB = new THREE.Vector3().subVectors(b.mesh.position, origin);
      const projection = toB.dot(dir);
      if (projection > 0 && projection < closestDist) {
        const closestPt = origin.clone().addScaledVector(dir, projection);
        const distToCenter = closestPt.distanceTo(b.mesh.position);
        if (distToCenter < b.radius) {
          const offset = Math.sqrt(b.radius * b.radius - distToCenter * distToCenter);
          const hitDist = projection - offset;
          if (hitDist > 0 && hitDist < closestDist) {
            closestDist = hitDist;
          }
        }
      }
    });

    // Save range in centimeters to match standard sensor variables
    this.sensorDistance = Math.round(closestDist * 100);

    // 3. Render rotating laser beam line
    if (this.showLaser) {
      this.laserLine.visible = true;
      this.laserTargetRing.visible = true;

      const hitPoint = origin.clone().addScaledVector(dir, closestDist);
      const linePositions = this.laserLine.geometry.attributes.position;
      linePositions.setXYZ(0, origin.x, origin.y, origin.z);
      linePositions.setXYZ(1, hitPoint.x, hitPoint.y, hitPoint.z);
      linePositions.needsUpdate = true;

      this.laserTargetRing.position.copy(hitPoint);
      this.laserTargetRing.position.y = this.getTerrainHeight(hitPoint.x, hitPoint.z) + 0.1;
      
      // Cyan scan pulse visual coloring
      this.laserMaterial.color.setHex(0x00f2fe);
      this.laserTargetRing.material.color.setHex(0x00f2fe);
    } else {
      this.laserLine.visible = false;
      this.laserTargetRing.visible = false;
    }
  }

  // Update cameras based on modes
  updateCameras() {
    const dir = new THREE.Vector3(Math.sin(this.car.angle), 0, Math.cos(this.car.angle)).normalize();
    
    // FPV Camera follows front bumper looking forward
    this.fpvCamera.position.copy(this.car.position).addScaledVector(dir, 1.8);
    this.fpvCamera.position.y += 0.8; // height of camera mount
    const fpvTarget = this.car.position.clone().addScaledVector(dir, 20);
    fpvTarget.y += 0.6;
    this.fpvCamera.lookAt(fpvTarget);

    // Main viewport camera modes
    if (this.cameraMode === 'fpv') {
      this.camera.position.copy(this.fpvCamera.position);
      this.camera.rotation.copy(this.fpvCamera.rotation);
      this.controls.enabled = false;
    } else if (this.cameraMode === 'chase') {
      this.controls.enabled = false;
      const targetPos = this.car.position.clone().addScaledVector(dir, -15);
      targetPos.y += 6;
      this.camera.position.lerp(targetPos, 0.1);
      
      const lookTarget = this.car.position.clone();
      lookTarget.y += 1;
      this.camera.lookAt(lookTarget);
    } else if (this.cameraMode === 'top') {
      this.controls.enabled = false;
      this.camera.position.set(this.car.position.x, 60, this.car.position.z);
      this.camera.lookAt(this.car.position);
    } else {
      // Orbit View - controls handles movement, but let controls focus on target
      this.controls.enabled = true;
      this.controls.target.copy(this.car.position);
      this.controls.update();
    }
  }

  // Telemetry updates
  updateTelemetry() {
    let steerValue = 0;
    if (this.trackAActive) {
      // Steer value in degrees for Track A
      steerValue = Math.round(this.trackASteeringAngle * 180 / Math.PI);
    } else {
      // Steer value for Track B based on GP direction differences
      steerValue = Math.round((this.picoVM.gpio.GP3 - this.picoVM.gpio.GP8) * this.picoVM.pwmPercent * 0.3);
    }

    const data = {
      speed: Math.abs(this.car.speed).toFixed(1),
      steer: steerValue.toFixed(0),
      distance: this.sensorDistance,
      heading: Math.round(((this.car.angle * 180 / Math.PI) % 360 + 360) % 360),
      x: this.car.position.x,
      z: this.car.position.z,
      // Track A variables
      trackAActive: this.trackAActive,
      cte: this.trackACTE,
      speedError: this.trackASpeedError,
      headingError: this.trackAHeadingError
    };
    this.onTelemetryUpdate(data);
  }

  // Visualize Detailed Road Map Waypoints in 3D
  drawWaypoints(waypoints) {
    // Clear previous visual children
    while(this.waypointGroup.children.length > 0) { 
      this.waypointGroup.remove(this.waypointGroup.children[0]); 
    }

    if (!waypoints || waypoints.length === 0) {
      this.currentWaypoints = [];
      return;
    }
    this.currentWaypoints = waypoints;

    const sphereGeo = new THREE.SphereGeometry(0.8, 8, 8);
    const sphereMat = new THREE.MeshBasicMaterial({ 
      color: 0x00f2fe, 
      transparent: true, 
      opacity: 0.8 
    });

    const linePoints = [];

    waypoints.forEach((wp, idx) => {
      const y = this.getTerrainHeight(wp.x, wp.z) + 1.2; // float 1.2m above ground
      
      // Floating Sphere Mesh
      const mesh = new THREE.Mesh(sphereGeo, sphereMat);
      mesh.position.set(wp.x, y, wp.z);
      // store base height for floating animation
      mesh.userData = { baseHeight: y, index: idx };
      this.waypointGroup.add(mesh);

      linePoints.push(new THREE.Vector3(wp.x, y, wp.z));
    });

    // Close the loop
    linePoints.push(linePoints[0].clone());

    // Connect with dashed lines representing the road track
    const pathGeo = new THREE.BufferGeometry().setFromPoints(linePoints);
    const pathMat = new THREE.LineBasicMaterial({ 
      color: 0x4facfe, 
      transparent: true, 
      opacity: 0.5 
    });
    
    const pathLine = new THREE.Line(pathGeo, pathMat);
    this.waypointGroup.add(pathLine);
  }

  resetVehicleToStart() {
    if (this.currentMapName === 'obstacle-course') {
      this.car.position.set(0, 0.5, 40);
      this.car.angle = Math.PI;
    } else {
      this.car.position.set(0, 0.5, 80);
      this.car.angle = Math.PI;
    }
    this.car.velocity.set(0, 0, 0);
    this.car.speed = 0;

    // Reset controllers
    if (this.speedPIDSim) this.speedPIDSim.reset();
    if (this.steerPIDSim) this.steerPIDSim.reset();
    this.trackASteeringAngle = 0.0;
    this.trackASpeedError = 0.0;
    this.trackACTE = 0.0;
    this.trackAHeadingError = 0.0;
    this.hasCollidedThisTick = false;

    // Update meshes immediately
    if (this.robotGroup) {
      this.robotGroup.position.copy(this.car.position);
      this.robotGroup.rotation.y = this.car.angle;
    }
    if (this.sedanGroup) {
      this.sedanGroup.position.copy(this.car.position);
      this.sedanGroup.rotation.y = this.car.angle;
    }
  }

  tickRLAgent(dt, isLearning = true) {
    if (!window.rlAgent) return;

    const agent = window.rlAgent;
    const cte = this.trackACTE;
    const yawError = this.trackAHeadingError;
    const sensorDist = this.sensorDistance; // in cm

    // 1. Check for episode termination (done state)
    let done = false;
    let collision = this.hasCollidedThisTick;
    this.hasCollidedThisTick = false; // Reset for next tick

    // Drifting too far from the road map boundary (|e_cte| > 12.0 meters)
    let outOfBounds = Math.abs(cte) > 12.0;

    if (collision || outOfBounds) {
      done = true;
    }

    // 2. Calculate Reward
    let reward = 0.0;
    if (done) {
      reward = -agent.wCollision;
    } else {
      const progressReward = this.car.speed * Math.cos(yawError) * agent.wSpeed;
      const ctePenalty = -Math.abs(cte) * agent.wCte;
      reward = progressReward + ctePenalty;
    }

    // Store the reward on agent so it can be picked up by telemetry
    agent.lastReward = reward;

    // 3. Perform Q-learning Update
    if (isLearning) {
      agent.updateQTable(cte, yawError, sensorDist, reward, done);
    }

    // 4. If episode is done, reset vehicle and notify dashboard
    if (done) {
      const finalReward = agent.endEpisode();
      this.resetVehicleToStart();
      if (this.onRLEpisodeEnd) {
        this.onRLEpisodeEnd(agent.episodeCount, finalReward);
      }
    } else {
      // Choose next action (returns steering angle in radians)
      this.trackASteeringAngle = agent.selectAction(cte, yawError, sensorDist);
    }
  }

  // Animation Loop
  animate() {
    requestAnimationFrame(() => this.animate());

    const dt = Math.min(0.03, this.clock.getDelta()); // clamp DT to avoid physics jumps

    // Animate floating waypoints
    if (this.waypointGroup) {
      const time = Date.now() * 0.003;
      this.waypointGroup.children.forEach(child => {
        if (child.userData && child.userData.baseHeight !== undefined) {
          child.position.y = child.userData.baseHeight + Math.sin(time + child.userData.index) * 0.25;
        }
      });
    }

    // Spin Lidar Visual turret disc
    if (this.lidarDisk && (this.trackAActive || this.trackCActive)) {
      this.lidarDisk.rotation.y += dt * 8;
    }

    // Update Physics and Hardware
    if (this.trackCActive && this.rlTrainingActive) {
      const iterations = this.trackCSpeedup || 1;
      for (let i = 0; i < iterations; i++) {
        this.updatePhysics(dt);
        this.updateUltrasonicSensor();
        this.tickRLAgent(dt, true);
      }
    } else {
      this.updatePhysics(dt);
      this.updateUltrasonicSensor();
      if (this.trackCActive) {
        this.tickRLAgent(dt, false);
      }
    }

    this.updateCameras();
    this.updateTelemetry();

    // Render Scene (Main)
    if (this.cameraMode === 'fpv') {
      this.renderer.render(this.scene, this.fpvCamera);
    } else {
      this.renderer.render(this.scene, this.camera);
    }

    // Render FPV window always
    this.fpvRenderer.render(this.scene, this.fpvCamera);
  }
}

// Export for browser usage
window.SimulationEngine = SimulationEngine;
