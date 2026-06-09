# R-pi-picow-Autonomous-RC (3D Real-Time Simulation)

Welcome to the revised **Raspberry Pi Pico W Autonomous RC Car** project. 

To overcome the physical voltage, current, and sensor integration limitations identified in the original hardware implementation (as detailed in the [project presentation PDF](./Pi%20Pico-w%20%EC%A3%BC%ED%96%89%EB%A1%9C%EB%B4%87%20%ED%94%84%EB%A1%9C%EC%A0%9D%ED%8A%B8.pdf)), this version introduces a high-fidelity, real-time **3D WebGL Simulation Dashboard**.

## 🌟 Key Features

1. **Real-world 3D Data Maps**:
   - Drive the simulated RC car on a virtual representation of the **KNUT Chungju Campus** (Main Campus) with realistic terrain elevation, roads, trees, and buildings (such as the Central Library, Administration Building, and Student Center).
   - Test obstacle avoidance on a specialized **Obstacle Course** featuring walls, columns, and narrow corridors.
2. **First Person View (FPV) Camera**:
   - A virtual camera mounted on the front bumper renders a live FPV feed in a picture-in-picture viewport.
3. **Simulated HC-SR04 Ultrasonic Sensor**:
   - Casts a dynamic 15-degree cone raycast in real-time, calculating distances to buildings, trees, and barriers, resolving the original project's physical sensor power issues.
4. **Virtual Pi Pico W Microcontroller VM**:
   - Emulates the console debugging outputs and socket HTTP server logs.
   - Visually indicates the status of GPIO Pins on an interactive breadboard graphic:
     - **GP2, GP3, GP7, GP8**: Motor direction pins (Left & Right H-Bridge logic).
     - **GP4, GP6**: Enable pins showing PWM duty cycle widths (speed percentage).
     - **GP5, GP9**: Ultrasonic Trigger & Echo pin pulse signals.
5. **Autopilot Mode**:
   - An autonomous obstacle-avoidance algorithm that monitors the sensor distance and steers the robot away from buildings or barriers in real-time.
6. **Time of Day & Environment Settings**:
   - Toggle between **Day**, **Sunset**, and **Night** modes. In night/sunset modes, the robot's headlights cast dynamic spotlight shadows onto the terrain.
7. **Dual Controls**:
   - Control the car using the GUI dashboard buttons (simulating original HTTP requests) or drive directly using **WASD** / **Arrow Keys** with keyboard helper indicators.

---

## 📁 File Structure

```text
├── main.py                  # Original MicroPython code (reference)
├── index.html               # Main simulation dashboard shell
├── style.css                # Glassmorphic UI styles
├── js/
│   ├── pico_vm.js           # Pi Pico W VM & socket log emulator
│   ├── simulation.js        # Three.js 3D renderer, physics & raycasting
│   └── dashboard.js         # Input handlers, telemetry & autopilot loop
├── package.json             # Dev server configuration (Vite)
└── README.md                # Project documentation
```

---

## 🚀 Quick Start

### Prerequisites
- [Node.js](https://nodejs.org/) (includes npm) installed on your system.

### Running the Simulator
1. Open your terminal in the project directory.
2. Install the lightweight development server:
   ```bash
   npm install
   ```
3. Launch the development server:
   ```bash
   npm run dev
   ```
4. Click the local server link in the terminal (typically `http://localhost:5173`) to open the simulator in your browser.

---

## 🛠️ Emulation Technical Details

- **Skid Steering**: Since the robot has 2 independently driven side wheels, steering is modeled via skid steering (applying opposite rotation speeds to Motor A and Motor B), matching the L298N H-Bridge hardware output.
- **GPS Displacement**: The dashboard calculates real-time latitude and longitude values by displacing the actual campus base coordinates (Chungju Campus: `36.969722°N, 127.871389°E`) using the car's local translation vector.
- **MicroPython Web Server logs**: The VM mimics connection timeouts and logs the exact HTTP socket request format from the original code (e.g. `GET /forward? HTTP/1.1`) as commands are sent.
