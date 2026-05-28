/**
 * Pi Pico-W Virtual Machine (Hardware & Network Emulator)
 * Bridges the original MicroPython logic with the browser-based simulation.
 */
class PicoVM {
  constructor(onLogCallback, onGpioChangeCallback) {
    this.onLog = onLogCallback || (() => {});
    this.onGpioChange = onGpioChangeCallback || (() => {});
    
    // Original configuration matching main.py
    this.ssid = 'Wi_KNUT';
    this.ip = '192.168.213.21'; // Simulated local IP
    this.port = 80;
    
    // GPIO States (0 = Low, 1 = High, numbers for PWM)
    this.gpio = {
      GP2: 0,  // Mot_A_Forward
      GP3: 0,  // Mot_A_Back
      GP7: 0,  // Mot_B_Forward
      GP8: 0,  // Mot_B_Back
      GP4: 45875, // ENA PWM (default 70% = 45875/65535)
      GP6: 45875, // ENB PWM (default 70% = 45875/65535)
      GP5: 0,  // Trigger (Ultrasonic)
      GP9: 0   // Echo (Ultrasonic)
    };
    
    this.connectionState = 'DISCONNECTED'; // DISCONNECTED, CONNECTING, CONNECTED
    this.pwmPercent = 70; // Cache percentage
  }

  // Add system logs
  log(message, type = '') {
    const timestamp = new Date().toLocaleTimeString();
    this.onLog(`[${timestamp}] ${message}`, type);
  }

  // Network initialization matching connect()
  async boot() {
    this.connectionState = 'CONNECTING';
    this.log(`SSID '${this.ssid}' 연결 중...`, 'info');
    
    for (let i = 1; i <= 3; i++) {
      await new Promise(resolve => setTimeout(resolve, 800));
      this.log('연결 대기...');
    }
    
    this.connectionState = 'CONNECTED';
    this.log(`Wi-Fi 연결 완료. IP 주소: ${this.ip}`, 'success');
    this.log(`소켓 열기 완료: ${this.ip}:${this.port}`, 'info');
    this.log(`웹 서버 대기 중...`, 'info');
    this.notifyGpio();
  }

  // Trigger GPIO state changes
  notifyGpio() {
    this.onGpioChange(this.gpio);
  }

  // Set PWM Duty Cycle (from slider, translated to duty_u16)
  setPWM(percent) {
    this.pwmPercent = percent;
    // Maps 0-100% to 0-65535
    const duty = Math.round((percent / 100) * 65535);
    this.gpio.GP4 = duty;
    this.gpio.GP6 = duty;
    this.log(`PWM 주파수 설정: 1000Hz, 듀티 사이클: ${percent}% (duty_u16: ${duty})`, 'cmd');
    this.notifyGpio();
  }

  // Receive Virtual HTTP Socket Requests
  receiveRequest(urlPath) {
    if (this.connectionState !== 'CONNECTED') {
      this.log('경고: 소켓이 연결되지 않았습니다.', 'error');
      return;
    }

    this.log(`HTTP request: GET ${urlPath} HTTP/1.1`, 'cmd');
    
    let commandHandled = true;
    switch(urlPath) {
      case '/forward?':
        this.move_forward();
        break;
      case '/back?':
        this.move_backward();
        break;
      case '/left?':
        this.move_left();
        break;
      case '/right?':
        this.move_right();
        break;
      case '/stop?':
        this.move_stop();
        break;
      default:
        commandHandled = false;
        break;
    }

    if (commandHandled) {
      this.notifyGpio();
    }
    
    // Simulate serving the webpage HTML template
    const htmlResponse = this.getWebpageHTML();
    return htmlResponse;
  }

  // Move Methods matching main.py logic
  // Original notes: In main.py:
  // Mot_A_Forward (GP2), Mot_A_Back (GP3), Mot_B_Forward (GP7), Mot_B_Back (GP8)
  // move_forward(): Mot_A_Forward=0, Mot_B_Forward=0, Mot_A_Back=1, Mot_B_Back=1
  // move_backward(): Mot_A_Forward=1, Mot_B_Forward=1, Mot_A_Back=0, Mot_B_Back=0
  // move_stop(): Mot_A_Forward=0, Mot_B_Forward=0, Mot_A_Back=0, Mot_B_Back=0
  // move_left(): Mot_A_Forward=0, Mot_B_Forward=1, Mot_A_Back=1, Mot_B_Back=0
  // move_right(): Mot_A_Forward=1, Mot_B_Forward=0, Mot_A_Back=0, Mot_B_Back=1

  move_forward() {
    this.gpio.GP2 = 0;
    this.gpio.GP3 = 1;
    this.gpio.GP7 = 0;
    this.gpio.GP8 = 1;
    this.log('move_forward() 호출됨 (모터 A/B 전진구동)', 'success');
  }

  move_backward() {
    this.gpio.GP2 = 1;
    this.gpio.GP3 = 0;
    this.gpio.GP7 = 1;
    this.gpio.GP8 = 0;
    this.log('move_backward() 호출됨 (모터 A/B 후진구동)', 'success');
  }

  move_left() {
    this.gpio.GP2 = 0;
    this.gpio.GP3 = 1;
    this.gpio.GP7 = 1;
    this.gpio.GP8 = 0;
    this.log('move_left() 호출됨 (모터 A 전진 / 모터 B 후진 -> 좌회전)', 'success');
  }

  move_right() {
    this.gpio.GP2 = 1;
    this.gpio.GP3 = 0;
    this.gpio.GP7 = 0;
    this.gpio.GP8 = 1;
    this.log('move_right() 호출됨 (모터 A 후진 / 모터 B 전진 -> 우회전)', 'success');
  }

  move_stop() {
    this.gpio.GP2 = 0;
    this.gpio.GP3 = 0;
    this.gpio.GP7 = 0;
    this.gpio.GP8 = 0;
    this.log('move_stop() 호출됨 (모터 구동 정지)', 'info');
  }

  // Simulated Ultrasonic HC-SR04 Trigger / Echo loop
  async measureDistance(actualDistanceCm) {
    if (this.connectionState !== 'CONNECTED') return actualDistanceCm;

    // Trigger pin high
    this.gpio.GP5 = 1;
    this.notifyGpio();
    
    // Keep high for trigger pulse
    await new Promise(resolve => setTimeout(resolve, 10)); // 10ms trigger
    
    this.gpio.GP5 = 0;
    this.notifyGpio();
    
    // Echo pin goes high for a duration based on distance
    this.gpio.GP9 = 1;
    this.notifyGpio();
    
    // Echo duration (simulating pulse)
    const delay = Math.min(100, Math.max(5, Math.round(actualDistanceCm * 0.5)));
    await new Promise(resolve => setTimeout(resolve, delay));
    
    this.gpio.GP9 = 0;
    this.notifyGpio();
    
    return actualDistanceCm;
  }

  // Get raw html matching webpage() function in main.py
  getWebpageHTML() {
    return `
<!DOCTYPE html>
<html>
<head>
  <title>Zumo Robot Control</title>
</head>
<body>
  <center>
    <b>
    <form action="./forward">
      <input type="submit" value="Forward" style="height:120px; width:120px" />
    </form>
    <table>
      <tr>
        <td>
          <form action="./left">
            <input type="submit" value="Left" style="height:120px; width:120px" />
          </form>
        </td>
        <td>
          <form action="./stop">
            <input type="submit" value="Stop" style="height:120px; width:120px" />
          </form>
        </td>
        <td>
          <form action="./right">
            <input type="submit" value="Right" style="height:120px; width:120px" />
          </form>
        </td>
      </tr>
    </table>
    <form action="./back">
      <input type="submit" value="Back" style="height:120px; width:120px" />
    </form>
  </center>
</body>
</html>`;
  }
}

// Export for browser usage
window.PicoVM = PicoVM;
