import network
import socket
from machine import Pin, PWM, reset
import time

#네트워크 기기와 임베디드 시스템은 같은 IP를 공유해야 합니다.
ssid = 'Wi_KNUT' #네트워크 이름
password = '' #네트워크 패스워드

#핀모터 매소드를 정의합니다.
Mot_A_Forward = Pin(2, Pin.OUT)
Mot_A_Back = Pin(3, Pin.OUT)
Mot_B_Forward = Pin(7, Pin.OUT)
Mot_B_Back = Pin(8, Pin.OUT)
ENA = Pin(4, Pin.OUT)
ENB = Pin(6, Pin.OUT)
pwm_A = PWM(ENA)
pwm_B = PWM(ENB)
#echo_pin = machine.Pin(9, machine.Pin.IN)
#trigger_pin = machine.Pin(5,machine.Pin.OUT)

#PWM 주파수를 설정합니다.
pwm_A.freq(1000)
pwm_B.freq(1000)

# PWM 듀티 사이클을 설정합니다. (0-65535 사이의 값)
pwm_A.duty_u16(45875)  # 70% 듀티 사이클
pwm_B.duty_u16(45875)  # 70% 듀티 사이클

def move_forward():
    
    Mot_A_Forward.value(0)
    Mot_B_Forward.value(0)
    Mot_A_Back.value(1)
    Mot_B_Back.value(1)
    
def move_backward():
    Mot_A_Forward.value(1)
    Mot_B_Forward.value(1)
    Mot_A_Back.value(0)
    Mot_B_Back.value(0)
    
def move_stop():
    Mot_A_Forward.value(0)
    Mot_B_Forward.value(0)
    Mot_A_Back.value(0)
    Mot_B_Back.value(0)

def move_left():
    Mot_A_Forward.value(0)
    Mot_B_Forward.value(1)
    Mot_A_Back.value(1)
    Mot_B_Back.value(0)

def move_right():
    Mot_A_Forward.value(1)
    Mot_B_Forward.value(0)
    Mot_A_Back.value(0)
    Mot_B_Back.value(1)

#임베디드 시스템을 멈추게 합니다.
move_stop()
    
def connect():
    #WLAN 연결
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while wlan.isconnected() == False:
        print('연결 대기...')
        sleep(1)
    ip = wlan.ifconfig()[0]
    print(f'연결됨 {ip}')
    return ip
    
def open_socket(ip):
    #IP 소켓 오픈
    address = (ip, 80)
    connection = socket.socket()
    connection.bind(address)
    connection.listen(1)
    return connection

def webpage():
    #HTML 디자인
    html = f"""
            <!DOCTYPE html>
            <html>
            <head>
            <title>Zumo Robot Control</title>
            </head>
            <center><b>
            <form action="./forward">
            <input type="submit" value="Forward" style="height:120px; width:120px" />
            </form>
            <table><tr>
            <td><form action="./left">
            <input type="submit" value="Left" style="height:120px; width:120px" />
            </form></td>
            <td><form action="./stop">
            <input type="submit" value="Stop" style="height:120px; width:120px" />
            </form></td>
            <td><form action="./right">
            <input type="submit" value="Right" style="height:120px; width:120px" />
            </form></td>
            </tr></table>
            <form action="./back">
            <input type="submit" value="Back" style="height:120px; width:120px" />
            </form>
            </body>
            </html>
            """
    return str(html)

def serve(connection):
    #웹서버 시작
    while True:
        client = connection.accept()[0]
        request = client.recv(1024)
        request = str(request)
        try:
            request = request.split()[1]
        except IndexError:
            pass
        if request == '/forward?':
            move_forward()
        elif request =='/left?':
            move_left()
        elif request =='/stop?':
            move_stop()
        elif request =='/right?':
            move_right()
        elif request =='/back?':
            move_backward()
        html = webpage()
        client.send(html)
        client.close()

try:
    ip = connect()
    connection = open_socket(ip)
    serve(connection)

except KeyboardInterrupt:
    machine.reset()
    
"""

#초음파센서 매소드를 정의합니다.
def measure_distance():
    trigger_pin.value(1)
    time.sleep_us(10)
    trigger_pin.value(0)

    while echo_pin.value() == 0:
        pulse_start = time.ticks_us()

    while echo_pin.value() == 1:
        pulse_end = time.ticks_us()

    pulse_duration = pulse_end - pulse_start

    distance_cm = pulse_duration / 58.0

    return distance_cm

for i in range(100):
    distance = measure_distance()

    print("distance_cm: {:.2f} cm".format(distance_cm))

    time.sleep(0.1)

"""
