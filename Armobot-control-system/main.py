import network
import socket
import time
from machine import Pin, Timer, time_pulse_us, PWM
import utime
import json
import gc

delay = 400

# Import DNS Server for captive portal
try:
    from microDNSSrv import MicroDNSSrv
    DNS_AVAILABLE = True
except:
    DNS_AVAILABLE = False
    print("⚠️ DNS server not available")

# Access Point credentials
AP_SSID = 'RoboticArm_AP'
AP_PASSWORD = '12345678'


class Gripper:
    def __init__(self):
        SERVO_PIN = 26
        self.servo = machine.PWM(machine.Pin(SERVO_PIN))
        self.servo.freq(50)
        self.MIN_DUTY = 1638
        self.MAX_DUTY = 8192

    def set_angle(self, angle):
        duty = int((angle / 180) * (self.MAX_DUTY - self.MIN_DUTY) + self.MIN_DUTY)
        self.servo.duty_u16(duty)

    def open(self):
        self.set_angle(0)
        utime.sleep_ms(500)

    def close(self):
        self.set_angle(85)
        utime.sleep_ms(500)


class Joint:
    def __init__(self, dir, pulse, maxDegree, minDegree, maxPulse, degreeToPulseRatio):
        self.dirPin = Pin(dir, Pin.OUT)
        self.pulsePin = Pin(pulse, Pin.OUT)
        self.maxDegree = maxDegree
        self.minDegree = minDegree
        self.degreeToPulseRatio = degreeToPulseRatio
        self.maxPulse = maxPulse
        self.currentDegree = 0

    def jointDir(self, value):
        self.dirPin.value(0 if value else 1)

    def jointPul(self, value):
        self.pulsePin.value(value)


# Stepper parameters
maxDegree1 = 150.0
minDegree1 = -175.0
maxPulse1 = 3875

minDegree2 = -20.0
maxDegree2 = 70.0
degreeToPulseRatio2 = 2625.0 / 90.0

pulsesPerDegree3 = 2500.0 / 90.0
minDegree3 = -85.0
maxDegree3 = 85.0

saved_movement_1 = []
saved_movement_2 = []

default_p1 = [-50.0, 50.0, -20.0]
default_p2 = [40.0, 30.0, -30.0]

continuous_run = False
gripper_state = 'closed'  # track open/close for dashboard


min_s2 = -20
max_s2 = 70
min_s3 = -85
max_s3 = 85

import math

l1 = 18
l2 = 21
l3 = 35


def solve_d3(d2):
    global max_s3, min_s3
    d2_rad = math.radians(d2)
    value = (l1 + l2 * math.cos(d2_rad)) / l3
    if value > 1 or value < -1:
        return None
    asin_val = math.degrees(math.asin(value))
    d3_1 = d2 - asin_val
    d3_2 = d2 - (180 - asin_val)
    max_s3 = int(-d3_2)
    min_s3 = int(d3_1)
    try:
        joint3.minDegree = min_s3
        joint3.maxDegree = max_s3
    except NameError:
        pass # joint3 not initialized yet during first boot
    print(f"Updated Stepper 3 range: min={min_s3}, max={max_s3}")
    return -d3_1, -d3_2


def solve_d2(d3_degrees):
    global max_s2, min_s2
    d3 = math.radians(d3_degrees)
    a = l2 + l3 * math.sin(d3)
    b = l3 * math.cos(d3)
    c = -l1
    R = math.sqrt(a * a + b * b)
    if abs(c) > R:
        return None
    phi = math.atan2(b, a)
    psi = math.acos(c / R)
    d2_1 = math.degrees(phi + psi)
    d2_2 = math.degrees(phi - psi)
    if -int(d2_2) > 70:
        max_s2 = 70
    else:
        max_s2 = -int(d2_2) - 5
    if -int(d2_1) < -20:
        min_s2 = -20
    else:
        min_s2 = -int(d2_1)
    try:
        joint2.minDegree = min_s2
        joint2.maxDegree = max_s2
    except NameError:
        pass
    print(f"Updated Stepper 2 range: min={min_s2}, max={max_s2}")
    return d2_1, d2_2


def create_access_point():
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap_ip = '192.168.4.1'
    ap.ifconfig((ap_ip, '255.255.255.0', ap_ip, ap_ip))
    ap.config(essid=AP_SSID, password=AP_PASSWORD)
    while not ap.active():
        time.sleep(0.1)
    print("✅ Access Point Active!")
    print(f"SSID: {AP_SSID}")
    print(f"Password: {AP_PASSWORD}")
    print(f"IP Address: {ap.ifconfig()[0]}")
    if DNS_AVAILABLE:
        dns_domains = {"*": ap_ip}
        if MicroDNSSrv.Create(dns_domains):
            print("✅ DNS Server started")
        else:
            print("⚠️ DNS Server failed")
    return ap


# Ultrasonic Sensors
us_right_trig = Pin(18, Pin.OUT)
us_right_echo = Pin(19, Pin.IN)
us_left_trig = Pin(20, Pin.OUT)
us_left_echo = Pin(21, Pin.IN)

dist_right = -1.0
dist_left = -1.0

def get_distance(trig, echo):
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    pulse_time = time_pulse_us(echo, 1, 30000)
    if pulse_time < 0:
        return -1
    return (pulse_time * 0.0343) / 2

# Emergency Button
emergency_btn = Pin(16, Pin.IN, Pin.PULL_UP)

def check_emergency():
    if emergency_btn.value() == 1:
        print("🚨 EMERGENCY HOLD TRIGGERED! All motors stopped.")
        while emergency_btn.value() == 1:
            time.sleep_ms(50)
            
        print("✅ Emergency Hold Released")
        return True
    return False

# Limit switches
limit_switch_1 = Pin(3, Pin.IN, Pin.PULL_UP)
limit_switch_2 = Pin(2, Pin.IN, Pin.PULL_UP)
limit_switch_3 = Pin(4, Pin.IN, Pin.PULL_UP)

# --- LIMIT SWITCH CALIBRATION ANGLES ---
# Edit these variables to change the exact angle (in degrees) 
# where the limit switch physically triggers.
# NOTE: If a joint's limit switch is on the negative side (e.g. minDegree),
# use a negative number here (like -20.0 instead of 20.0).
LIMIT_ANGLE_1 = 155.0
LIMIT_ANGLE_2 = -25.0
LIMIT_ANGLE_3 = 90.0

# Global limit state
limit_triggered = False
limit_trigger_time = 0
is_calibrating = False
motion_interrupted = False
calib_hit_1 = False
calib_hit_2 = False
calib_hit_3 = False

def limit1_isr(pin):
    global motion_interrupted, calib_hit_1, limit_triggered, limit_trigger_time
    if pin.value() == 0:
        print(f"🛑 Limit 1 Triggered! Actual angle before reset: {joint1.currentDegree}°")
        if is_calibrating:
            calib_hit_1 = True
        else:
            limit_triggered = True
            limit_trigger_time = time.ticks_ms()
            motion_interrupted = True
            joint1.currentDegree = LIMIT_ANGLE_1

def limit2_isr(pin):
    global motion_interrupted, calib_hit_2, limit_triggered, limit_trigger_time
    if pin.value() == 0:
        print(f"🛑 Limit 2 Triggered! Actual angle before reset: {joint2.currentDegree}°")
        if is_calibrating:
            calib_hit_2 = True
        else:
            limit_triggered = True
            limit_trigger_time = time.ticks_ms()
            motion_interrupted = True
            joint2.currentDegree = LIMIT_ANGLE_2

def limit3_isr(pin):
    global motion_interrupted, calib_hit_3, limit_triggered, limit_trigger_time
    if pin.value() == 0:
        print(f"🛑 Limit 3 Triggered! Actual angle before reset: {joint3.currentDegree}°")
        if is_calibrating:
            calib_hit_3 = True
        else:
            limit_triggered = True
            limit_trigger_time = time.ticks_ms()
            motion_interrupted = True
            joint3.currentDegree = LIMIT_ANGLE_3

def step_motor(steps, dirPin, pulPin, direction, delay_step=delay):
    """Step motor with interrupt checking. Stops immediately on trigger."""
    global limit_triggered, limit_trigger_time, motion_interrupted
    dirPin.value(0 if direction else 1)
    for _ in range(steps):
        if limit_triggered or check_emergency() or motion_interrupted:
            print("⚠️ Movement blocked — interrupt limit or hold active.")
            motion_interrupted = False
            return
        pulPin.value(1)
        time.sleep_us(delay_step)
        pulPin.value(0)
        time.sleep_us(delay_step)


def move_stepper(target_degree, joint: Joint):
    """Move a joint to target_degree. Silently blocked if limit_triggered."""
    global limit_triggered
    if limit_triggered or check_emergency():
        print("⚠️ Movement blocked — limit switch lockout or hold active.")
        return

    target_degree = max(min(target_degree, joint.maxDegree), joint.minDegree)
    degree_delta = target_degree - joint.currentDegree
    pulse_delta = int(abs(degree_delta) * joint.degreeToPulseRatio)

    if pulse_delta == 0:
        return

    direction = degree_delta > 0
    step_motor(abs(pulse_delta), joint.dirPin, joint.pulsePin, direction)

    # Only update position if we were not stopped by a limit
    if not limit_triggered:
        joint.currentDegree = target_degree
    print(f"Joint moved to: {joint.currentDegree}°")





def calibrate_steppers(joint1: Joint, joint2: Joint, joint3: Joint):
    """
    Non-blocking concurrent calibration of all 3 joints.
    Each joint goes to its limit switch, updates to max angle, then goes to 0.
    """
    global is_calibrating, calib_hit_1, calib_hit_2, calib_hit_3
    is_calibrating = True
    calib_hit_1 = False
    calib_hit_2 = False
    calib_hit_3 = False

    state1 = 0
    state2 = 0
    state3 = 0

    steps_to_zero_1 = steps_to_zero_2 = steps_to_zero_3 = 0
    steps_done_1 = steps_done_2 = steps_done_3 = 0

    last_step_time_1 = last_step_time_2 = last_step_time_3 = time.ticks_us()
    pulse_delay_1 = pulse_delay_2 = pulse_delay_3 = 533

    print("🔧 Calibrating steppers concurrently at 75% speed...")

    while not (state1 == 2 and state2 == 2 and state3 == 2):
        if check_emergency():
            print("🚨 EMERGENCY HOLD! Aborting calibration.")
            is_calibrating = False
            return
        now = time.ticks_us()

        # --- Joint 3 (First) ---
        if state3 != 2:
            if state3 == 0:
                if calib_hit_3:
                    joint3.currentDegree = LIMIT_ANGLE_3
                    joint3.jointDir(0)
                    steps_to_zero_3 = int(abs(LIMIT_ANGLE_3) * joint3.degreeToPulseRatio)
                    steps_done_3 = 0
                    state3 = 1
                    print(f"Joint 3 reversing to 0. Steps required: {steps_to_zero_3}")
                elif time.ticks_diff(now, last_step_time_3) >= pulse_delay_3:
                    joint3.jointDir(1)
                    joint3.jointPul(1)
                    time.sleep_us(delay)
                    joint3.jointPul(0)
                    last_step_time_3 = now
            elif state3 == 1:
                if steps_done_3 < steps_to_zero_3:
                    if time.ticks_diff(now, last_step_time_3) >= pulse_delay_3:
                        joint3.jointPul(1)
                        time.sleep_us(delay)
                        joint3.jointPul(0)
                        steps_done_3 += 1
                        last_step_time_3 = now
                else:
                    joint3.currentDegree = 0
                    state3 = 2
                    print("Joint 3 calibration complete.")

        # --- Joint 2 ---
        if state2 != 2:
            if state2 == 0:
                if calib_hit_2:
                    joint2.currentDegree = LIMIT_ANGLE_2
                    joint2.jointDir(1)
                    steps_to_zero_2 = int(abs(LIMIT_ANGLE_2) * joint2.degreeToPulseRatio)
                    steps_done_2 = 0
                    state2 = 1
                    print(f"Joint 2 reversing to 0. Steps required: {steps_to_zero_2}")
                elif time.ticks_diff(now, last_step_time_2) >= pulse_delay_2:
                    joint2.jointDir(0)
                    joint2.jointPul(1)
                    time.sleep_us(delay)
                    joint2.jointPul(0)
                    last_step_time_2 = now
            elif state2 == 1:
                if steps_done_2 < steps_to_zero_2:
                    if time.ticks_diff(now, last_step_time_2) >= pulse_delay_2:
                        joint2.jointPul(1)
                        time.sleep_us(delay)
                        joint2.jointPul(0)
                        steps_done_2 += 1
                        last_step_time_2 = now
                else:
                    joint2.currentDegree = 0
                    state2 = 2
                    print("Joint 2 calibration complete.")

        # --- Joint 1 ---
        if state1 != 2:
            if state1 == 0:
                if calib_hit_1:
                    joint1.currentDegree = LIMIT_ANGLE_1
                    joint1.jointDir(0)
                    steps_to_zero_1 = int(abs(LIMIT_ANGLE_1) * joint1.degreeToPulseRatio)
                    steps_done_1 = 0
                    state1 = 1
                    print(f"Joint 1 reversing to 0. Steps required: {steps_to_zero_1}")
                elif time.ticks_diff(now, last_step_time_1) >= pulse_delay_1:
                    joint1.jointDir(1)
                    joint1.jointPul(1)
                    time.sleep_us(delay)
                    joint1.jointPul(0)
                    last_step_time_1 = now
            elif state1 == 1:
                if steps_done_1 < steps_to_zero_1:
                    if time.ticks_diff(now, last_step_time_1) >= pulse_delay_1:
                        joint1.jointPul(1)
                        time.sleep_us(delay)
                        joint1.jointPul(0)
                        steps_done_1 += 1
                        last_step_time_1 = now
                else:
                    joint1.currentDegree = 0
                    state1 = 2
                    print("Joint 1 calibration complete.")

    is_calibrating = False
    print("✅ Calibration complete!")


def send_chunked_response(cl, response, content_type="text/html"):
    """Send HTTP response in small chunks without encoding full string at once."""
    try:
        cl.send(f'HTTP/1.0 200 OK\r\nContent-type: {content_type}\r\n\r\n'.encode())
        chunk_size = 256
        start = 0
        total = len(response)
        while start < total:
            end = min(start + chunk_size, total)
            cl.send(response[start:end].encode('utf-8'))
            start = end
            gc.collect()
            time.sleep_ms(5)
    except Exception as e:
        print(f"Send error: {e}")


def pick_place():
    """Execute pick and place sequence. Blocked if limit triggered."""
    global saved_movement_1, saved_movement_2
    if not saved_movement_1 or not saved_movement_2:
        print("❌ Save Position 1 and Position 2 first")
        return

    if limit_triggered:
        print("⚠️ Pick & Place blocked — limit switch lockout active.")
        return

    print("▶️ Moving to Position 1")
    move_stepper(saved_movement_1[0], joint1)
    solve_d3(saved_movement_1[1])
    move_stepper(saved_movement_1[1], joint2)
    solve_d2(saved_movement_1[2])
    move_stepper(saved_movement_1[2], joint3)
    time.sleep(1)
    gripper.open()
    time.sleep(1)
    gripper.close()
    time.sleep(1)

    if limit_triggered:
        print("⚠️ Limit triggered during Position 1 move. Aborting.")
        return

    print("▶️ Moving to Position 2")
    move_stepper(saved_movement_2[0], joint1)
    solve_d3(saved_movement_2[1])
    move_stepper(saved_movement_2[1], joint2)
    solve_d2(saved_movement_2[2])
    move_stepper(saved_movement_2[2], joint3)
    time.sleep(1)
    gripper.open()
    time.sleep(1)
    gripper.close()
    time.sleep(1)
    print("✅ Pick and Place Complete")


def pick_place_default():
    """Execute default pick and place for continuous mode."""
    global limit_triggered
    if limit_triggered:
        return

    print("▶️ Moving to Default Position 1")
    move_stepper(default_p1[0], joint1)
    solve_d3(default_p1[1])
    move_stepper(default_p1[1], joint2)
    solve_d2(default_p1[2])
    move_stepper(default_p1[2], joint3)
    time.sleep(1)
    gripper.open()
    time.sleep(1)
    gripper.close()
    time.sleep(1)

    if limit_triggered:
        return

    print("▶️ Moving to Default Position 2")
    move_stepper(default_p2[0], joint1)
    solve_d3(default_p2[1])
    move_stepper(default_p2[1], joint2)
    solve_d2(default_p2[2])
    move_stepper(default_p2[2], joint3)
    time.sleep(1)
    gripper.open()
    time.sleep(1)
    gripper.close()
    time.sleep(1)
    print("✅ Default Pick and Place Complete")


# ─────────────────────────── MOTOR COMMAND PATHS ────────────────────────────
MOTOR_PATHS = ['/stepper', '/run1', '/run2', '/pickplace', '/start_continuous']


def is_motor_path(path):
    return any(path.startswith(p) for p in MOTOR_PATHS)



# ──────────────────────────────── MAIN ──────────────────────────────────────
if __name__ == "__main__":
    ap = create_access_point()

    gripper = Gripper()

    joint1 = Joint(dir=9,  pulse=11, minDegree=-175, maxDegree=150, maxPulse=3875, degreeToPulseRatio=3875/175)
    joint2 = Joint(dir=7,  pulse=13, minDegree=-20,  maxDegree=70,  maxPulse=3875, degreeToPulseRatio=2625/90)
    joint3 = Joint(dir=14, pulse=15, minDegree=-85,  maxDegree=85,  maxPulse=3875, degreeToPulseRatio=2500/90)

    # Attach interrupt handlers to limits
    limit_switch_1.irq(trigger=Pin.IRQ_FALLING, handler=limit1_isr)
    limit_switch_2.irq(trigger=Pin.IRQ_FALLING, handler=limit2_isr)
    limit_switch_3.irq(trigger=Pin.IRQ_FALLING, handler=limit3_isr)

    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)
    s.setblocking(False)   # ← KEY: non-blocking so recovery loop runs every tick
    print(f"Server running on http://{ap.ifconfig()[0]}:80")

    while True:
        # ── Handle incoming HTTP request (non-blocking) ──────────────────────
        cl = None
        try:
            cl, addr = s.accept()
            cl.settimeout(5.0)
            request = cl.recv(1024).decode()
            path = request.split(' ')[1]
            print("Request:", path)

            if is_motor_path(path):
                global limit_triggered
                limit_triggered = False

            # ── Serve image ─────────────────────────────────────────────────
            if path.startswith('/arnobot.jpeg'):
                try:
                    with open('arnobot.jpeg', 'rb') as f:
                        f.seek(0, 2)
                        file_size = f.tell()
                        f.seek(0)
                        header = f'HTTP/1.0 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: {file_size}\r\nConnection: close\r\n\r\n'
                        cl.send(header.encode())
                        bytes_sent = 0
                        while True:
                            chunk = f.read(1024)
                            if not chunk:
                                break
                            cl.send(chunk)
                            bytes_sent += len(chunk)
                            gc.collect()
                        print(f'✓ Image served: {bytes_sent}/{file_size} bytes')
                except Exception as e:
                    print(f'Image error: {e}')
                    cl.send(b'HTTP/1.0 404 Not Found\r\n\r\n')

            # ── Stepper control ─────────────────────────────────────────────
            elif path.startswith('/stepper'):
                parts = path.split('?')[1].split('&')
                num = int(parts[0].split('=')[1])
                angle = int(parts[1].split('=')[1])
                if num == 1:
                    move_stepper(angle, joint1)
                elif num == 2:
                    solve_d3(angle)
                    move_stepper(angle, joint2)
                elif num == 3:
                    solve_d2(angle)
                    move_stepper(angle, joint3)
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: text/plain\r\n\r\nOK')

            elif path.startswith('/get_range3'):
                response = '{{"min":{},"max":{}}}'.format(min_s3, max_s3)
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n')
                cl.send(response.encode())

            elif path.startswith('/get_range2'):
                response = '{{"min":{},"max":{}}}'.format(min_s2, max_s2)
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n')
                cl.send(response.encode())

            elif path.startswith('/status'):
                response = '{{"s1":{}, "s2":{}, "s3":{}, "dist_right":{:.1f}, "dist_left":{:.1f}, "emergency":{}}}'.format(
                    joint1.currentDegree, joint2.currentDegree, joint3.currentDegree, dist_right, dist_left, "true" if emergency_btn.value() == 1 else "false")
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n')
                cl.send(response.encode())

            elif path.startswith('/arnobot.png'):
                try:
                    with open('arnobot.png', 'rb') as f:
                        f.seek(0, 2)
                        file_size = f.tell()
                        f.seek(0)
                        header = f'HTTP/1.0 200 OK\r\nContent-Type: image/png\r\nContent-Length: {file_size}\r\nConnection: close\r\n\r\n'
                        cl.send(header.encode())
                        bytes_sent = 0
                        while True:
                            chunk = f.read(1024)
                            if not chunk:
                                break
                            cl.send(chunk)
                            bytes_sent += len(chunk)
                            gc.collect()
                        print(f'✓ Logo served: {bytes_sent}/{file_size} bytes')
                except Exception as e:
                    print(f'Logo error: {e}')
                    cl.send(b'HTTP/1.0 404 Not Found\r\n\r\n')

            elif path.startswith('/gripper'):
                action = path.split('=')[1]
                if action == 'open':
                    gripper.open()
                    gripper_state = 'open'
                elif action == 'close':
                    gripper.close()
                    gripper_state = 'closed'
                cl.send(b'HTTP/1.0 200 OK\r\n\r\nOK')

            elif path.startswith('/calibrate'):
                calibrate_steppers(joint1, joint2, joint3)
                cl.send(b'HTTP/1.0 200 OK\r\n\r\nOK')

            elif path.startswith('/save_movement1'):
                saved_movement_1.clear()
                saved_movement_1.extend([joint1.currentDegree, joint2.currentDegree, joint3.currentDegree])
                print("✓ Position 1 saved:", saved_movement_1)
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: text/plain\r\n\r\nPosition 1 Saved')

            elif path.startswith('/save_movement2'):
                saved_movement_2.clear()
                saved_movement_2.extend([joint1.currentDegree, joint2.currentDegree, joint3.currentDegree])
                print("✓ Position 2 saved:", saved_movement_2)
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: text/plain\r\n\r\nPosition 2 Saved')

            elif path.startswith('/run1'):
                if saved_movement_1 and not limit_triggered:
                    print("Running to Position 1...")
                    move_stepper(0, joint1)
                    move_stepper(0, joint2)
                    move_stepper(0, joint3)
                    time.sleep(1)
                    move_stepper(saved_movement_1[0], joint1)
                    solve_d3(saved_movement_1[1])
                    move_stepper(saved_movement_1[1], joint2)
                    solve_d2(saved_movement_1[2])
                    move_stepper(saved_movement_1[2], joint3)
                    print("✅ Position 1 reached")
                cl.send(b'HTTP/1.0 200 OK\r\n\r\nOK')

            elif path.startswith('/run2'):
                if saved_movement_2 and not limit_triggered:
                    print("Running to Position 2...")
                    move_stepper(0, joint1)
                    move_stepper(0, joint2)
                    move_stepper(0, joint3)
                    time.sleep(1)
                    move_stepper(saved_movement_2[0], joint1)
                    solve_d3(saved_movement_2[1])
                    move_stepper(saved_movement_2[1], joint2)
                    solve_d2(saved_movement_2[2])
                    move_stepper(saved_movement_2[2], joint3)
                    print("✅ Position 2 reached")
                cl.send(b'HTTP/1.0 200 OK\r\n\r\nOK')

            elif path.startswith('/pickplace'):
                pick_place()
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: text/plain\r\n\r\nPick Place Done')

            elif path.startswith('/start_continuous'):
                continuous_run = True
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: text/plain\r\n\r\nContinuous Run Started')

            elif path.startswith('/stop_continuous'):
                continuous_run = False
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: text/plain\r\n\r\nContinuous Run Stopped')

            elif path.startswith('/favicon.ico'):
                cl.send(b'HTTP/1.0 404 Not Found\r\n\r\n')

            else:
                # Main page
                cl.send(b'HTTP/1.0 404 Not Found\\r\\n\\r\\nAPI Only')

        except OSError as e:
            # errno 11 = EAGAIN: no connection available on non-blocking socket — totally normal
            if e.args[0] != 11:
                print("Socket error:", e)
        except Exception as e:
            print("Request handling error:", e)
        finally:
            if cl:
                try:
                    cl.close()
                except:
                    pass

        # ── Continuous run (only when not in limit lockout) ──────────────────
        if continuous_run and not limit_triggered and emergency_btn.value() == 0:
            pick_place_default()
            time.sleep(2)
            
        # ── Ultrasonic Distance Measurement ──────────────────────────────────────
        dist_right = get_distance(us_right_trig, us_right_echo)
        dist_left = get_distance(us_left_trig, us_left_echo)

        # ── Emergency blocking loop check ───────────────────────────────────
        check_emergency()

        gc.collect()
        time.sleep_ms(5)  # Yield — keeps loop ~200Hz without burning CPU

