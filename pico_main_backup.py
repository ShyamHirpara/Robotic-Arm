import network
import socket
import time
import math
import micropython
import gc
import utime
from machine import Pin, PWM, time_pulse_us
from stepper import Stepper

# ─────────────────────────── INSTALL NOTE ────────────────────────────────────
# Install micropython-stepper on your Pico W before running this file.
# In the Thonny REPL run:
#   import mip
#   mip.install("github:redoxcode/micropython-stepper")
# This installs lib/stepper/__init__.py and lib/stepper/__main__.py
# ─────────────────────────────────────────────────────────────────────────────

# Import DNS Server for captive portal
try:
    from microDNSSrv import MicroDNSSrv
    DNS_AVAILABLE = True
except:
    DNS_AVAILABLE = False
    print("⚠️ DNS server not available")

# Access Point credentials
AP_SSID     = 'RoboticArm_AP'
AP_PASSWORD = '12345678'


# ─────────────────────────── GRIPPER ─────────────────────────────────────────
class Gripper:
    def __init__(self):
        self.servo = PWM(Pin(26))
        self.servo.freq(50)
        self.MIN_DUTY = 1638
        self.MAX_DUTY = 8192

    def set_angle(self, angle):
        duty = int((angle / 180) * (self.MAX_DUTY - self.MIN_DUTY) + self.MIN_DUTY)
        self.servo.duty_u16(duty)

    def open(self):
        self.set_angle(0)
        utime.sleep_ms(300)

    def close(self):
        self.set_angle(85)
        utime.sleep_ms(300)


# ─────────────────────────── JOINT ───────────────────────────────────────────
class Joint:
    """
    Wraps micropython-stepper Stepper with degree-based limits.

    steps_per_rev = degreeToPulseRatio × 360
      J1: (3875/175) × 360 ≈ 7971
      J2: (2625/90)  × 360 ≈ 10500
      J3: (2500/90)  × 360 ≈ 10000

    Hardware joint ranges:
      J1: -175° to +150°
      J2:  -20° to  +70°
      J3:  -75° to  +85°  (min updated from -85° after mechanical testing)

    invert_dir=True because the hardware wires positive direction to dirPin LOW (0),
    while the Stepper library defaults to dirPin HIGH for positive steps.
    Adjust per joint if direction is wrong after testing.

    jog_sps  — steps/sec during jogging  (≈ 27°/s for J1, 18.9°/s for J2, 40°/s for J3)
    calib_sps — steps/sec during calibration (same as jog_sps by default)
    timer_id — always use -1 on Pico W (RP2040 only supports software timers via Timer(-1))
    """
    def __init__(self, step_pin, dir_pin, min_deg, max_deg,
                 steps_per_rev, jog_sps=1000, calib_sps=750,
                 timer_id=-1, invert_dir=True):
        self.stepper        = Stepper(step_pin, dir_pin,
                                      steps_per_rev=steps_per_rev,
                                      speed_sps=jog_sps,
                                      invert_dir=invert_dir,
                                      timer_id=timer_id)
        self.minDegree      = min_deg
        self.maxDegree      = max_deg
        self._steps_per_rev = steps_per_rev
        self._jog_sps       = jog_sps
        self._calib_sps     = calib_sps

    @property
    def currentDegree(self):
        """Read live position from the stepper's internal step counter."""
        return self.stepper._pos * 360.0 / self._steps_per_rev

    @currentDegree.setter
    def currentDegree(self, val):
        """
        Force-set position (e.g. after limit switch hit or calibration).
        Calls overwrite_pos() which syncs both pos and target_pos in the library.
        """
        self.stepper.overwrite_pos(round(val * self._steps_per_rev / 360.0))


# ─────────────────────────── CONSTANTS ───────────────────────────────────────
# Angles set after each joint hits its limit switch during calibration.
LIMIT_ANGLE_1 =  155.0   # J1 positive limit
LIMIT_ANGLE_2 =  -25.0   # J2 negative limit
LIMIT_ANGLE_3 =   86.0   # J3 positive limit

# ─────────────────────────── GLOBAL STATE ────────────────────────────────────
limit_triggered = False
is_calibrating  = False
calib_hit_1     = False
calib_hit_2     = False
calib_hit_3     = False

# Populated after joint init — used by stop_all_motors() / check_emergency()
all_joints = []

gripper_state    = 'closed'

dist_right = -1.0
dist_left  = -1.0

# Dynamic kinematic limits (updated by solve_d2 / solve_d3)
min_s2 = -20
max_s2 =  70
min_s3 = -75   # matches J3_ABS_MIN
max_s3 =  85

# ─────────────────────────── JOG CONTROLLER STATE ───────────────────────────
jog_axis = 0   # index of last jogged joint (1/2/3); used for post-stop kinematics

# ─────────────────────────── KINEMATICS ──────────────────────────────────────
l1 = 24
l2 = 21
l3 = 35

# Absolute (hardware) joint limits — used to clamp kinematic solutions
J2_ABS_MIN, J2_ABS_MAX = -20,  90
J3_ABS_MIN, J3_ABS_MAX = -75,  85   # J3 min updated from -85° after mechanical testing


def solve_d3(d2):
    """Update joint3's dynamic limits based on current joint2 angle."""
    global min_s3, max_s3
    d2_rad = math.radians(d2)
    value  = (l1 + l2 * math.cos(d2_rad)) / l3
    if value > 1 or value < -1:      # asin domain check
        # No kinematic constraint at this configuration — restore full hardware range
        min_s3, max_s3 = J3_ABS_MIN, J3_ABS_MAX
        try: joint3.minDegree = J3_ABS_MIN; joint3.maxDegree = J3_ABS_MAX
        except NameError: pass
        return
    asin_val = math.degrees(math.asin(value))
    d3_a = d2 - asin_val              # solution A
    d3_b = d2 - (180.0 - asin_val)   # solution B
    # Clamp to hardware absolute limits
    min_s3 = max(round(d3_a),  J3_ABS_MIN)
    max_s3 = min(round(-d3_b), J3_ABS_MAX)
    try:
        joint3.minDegree = min_s3
        joint3.maxDegree = max_s3
    except NameError:
        pass


def solve_d2(d3_degrees):
    """Update joint2's dynamic limits based on current joint3 angle."""
    global min_s2, max_s2
    d3 = math.radians(d3_degrees)
    a  = l2 + l3 * math.sin(d3)
    b  = l3 * math.cos(d3)
    c  = -l1
    R  = math.sqrt(a*a + b*b)
    if abs(c) > R:                   # acos domain check
        # No kinematic constraint at this configuration — restore full hardware range
        min_s2, max_s2 = J2_ABS_MIN, J2_ABS_MAX
        try: joint2.minDegree = J2_ABS_MIN; joint2.maxDegree = J2_ABS_MAX
        except NameError: pass
        return
    phi  = math.atan2(b, a)
    psi  = math.acos(c / R)
    d2_a = math.degrees(phi + psi)   # solution A
    d2_b = math.degrees(phi - psi)   # solution B
    # Clamp to hardware absolute limits
    min_s2 = max(round(-d2_a), J2_ABS_MIN)
    max_s2 = min(round(-d2_b), J2_ABS_MAX)
    try:
        joint2.minDegree = min_s2
        joint2.maxDegree = max_s2
    except NameError:
        pass


# ─────────────────────────── ACCESS POINT ────────────────────────────────────
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
        if MicroDNSSrv.Create({"*": ap_ip}):
            print("✅ DNS Server started")
        else:
            print("⚠️ DNS Server failed")
    return ap


# ─────────────────────────── ULTRASONIC ──────────────────────────────────────
us_right_trig = Pin(18, Pin.OUT)
us_right_echo = Pin(19, Pin.IN)
us_left_trig  = Pin(20, Pin.OUT)
us_left_echo  = Pin(21, Pin.IN)


def get_distance(trig, echo):
    trig.value(0); time.sleep_us(2)
    trig.value(1); time.sleep_us(10)
    trig.value(0)
    pulse_time = time_pulse_us(echo, 1, 30000)
    return -1 if pulse_time < 0 else (pulse_time * 0.0343) / 2


# ─────────────────────────── EMERGENCY ───────────────────────────────────────
emergency_btn = Pin(16, Pin.IN, Pin.PULL_UP)


def stop_all_motors():
    """Stop all stepper timers immediately. Safe to call from any context."""
    for j in all_joints:
        j.stepper.stop()


def check_emergency():
    if emergency_btn.value() == 1:
        time.sleep_ms(20)                # debounce — ignore motor EMF glitches
        if emergency_btn.value() == 1:   # still pressed — genuine emergency
            stop_all_motors()            # ← IMMEDIATE stop BEFORE blocking wait
            print("\U0001f6a8 EMERGENCY HOLD TRIGGERED! All motors stopped.")
            while emergency_btn.value() == 1:
                time.sleep_ms(50)
            print("\u2705 Emergency Hold Released")
            return True
    return False


# ─────────────────────────── JOG CONTROLLER ─────────────────────────────────
def jog_motion(j, d, tcp_sock):
    """
    Blocking jog loop.

    Starts j.stepper.free_run() then loops until any stopping condition:
      • limit_triggered  (set by limit-switch ISR — fires even inside a while loop)
      • emergency button pressed
      • soft degree limit (maxDegree / minDegree) reached
      • "STOP" received on the non-blocking TCP socket (button released)
      • TCP connection closed

    The stepper library’s own Timer(-1) drives the step pulses, so motion
    is perfectly smooth regardless of the while-loop’s iteration rate (2 ms).

    After the motor stops, kinematic coupling limits are updated once.
    """
    global jog_axis, limit_triggered

    # Pre-check: don’t start if already at the requested boundary
    if d == 1 and j.currentDegree >= j.maxDegree:
        return
    if d == 0 and j.currentDegree <= j.minDegree:
        return

    jog_axis = all_joints.index(j) + 1  # track for post-stop kinematics

    # ── Start continuous motion ────────────────────────────────────────────────
    if not (limit_triggered or emergency_btn.value() == 1) : 
        j.stepper.speed(j._jog_sps)
        j.stepper.free_run(1 if d else -1)

    last_send = time.ticks_ms()

    # ── Guard loop ──────────────────────────────────────────────────────────
    while True:

        # Stopping condition 1: limit switch ISR
        # Stopping condition 2: emergency button
        # Stopping condition 3: soft degree limits
        if limit_triggered or emergency_btn.value() == 1 or (d == 1 and j.currentDegree >= j.maxDegree) or (d == 0 and j.currentDegree <= j.minDegree):
            j.stepper.stop()
            break

        # Stopping condition 4: "STOP" on non-blocking TCP socket (button released)
        try:
            data = tcp_sock.recv(64)
            if data:
                if b"STOP" in data:
                    j.stepper.stop()
                    break
            else:
                # Connection closed by client
                j.stepper.stop()
                break
        except OSError as e:
            if e.args[0] != 11:   # 11 = EAGAIN (no data yet) — normal, keep looping
                j.stepper.stop()
                break

        # Send position + range state to frontend every 100 ms while jogging
        now = time.ticks_ms()
        if time.ticks_diff(now, last_send) > 100:
            last_send = now
            try:
                state = '{{"s1":{:.1f},"s2":{:.1f},"s3":{:.1f},"n2":{},"x2":{},"n3":{},"x3":{}}}\n'.format(
                    joint1.currentDegree, joint2.currentDegree, joint3.currentDegree,
                    min_s2, max_s2, min_s3, max_s3)
                tcp_sock.send(state.encode())
            except OSError:
                break

        #time.sleep_ms(1)   # 1ms poll — keeps loop tight, avoids busy-spinning

    # ── Post-stop: one-shot kinematic coupling update ─────────────────────────
    if jog_axis == 2: solve_d3(j.currentDegree)
    elif jog_axis == 3: solve_d2(j.currentDegree)


limit_switch_1 = Pin(3, Pin.IN, Pin.PULL_UP)
limit_switch_2 = Pin(2, Pin.IN, Pin.PULL_UP)
limit_switch_3 = Pin(4, Pin.IN, Pin.PULL_UP)


def limit1_isr(pin):
    global calib_hit_1, limit_triggered
    if pin.value() == 0:
        if is_calibrating:
            # Guard: only register once — prevents bounce spam
            if not calib_hit_1:
                calib_hit_1 = True
        else:
            limit_triggered = True
            # Stop is deferred to main loop to avoid ISR heap allocation


def limit2_isr(pin):
    global calib_hit_2, limit_triggered
    if pin.value() == 0:
        if is_calibrating:
            if not calib_hit_2:
                calib_hit_2 = True
        else:
            limit_triggered = True


def limit3_isr(pin):
    global calib_hit_3, limit_triggered
    if pin.value() == 0:
        if is_calibrating:
            if not calib_hit_3:
                calib_hit_3 = True
        else:
            limit_triggered = True


# ─────────────────────────── MOVE HELPER ─────────────────────────────────────
def  move_stepper(target_deg, joint: Joint):
    """
    Non-blocking target move via library, polled here to appear blocking.
    The Stepper timer runs underneath — HTTP/TCP sockets remain available
    (though this function does yield every 5 ms).
    """
    global limit_triggered
    if limit_triggered or check_emergency():
        print("⚠️ Movement blocked — limit or emergency active.")
        return

    target_deg = max(min(target_deg, joint.maxDegree), joint.minDegree)
    joint.stepper.speed(joint._jog_sps)
    joint.stepper.target_deg(target_deg)

    # Poll until stepper reaches target (_pos == _target means done)
    while joint.stepper._pos != joint.stepper._target:
        if limit_triggered or check_emergency():
            joint.stepper.stop()
            print(f"⚠️ Move interrupted at {joint.currentDegree:.1f}°")
            return
        time.sleep_ms(1)

    print(f"Joint moved to: {joint.currentDegree:.1f}°")


# ─────────────────────────── CALIBRATION ─────────────────────────────────────
def _calib_abort(j1, j2, j3, reason=""):
    """Stop all motors and re-enable limit IRQs after calibration abort."""
    global is_calibrating
    j1.stepper.stop(); j2.stepper.stop(); j3.stepper.stop()
    is_calibrating = False
    limit_switch_1.irq(trigger=Pin.IRQ_FALLING, handler=limit1_isr)
    limit_switch_2.irq(trigger=Pin.IRQ_FALLING, handler=limit2_isr)
    limit_switch_3.irq(trigger=Pin.IRQ_FALLING, handler=limit3_isr)
    if reason:
        print(reason)


def calibrate_steppers(j1: Joint, j2: Joint, j3: Joint):
    """
    Per-joint independent calibration state machine.

    Each joint cycles through three states independently:
      0 = SEEKING  — free_run toward limit switch (polls pin every 2 ms)
      1 = RETURNING — limit hit: reversed immediately via target_deg(0)
      2 = DONE     — reached 0°, waiting for others

    A joint does NOT wait for the others before reversing — it starts
    heading back to 0° the instant its own limit pin goes LOW.
    Calibration is complete when all three reach state 2.

    Direct pin polling is used (not IRQ_FALLING) so that an already-pressed
    limit switch is detected immediately on the first poll.
    """
    global is_calibrating
    is_calibrating = True

    # Disable limit IRQs — we poll directly for reliability
    limit_switch_1.irq(handler=None)
    limit_switch_2.irq(handler=None)
    limit_switch_3.irq(handler=None)

    # Per-joint state: 0=seeking, 1=returning to 0, 2=done
    SEEKING, RETURNING, DONE = 0, 1, 2
    state = [SEEKING, SEEKING, SEEKING]

    joints   = [j1,              j2,              j3             ]
    switches = [limit_switch_1,  limit_switch_2,  limit_switch_3 ]
    angles   = [LIMIT_ANGLE_1,   LIMIT_ANGLE_2,   LIMIT_ANGLE_3  ]
    dirs     = [1,               -1,              1              ]  # free_run direction
    names    = ["J1",            "J2",            "J3"           ]

    print("🔧 Calibrating steppers (independent homing)...")
    print(f"  Limit pins: L1={limit_switch_1.value()} L2={limit_switch_2.value()} L3={limit_switch_3.value()}")

    # Start all three seeking simultaneously
    for j, d in zip(joints, dirs):
        j.stepper.speed(j._calib_sps)
        j.stepper.free_run(d)

    while any(s != DONE for s in state):
        if check_emergency():
            _calib_abort(j1, j2, j3, "🚨 Emergency! Aborting calibration.")
            return

        for i in range(3):
            if state[i] == SEEKING:
                if switches[i].value() == 0:          # limit pin LOW → hit
                    joints[i].stepper.stop()
                    joints[i].currentDegree = angles[i]  # sync position
                    state[i] = RETURNING
                    print(f"✓ {names[i]} limit hit @ {angles[i]}°  → reversing to 0°")
                    joints[i].stepper.speed(joints[i]._calib_sps)
                    joints[i].stepper.target_deg(0)        # reverse immediately

            elif state[i] == RETURNING:
                if joints[i].stepper._pos == joints[i].stepper._target:
                    state[i] = DONE
                    print(f"✓ {names[i]} reached 0°")

        time.sleep_ms(1)

    is_calibrating = False
    limit_switch_1.irq(trigger=Pin.IRQ_FALLING, handler=limit1_isr)
    limit_switch_2.irq(trigger=Pin.IRQ_FALLING, handler=limit2_isr)
    limit_switch_3.irq(trigger=Pin.IRQ_FALLING, handler=limit3_isr)
    # All joints are now at 0°. Reset dynamic kinematic limits to hardware absolutes.
    # solve_d3(0) fails domain check (value > 1), so we must reset explicitly here.
    global min_s2, max_s2, min_s3, max_s3
    min_s2, max_s2 = J2_ABS_MIN, J2_ABS_MAX
    min_s3, max_s3 = J3_ABS_MIN, J3_ABS_MAX
    j2.minDegree = J2_ABS_MIN;  j2.maxDegree = J2_ABS_MAX
    j3.minDegree = J3_ABS_MIN;  j3.maxDegree = J3_ABS_MAX
    print("✅ Calibration complete! All joints at 0°.")



# ─────────────────────────── GOTO POSITION (New PnP Command) ────────────────
def goto_position(order, axis1_deg, axis2_deg, axis3_deg, gripper_action, tcp_sock=None):
    """
    Move all three axes CONCURRENTLY to the requested angles, then apply the
    gripper, then respond to the backend via TCP.

    Steps:
      1. Check delta between current and requested angle for each axis.
      2. Call target_deg() on every axis whose delta > 0.5 deg (non-blocking).
         All three timers fire simultaneously — true concurrent movement.
      3. Poll a single loop until all moving axes reach their targets.
         Emergency / limit switch aborts the move at any point.
      4. Execute gripper open/close.
      5. Send /position_order=N/status=complete (or error) via TCP socket.

    Response format:
      /position_order=<order>/status=complete
      /position_order=<order>/status=error:<reason>
    """
    global limit_triggered, gripper_state

    def send_response(status):
        resp = '/position_order={}/status={}\n'.format(order, status)
        print('PnP response:', resp.strip())
        if tcp_sock:
            try:
                tcp_sock.send(resp.encode())
            except OSError as e:
                print('TCP send error:', e)

    # ── Pre-flight checks ────────────────────────────────────────────────────
    if limit_triggered:
        send_response('error:limit_triggered')
        return
    if check_emergency():
        send_response('error:emergency')
        return

    try:
        # ── Step 1: Clamp targets to joint limits ────────────────────────────
        t1 = max(min(axis1_deg, joint1.maxDegree), joint1.minDegree)
        t2 = max(min(axis2_deg, joint2.maxDegree), joint2.minDegree)
        t3 = max(min(axis3_deg, joint3.maxDegree), joint3.minDegree)

        # Update kinematic coupling limits before moving
        solve_d3(t2)
        solve_d2(t3)

        # ── Step 2: Start concurrent movement on axes that need to move ──────
        moving = []   # joints that are actively moving

        if abs(t1 - joint1.currentDegree) > 0.5:
            joint1.stepper.speed(joint1._jog_sps)
            joint1.stepper.target_deg(t1)
            moving.append(joint1)
            print('J1 -> {:.1f}deg'.format(t1))

        if abs(t2 - joint2.currentDegree) > 0.5:
            joint2.stepper.speed(joint2._jog_sps)
            joint2.stepper.target_deg(t2)
            moving.append(joint2)
            print('J2 -> {:.1f}deg'.format(t2))

        if abs(t3 - joint3.currentDegree) > 0.5:
            joint3.stepper.speed(joint3._jog_sps)
            joint3.stepper.target_deg(t3)
            moving.append(joint3)
            print('J3 -> {:.1f}deg'.format(t3))

        # ── Step 3: Poll until ALL axes reach their targets ──────────────────
        while any(j.stepper._pos != j.stepper._target for j in moving):
            if limit_triggered:
                for j in moving: j.stepper.stop()
                send_response('error:limit_triggered')
                return
            if check_emergency():
                for j in moving: j.stepper.stop()
                send_response('error:emergency')
                return
            time.sleep_ms(2)

        print('All axes at target for position {}'.format(order))

        # ── Step 4: Gripper ──────────────────────────────────────────────────
        time.sleep_ms(200)   # brief settle before gripper moves
        if gripper_action == 'open':
            gripper.open()
            gripper_state = 'open'
        else:
            gripper.close()
            gripper_state = 'closed'

        send_response('complete')
        print('Position {} complete'.format(order))

    except Exception as e:
        send_response('error:{}'.format(str(e)))




# ──────────────────────────────── MAIN ───────────────────────────────────────
if __name__ == "__main__":
    ap = create_access_point()

    gripper = Gripper()

    # ── Joint initialization ─────────────────────────────────────────────────
    # steps_per_rev = degreeToPulseRatio × 360
    # invert_dir=True: our hardware uses dirPin LOW for positive direction,
    #   which is opposite to the library's default (dirPin HIGH = positive).
    # Adjust invert_dir per joint if motor runs the wrong way after testing.

    joint1 = Joint(step_pin=11, dir_pin=9,
                   min_deg=-170, max_deg=150,
                   steps_per_rev=7971,          # (3875/175) × 360
                   jog_sps=600,                # ≈ 27°/s
                   calib_sps=1000,               # ≈ 45°/s
                   timer_id=-1, invert_dir=True)

    joint2 = Joint(step_pin=13, dir_pin=7,
                   min_deg=-20, max_deg=70,
                   steps_per_rev=10500,          # (2625/90) × 360
                   jog_sps=550,                  # ≈ 18.9°/s  (reduced from 800 for torque)
                   calib_sps=550,                # ≈ 18.9°/s
                   timer_id=-1, invert_dir=True)

    joint3 = Joint(step_pin=15, dir_pin=14,
                   min_deg=-75, max_deg=85,      # min updated from -85° after mechanical testing
                   steps_per_rev=10000,          # (2500/90) × 360
                   jog_sps=1111,                 # ≈ 40°/s
                   calib_sps=1111,               # ≈ 40°/s
                   timer_id=-1, invert_dir=True)

    # ── Attach limit switch interrupts ───────────────────────────────────────
    limit_switch_1.irq(trigger=Pin.IRQ_FALLING, handler=limit1_isr)
    limit_switch_2.irq(trigger=Pin.IRQ_FALLING, handler=limit2_isr)
    limit_switch_3.irq(trigger=Pin.IRQ_FALLING, handler=limit3_isr)

    # ── Populate motor list so check_emergency() / stop_all_motors() work ────
    all_joints.extend([joint1, joint2, joint3])


    # ── HTTP server (port 80) ────────────────────────────────────────────────
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr); s.listen(1); s.setblocking(False)
    print(f"Server running on http://{ap.ifconfig()[0]}:80")

    # ── TCP Bridge (port 81) for real-time jog control ───────────────────────
    addr2 = socket.getaddrinfo('0.0.0.0', 81)[0][-1]
    s2 = socket.socket()
    s2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s2.bind(addr2); s2.listen(1); s2.setblocking(False)
    print("TCP Bridge listening on port 81")

    # ── Jog TCP mailbox state ─────────────────────────────────────────────────
    tcp_client    = None
    last_tcp_send = 0

    # ─────────────────────────── MAIN LOOP ───────────────────────────────────
    while True:
        # ── Handle incoming HTTP request (non-blocking) ───────────────────────
        cl = None
        try:
            cl, addr = s.accept()
            cl.settimeout(5.0)
            request = cl.recv(1024).decode()
            path = request.split(' ')[1]
            print("Request:", path)

            # ── Stepper control ──────────────────────────────────────────────
            if path.startswith('/stepper'):
                limit_triggered = False   # reset before any motor movement
                parts = path.split('?')[1].split('&')
                num   = int(parts[0].split('=')[1])
                angle = float(parts[1].split('=')[1])
                if num == 1:
                    move_stepper(angle, joint1)
                elif num == 2:
                    solve_d3(angle); move_stepper(angle, joint2)
                elif num == 3:
                    solve_d2(angle); move_stepper(angle, joint3)
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: text/plain\r\n\r\nOK')

            elif path.startswith('/status'):
                # Diagnostic endpoint — returns current joint angles, distances, emergency state
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n')
                cl.send('{{"s1":{:.1f},"s2":{:.1f},"s3":{:.1f},"dist_right":{:.1f},"dist_left":{:.1f},"emergency":{}}}'.format(
                    joint1.currentDegree, joint2.currentDegree, joint3.currentDegree,
                    dist_right, dist_left,
                    "true" if emergency_btn.value() == 1 else "false").encode())

            elif path.startswith('/current_position'):
                cl.send(b'HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n')
                cl.send('{{"joint1":{:.1f},"joint2":{:.1f},"joint3":{:.1f}}}'.format(
                    joint1.currentDegree, joint2.currentDegree, joint3.currentDegree).encode())

            elif path.startswith('/gripper'):
                action = path.split('=')[1]
                if action == 'open':
                    gripper.open(); gripper_state = 'open'
                elif action == 'close':
                    gripper.close(); gripper_state = 'closed'
                cl.send(b'HTTP/1.0 200 OK\r\n\r\nOK')

            elif path.startswith('/calibrate'):
                calibrate_steppers(joint1, joint2, joint3)
                cl.send(b'HTTP/1.0 200 OK\r\n\r\nOK')

            elif path.startswith('/favicon.ico'):
                cl.send(b'HTTP/1.0 404 Not Found\r\n\r\n')

            else:
                cl.send(b'HTTP/1.0 404 Not Found\r\n\r\nAPI Only')

        except OSError as e:
            if e.args[0] != 11:
                print("Socket error:", e)
        except Exception as e:
            print("Request handling error:", e)
        finally:
            if cl:
                try: cl.close()
                except: pass

        # ── TCP Bridge for Real-time Jog Control (Port 81) ───────────────────
        try:
            cl2, addr2 = s2.accept()
            cl2.setblocking(False)
            if tcp_client:
                try: tcp_client.close()
                except: pass
            tcp_client = cl2
        except OSError as e:
            if e.args[0] != 11: print("TCP accept error:", e)

        if tcp_client:
            try:
                data = tcp_client.recv(256)   # 256 bytes — fits full /position_order command
                if data:
                    for cmd in data.decode().strip().split('\n'):
                        cmd = cmd.strip()
                        if cmd.startswith("START_"):
                            parts = cmd.split("_")
                            if len(parts) == 3:
                                try:
                                    ax = int(parts[1])
                                    d  = int(parts[2])
                                    if 1 <= ax <= 3:
                                        limit_triggered = False
                                        # jog_motion() blocks until motor stops
                                        jog_motion(all_joints[ax - 1], d, tcp_client)
                                except ValueError:
                                    pass
                        # ── Jog speed command: SPEED_<axis>_<steps_per_sec> ──
                        # Lets the PC scan fast and centre slowly (smooth
                        # visual servo).  Applied at the NEXT START_/move.
                        elif cmd.startswith("SPEED_"):
                            parts = cmd.split("_")
                            if len(parts) == 3:
                                try:
                                    ax  = int(parts[1])
                                    sps = int(parts[2])
                                    if 1 <= ax <= 3 and 50 <= sps <= 2000:
                                        all_joints[ax - 1]._jog_sps = sps
                                        print("Jog speed J{} = {} sps".format(ax, sps))
                                        try:
                                            tcp_client.send(b"SPEED_OK\n")
                                        except OSError:
                                            pass
                                except ValueError:
                                    pass
                        # ── Pick & Place position command ──────────────────
                        elif cmd.startswith('/position_order='):
                            try:
                                params = {}
                                for seg in cmd.split('/'):
                                    if '=' in seg:
                                        k, v = seg.split('=', 1)
                                        params[k] = v
                                order = int(params.get('position_order', 0))
                                ax1   = float(params.get('axis_1', 0))
                                ax2   = float(params.get('axis_2', 0))
                                ax3   = float(params.get('axis_3', 0))
                                grip  = params.get('gripper', 'close')
                                limit_triggered = False
                                # Blocking call — sends response on tcp_client when done
                                goto_position(order, ax1, ax2, ax3, grip, tcp_client)
                            except Exception as e:
                                err = '/position_order=0/status=error:{}\n'.format(str(e))
                                try: tcp_client.send(err.encode())
                                except: pass
                        # STOP is handled inside jog_motion() via tcp_sock.recv()
                else:
                    tcp_client.close(); tcp_client = None
            except OSError as e:
                if e.args[0] != 11:
                    tcp_client.close(); tcp_client = None

        # ── Ultrasonic ────────────────────────────────────────────────────────
        dist_right = get_distance(us_right_trig, us_right_echo)
        dist_left  = get_distance(us_left_trig,  us_left_echo)

        # ── Send position + range state to TCP client (every 100 ms) ──────────────
        now = time.ticks_ms()
        if tcp_client and time.ticks_diff(now, last_tcp_send) > 100:
            last_tcp_send = now
            state_str = '{{"s1":{:.1f},"s2":{:.1f},"s3":{:.1f},"n2":{},"x2":{},"n3":{},"x3":{},"gripper_state":"{}","dr":{:.1f},"dl":{:.1f}}}\n'.format(
                joint1.currentDegree, joint2.currentDegree, joint3.currentDegree,
                min_s2, max_s2, min_s3, max_s3, gripper_state, dist_right, dist_left)
            try:
                tcp_client.send(state_str.encode())
            except OSError:
                tcp_client.close(); tcp_client = None

        # ── Emergency check ───────────────────────────────────────────────────
        check_emergency()

        gc.collect()
        # No idle sleep — all sockets are non-blocking so loop is naturally fast.
        # A sleep here adds direct latency to every HTTP request cycle.
