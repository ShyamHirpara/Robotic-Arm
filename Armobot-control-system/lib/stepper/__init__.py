import machine
import math
import time


class Stepper:
    """
    micropython-stepper by redoxcode
    https://github.com/redoxcode/micropython-stepper
    MIT License

    Non-blocking stepper motor driver using machine.Timer.
    One timer per stepper instance fires at the configured speed (steps/sec)
    and pulses the step pin, tracking _pos and stopping when _pos == _target.
    """

    def __init__(self, step_pin, dir_pin, en_pin=None,
                 steps_per_rev=200, speed_sps=10,
                 invert_dir=False, invert_enable=False,
                 timer_id=-1):

        if not isinstance(step_pin, machine.Pin):
            step_pin = machine.Pin(step_pin, machine.Pin.OUT)
        if not isinstance(dir_pin, machine.Pin):
            dir_pin = machine.Pin(dir_pin, machine.Pin.OUT)
        if (en_pin is not None) and (not isinstance(en_pin, machine.Pin)):
            en_pin = machine.Pin(en_pin, machine.Pin.OUT)

        self.step_value_func  = step_pin.value
        self.dir_value_func   = dir_pin.value
        self.en_pin           = en_pin
        self.invert_dir       = invert_dir
        self.invert_enable    = invert_enable

        self.timer            = machine.Timer(timer_id)
        self.timer_is_running = False
        self.free_run_mode    = 0   # +1 = pos, -1 = neg, 0 = off

        self._steps_per_rev   = steps_per_rev
        self._speed_sps       = speed_sps
        self._pos             = 0   # current position in steps
        self._target          = 0   # target position in steps

        self._step_high       = False  # tracks step pin toggle state

        if en_pin is not None:
            self.enable(True)

    # ── Internal timer callback ───────────────────────────────────────────────
    def _step_callback(self, t):
        """Called by hardware timer at speed_sps frequency. Pulses step pin."""
        if self.free_run_mode != 0:
            # Continuous run — no target check
            self._set_dir(self.free_run_mode > 0)
            self._do_step()
            if self.free_run_mode > 0:
                self._pos += 1
            else:
                self._pos -= 1
        else:
            # Target move
            if self._pos == self._target:
                self._stop_timer()
                return
            going_pos = self._target > self._pos
            self._set_dir(going_pos)
            self._do_step()
            if going_pos:
                self._pos += 1
            else:
                self._pos -= 1

    def _do_step(self):
        """Generate a step pulse (toggle HIGH then LOW)."""
        self.step_value_func(1)
        # Brief high pulse — timer period guarantees enough low time
        self.step_value_func(0)

    def _set_dir(self, positive):
        """Set direction pin according to invert_dir flag."""
        if self.invert_dir:
            self.dir_value_func(0 if positive else 1)
        else:
            self.dir_value_func(1 if positive else 0)

    def _start_timer(self):
        if not self.timer_is_running:
            period_ms = max(1, int(1_000_000 / self._speed_sps) // 1000)
            # Create a FRESH Timer object each time — avoids re-init bug on RP2040
            # where calling timer.init() after timer.deinit() on the same Timer(-1)
            # object can silently fail and not fire any callbacks.
            self.timer = machine.Timer(-1)
            self.timer.init(period=period_ms,
                            mode=machine.Timer.PERIODIC,
                            callback=self._step_callback)
            self.timer_is_running = True


    def _stop_timer(self):
        if self.timer_is_running:
            self.timer.deinit()
            self.timer_is_running = False
            self.step_value_func(0)  # ensure step pin is low

    # ── Public API ────────────────────────────────────────────────────────────
    def speed(self, sps):
        """Set speed in steps per second. Restarts timer if running."""
        self._speed_sps = sps
        if self.timer_is_running:
            self._stop_timer()
            self._start_timer()

    def target(self, pos):
        """Move to target in steps (absolute). Non-blocking."""
        self.free_run_mode = 0
        self._target = pos
        if self._pos != self._target:
            self._start_timer()
        else:
            self._stop_timer()

    def target_deg(self, deg):
        """Move to target in degrees (absolute). Non-blocking."""
        self.target(round(deg * self._steps_per_rev / 360.0))

    def free_run(self, d):
        """
        Start continuous movement.
        d > 0 → positive direction
        d < 0 → negative direction
        """
        self.free_run_mode = 1 if d > 0 else -1
        self._start_timer()

    def stop(self):
        """Stop motor immediately. Syncs _target to _pos."""
        self.free_run_mode = 0
        self._stop_timer()
        self._target = self._pos

    def overwrite_pos(self, p):
        """Force-set current position in steps (e.g. after limit switch hit)."""
        self._pos    = p
        self._target = p

    def track_target(self):
        """Resume moving toward the last target (after overwrite_pos)."""
        self.free_run_mode = 0
        if self._pos != self._target:
            self._start_timer()

    def get_pos(self):
        """Return current position in steps."""
        return self._pos

    def get_pos_deg(self):
        """Return current position in degrees."""
        return self._pos * 360.0 / self._steps_per_rev

    def enable(self, state):
        """Enable or disable motor (if enable pin is connected)."""
        if self.en_pin is not None:
            if self.invert_enable:
                self.en_pin.value(0 if state else 1)
            else:
                self.en_pin.value(1 if state else 0)
