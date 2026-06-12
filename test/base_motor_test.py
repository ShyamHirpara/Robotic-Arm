"""
base_motor_test.py  --  Base motor LEFT/RIGHT rotation test (no camera)
========================================================================
Simplest possible hardware check: rotates ONLY the base (joint 1)
left and right a few times, then returns to 0 deg.

Run this FIRST after connecting to the Pico Wi-Fi to confirm:
  1. The Pico is reachable (http://192.168.4.1/).
  2. The base motor responds and the reported angle tracks the moves.
  3. Which physical direction is positive (watch the arm: +angle should
     be LEFT/CCW from above -- needed to verify BASE_DIRECTION in
     base_scan_stop.py).

USAGE:
  python base_motor_test.py --check        # ping only, NO movement
  python base_motor_test.py                # +-30 deg, 2 cycles
  python base_motor_test.py --angle 20 --cycles 1
  python base_motor_test.py --dry-run      # simulate (no Wi-Fi needed)

Wi-Fi:  RoboticArm_AP  /  password 12345678
"""

import argparse
import sys
import time

import requests

# Force UTF-8 output on Windows (prevents cp1252 UnicodeEncodeError)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

PICO_URL = "http://192.168.4.1/"
MAX_ANGLE = 70.0        # never command beyond this in a test
MOVE_TIMEOUT = 25       # s -- /stepper blocks on the Pico until move done
PAUSE_BETWEEN = 1.0     # s settle time between moves


def get_position(url):
    try:
        r = requests.get(url + "current_position", timeout=3)
        return r.json()
    except Exception:
        return None


def move_base(url, angle, dry_run=False):
    """Blocking absolute base move. Returns True on success."""
    if dry_run:
        print(f"  [DRY] base -> {angle:+.1f} deg (simulated)")
        time.sleep(0.2)
        return True
    print(f"  base -> {angle:+.1f} deg ... ", end="", flush=True)
    try:
        t0 = time.time()
        requests.get(url + f"stepper?num=1&angle={angle:.2f}",
                     timeout=MOVE_TIMEOUT)
        pos = get_position(url)
        reported = pos.get("joint1") if pos else None
        print(f"done in {time.time() - t0:.1f}s  "
              f"(Pico reports {reported:+.1f} deg)" if reported is not None
              else f"done in {time.time() - t0:.1f}s  (no position readback)")
        return True
    except Exception as e:
        print(f"FAILED: {e}")
        return False


def main():
    ap = argparse.ArgumentParser(description="Base left/right rotation test")
    ap.add_argument("--angle", type=float, default=30.0,
                    help="sweep amplitude in degrees (default 30)")
    ap.add_argument("--cycles", type=int, default=2,
                    help="number of left/right cycles (default 2)")
    ap.add_argument("--url", default=PICO_URL, help="Pico base URL")
    ap.add_argument("--check", action="store_true",
                    help="connection check only, no movement")
    ap.add_argument("--dry-run", action="store_true",
                    help="simulate moves, no Wi-Fi needed")
    args = ap.parse_args()

    angle = min(abs(args.angle), MAX_ANGLE)
    url = args.url if args.url.endswith("/") else args.url + "/"

    print("=" * 56)
    print("  BASE MOTOR TEST  (joint 1 only -- left/right)")
    print("=" * 56)

    # ── Connection check ──────────────────────────────────────────────────
    if args.dry_run:
        print("[ARM] DRY-RUN: robot simulated, no Wi-Fi needed.")
        pos = {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0}
    else:
        print(f"[ARM] Pinging {url}current_position ...")
        pos = get_position(url)
        if pos is None:
            print("[ARM] ERROR: Pico NOT reachable.")
            print("      1. Power the robot.")
            print("      2. Connect this PC to Wi-Fi 'RoboticArm_AP'"
                  " (password 12345678).")
            print("      3. Run again.")
            return 1
    print(f"[ARM] Connected. base={pos['joint1']:+.1f}  "
          f"shoulder={pos['joint2']:+.1f}  elbow={pos['joint3']:+.1f}")

    if args.check:
        print("[OK] Connection check passed (no movement requested).")
        return 0

    # ── Left / right sweep ────────────────────────────────────────────────
    print(f"\n[TEST] {args.cycles} cycle(s) of +-{angle:.0f} deg "
          f"(+ = LEFT/CCW from above)")
    for c in range(1, args.cycles + 1):
        print(f"\nCycle {c}/{args.cycles}:")
        if not move_base(url, +angle, args.dry_run):
            return 1
        time.sleep(PAUSE_BETWEEN)
        if not move_base(url, -angle, args.dry_run):
            return 1
        time.sleep(PAUSE_BETWEEN)

    print("\n[TEST] Returning to 0 deg ...")
    move_base(url, 0.0, args.dry_run)
    print("\n[OK] Base motor test complete.")
    print("     If +angle moved the base RIGHT instead of LEFT, set")
    print("     BASE_DIRECTION = -1 in base_scan_stop.py.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
