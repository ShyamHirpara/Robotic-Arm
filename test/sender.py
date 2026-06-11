"""
sender.py  --  Arm command sender (uses ArmControl, same as original)
======================================================================
Uses ArmControl class which manages the TCP connection properly
and prevents flooding the Pico with rapid requests.

Only Joint 1 (base) is active.
Joint 2, Joint 3, and Gripper are COMMENTED OUT.
"""

import time
import os, sys
import config

# ArmControl package is at ARMOBOT-SHYAM/ArmControl/
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PROJECT_ROOT)

from ArmControl import ArmControl


def sender_loop(command_queue, shared_state):
    print("[Sender] Thread started")

    arm = ArmControl()

    while True:
        if not command_queue.empty():
            d_j1, d_j2, d_j3 = command_queue.get()

            try:
                pos = arm.get_current_position()
                if not pos:
                    print("[SENDER BASE] Pico offline or unreachable. Retrying...")
                    shared_state["command_pending"] = False
                    continue

                curr_deg = pos['joint1']

                # Joint 1 (base) -- ACTIVE
                if d_j1 != 0:
                    tgt_deg = curr_deg + d_j1
                    arm.move_joint(1, tgt_deg)

                    # Update shared_state with EXPECTED post-move position
                    # so vision immediately knows the arm's new location
                    shared_state["current_degree"] = tgt_deg
                else:
                    tgt_deg = curr_deg
                    shared_state["current_degree"] = curr_deg

                # Joint 2 (shoulder) -- COMMENTED OUT
                # if d_j2 != 0:
                #     arm.move_joint(2, pos['joint2'] + d_j2)

                # Joint 3 (elbow) -- COMMENTED OUT
                # if d_j3 != 0:
                #     arm.move_joint(3, pos['joint3'] + d_j3)

                # Gripper -- COMMENTED OUT
                # arm.set_gripper("close")
                # arm.set_gripper("open")

                dist_deg = d_j1
                curr_mm = curr_deg * config.STEPPER1
                tgt_mm = tgt_deg * config.STEPPER1
                dist_mm = d_j1 * config.STEPPER1

                print(
                    f"[SENDER BASE] "
                    f"curr_deg={curr_deg:+.1f} deg  "
                    f"curr_pos={curr_mm:+.0f} mm"
                    f"  |  "
                    f"tgt_deg={tgt_deg:+.1f} deg  "
                    f"tgt_pos={tgt_mm:+.0f} mm"
                    f"  |  "
                    f"dist_deg={dist_deg:+d} deg  "
                    f"dist_pos={dist_mm:+.0f} mm"
                )

            except Exception as e:
                print("Sender error:", e)
            finally:
                # Allow vision to queue next command
                shared_state["command_pending"] = False

        time.sleep(0.02)
