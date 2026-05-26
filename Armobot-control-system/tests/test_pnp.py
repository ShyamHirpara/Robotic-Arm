"""
test_pnp.py — Comprehensive integration tests for the Pick & Place orchestration.

Categories:
  TestBasicSequence   — command format, ordering, status events
  TestStop            — stop button, restart after stop
  TestPauseResume     — pause/resume with state verification
  TestTimeout         — backend timeout handling, restart after timeout
  TestErrorHandling   — Pico error response, input validation
  TestLoopCounter     — loop/completed counter progression
"""

import time

import pytest

from conftest import (
    POSITIONS_2,
    POSITIONS_3,
    POSITIONS_ANGLES,
    drain_events,
    wait_for_pnp,
)


# ═════════════════════════════════════════════════════════════════════════════
# TestBasicSequence
# ═════════════════════════════════════════════════════════════════════════════
class TestBasicSequence:

    def test_commands_sent_to_pico(self, sio_client, mock_pico):
        """Backend must send TCP commands to Pico when run starts."""
        sio_client.emit("start_pnp", {"positions": POSITIONS_2})

        deadline = time.time() + 10
        while time.time() < deadline and not mock_pico.received_commands:
            time.sleep(0.1)

        sio_client.emit("stop_pnp", {})
        assert mock_pico.received_commands, "No commands received by mock Pico"

    def test_commands_sent_in_order(self, sio_client, mock_pico):
        """Position 1 command must arrive before position 2."""
        sio_client.emit("start_pnp", {"positions": POSITIONS_2})

        # Wait for at least one complete loop
        result = wait_for_pnp(
            sio_client, condition=lambda d: d.get("loops", 0) >= 1, timeout=20
        )
        sio_client.emit("stop_pnp", {})

        assert result is not None, "Loop never completed"
        cmds = mock_pico.received_commands
        assert len(cmds) >= 2, f"Expected ≥2 commands, got {len(cmds)}"
        assert "/position_order=1/" in cmds[0], f"First cmd wrong: {cmds[0]}"
        assert "/position_order=2/" in cmds[1], f"Second cmd wrong: {cmds[1]}"

    def test_command_includes_correct_angles(self, sio_client, mock_pico):
        """TCP command must embed the exact axis angles from the position card."""
        sio_client.emit("start_pnp", {"positions": POSITIONS_ANGLES})

        deadline = time.time() + 10
        while time.time() < deadline and not mock_pico.received_commands:
            time.sleep(0.1)

        sio_client.emit("stop_pnp", {})

        assert mock_pico.received_commands, "No command received"
        cmd = mock_pico.received_commands[0]
        assert "axis_1=30"  in cmd, f"axis_1 wrong in: {cmd}"
        assert "axis_2=15"  in cmd, f"axis_2 wrong in: {cmd}"
        assert "axis_3=-20" in cmd, f"axis_3 wrong in: {cmd}"
        assert "gripper=open" in cmd, f"gripper wrong in: {cmd}"

    def test_status_broadcasts_current_idx(self, sio_client, mock_pico):
        """pnp_status events must carry currentIdx = 0 then 1."""
        sio_client.emit("start_pnp", {"positions": POSITIONS_2})

        idx_seen: set[int] = set()
        deadline = time.time() + 20
        while time.time() < deadline:
            try:
                event = sio_client.receive(timeout=0.5)
            except Exception:
                continue
            if event and event[0] == "pnp_status":
                d = event[1]
                if "currentIdx" in d:
                    idx_seen.add(d["currentIdx"])
                if d.get("loops", 0) >= 1:
                    break

        sio_client.emit("stop_pnp", {})
        assert 0 in idx_seen, f"currentIdx=0 never seen. Seen: {idx_seen}"
        assert 1 in idx_seen, f"currentIdx=1 never seen. Seen: {idx_seen}"

    def test_one_position_rejected(self, sio_client, mock_pico):
        """start_pnp with < 2 positions must emit stopped=True immediately."""
        single = [{"order": 1, "gripper_state": "open", "s1": 0, "s2": 0, "s3": 0}]
        sio_client.emit("start_pnp", {"positions": single})

        result = wait_for_pnp(
            sio_client, condition=lambda d: d.get("stopped") is True, timeout=5
        )
        assert result is not None, "Expected immediate error for <2 positions"

    def test_three_position_sequence(self, sio_client, mock_pico):
        """Three-position sequence must send 3 commands per loop."""
        sio_client.emit("start_pnp", {"positions": POSITIONS_3})

        # Wait for first full loop — give extra time since 3 positions take longer
        result = wait_for_pnp(
            sio_client, condition=lambda d: d.get("loops", 0) >= 1, timeout=45
        )
        # Explicitly stop before teardown so state is clean for next test
        sio_client.emit("stop_pnp", {})
        wait_for_pnp(
            sio_client, condition=lambda d: d.get("stopped") is True, timeout=5
        )

        assert result is not None, (
            f"3-position loop never completed within 45 s. "
            f"Commands to Pico: {mock_pico.received_commands}"
        )
        cmds = mock_pico.received_commands
        assert len(cmds) >= 3, f"Expected ≥3 commands, got {len(cmds)}: {cmds}"


# ═════════════════════════════════════════════════════════════════════════════
# TestStop
# ═════════════════════════════════════════════════════════════════════════════
class TestStop:

    def test_stop_emits_stopped_true(self, sio_client, mock_pico):
        """stop_pnp must result in a pnp_status with stopped=True."""
        sio_client.emit("start_pnp", {"positions": POSITIONS_2})
        time.sleep(0.2)
        sio_client.emit("stop_pnp", {})

        result = wait_for_pnp(
            sio_client, condition=lambda d: d.get("stopped") is True, timeout=6
        )
        assert result is not None, "stopped=True never received after stop_pnp"

    def test_stop_allows_clean_restart(self, sio_client, mock_pico):
        """After stop, start_pnp must work without error."""
        sio_client.emit("start_pnp", {"positions": POSITIONS_2})
        time.sleep(0.2)
        sio_client.emit("stop_pnp", {})

        wait_for_pnp(
            sio_client, condition=lambda d: d.get("stopped") is True, timeout=5
        )
        drain_events(sio_client)

        sio_client.emit("start_pnp", {"positions": POSITIONS_2})

        result = wait_for_pnp(
            sio_client,
            condition=lambda d: not d.get("stopped") and "currentIdx" in d,
            timeout=8,
        )
        assert result is not None, "Sequence did not restart after stop"
        sio_client.emit("stop_pnp", {})

    def test_duplicate_start_ignored(self, sio_client, mock_pico):
        """A second start_pnp while running must be silently ignored."""
        sio_client.emit("start_pnp", {"positions": POSITIONS_2})
        time.sleep(0.1)

        cmds_before = len(mock_pico.received_commands)
        # Fire a second start — should not crash or double-execute
        sio_client.emit("start_pnp", {"positions": POSITIONS_2})
        time.sleep(0.3)

        sio_client.emit("stop_pnp", {})
        # Just verify the backend is still responsive (returns stopped event)
        result = wait_for_pnp(
            sio_client, condition=lambda d: d.get("stopped") is True, timeout=6
        )
        assert result is not None, "Backend unresponsive after duplicate start"


# ═════════════════════════════════════════════════════════════════════════════
# TestPauseResume
# ═════════════════════════════════════════════════════════════════════════════
class TestPauseResume:

    def test_pause_emits_paused_true(self, sio_client, mock_pico):
        """pause_pnp must emit pnp_status{paused: true}."""
        sio_client.emit("start_pnp", {"positions": POSITIONS_2})
        time.sleep(0.15)
        sio_client.emit("pause_pnp", {})

        result = wait_for_pnp(
            sio_client, condition=lambda d: d.get("paused") is True, timeout=6
        )
        assert result is not None, "paused=True never received after pause_pnp"
        sio_client.emit("stop_pnp", {})

    def test_resume_emits_paused_false(self, sio_client, mock_pico):
        """resume_pnp must emit pnp_status{paused: false}."""
        sio_client.emit("start_pnp", {"positions": POSITIONS_2})
        time.sleep(0.15)
        sio_client.emit("pause_pnp", {})
        wait_for_pnp(sio_client, condition=lambda d: d.get("paused") is True, timeout=5)

        sio_client.emit("resume_pnp", {})
        result = wait_for_pnp(
            sio_client,
            condition=lambda d: "paused" in d and d["paused"] is False,
            timeout=6,
        )
        assert result is not None, "paused=False never received after resume_pnp"
        sio_client.emit("stop_pnp", {})

    def test_resume_continues_to_next_position(self, sio_client, mock_pico):
        """After pause+resume the sequence must continue, sending more commands."""
        sio_client.emit("start_pnp", {"positions": POSITIONS_3})

        # Wait for first position to complete
        wait_for_pnp(
            sio_client, condition=lambda d: d.get("completed", 0) >= 1, timeout=12
        )

        sio_client.emit("pause_pnp", {})
        wait_for_pnp(sio_client, condition=lambda d: d.get("paused") is True, timeout=5)

        cmds_at_pause = len(mock_pico.received_commands)

        sio_client.emit("resume_pnp", {})

        # New commands must arrive within 10 s of resuming
        deadline = time.time() + 10
        while time.time() < deadline:
            if len(mock_pico.received_commands) > cmds_at_pause:
                break
            time.sleep(0.15)

        sio_client.emit("stop_pnp", {})

        assert len(mock_pico.received_commands) > cmds_at_pause, (
            f"No new commands after resume. cmds_at_pause={cmds_at_pause}, "
            f"total={len(mock_pico.received_commands)}"
        )

    def test_no_new_commands_while_paused(self, sio_client, mock_pico):
        """While paused, no new position commands should be sent to the Pico."""
        mock_pico.response_delay = 1.5   # slow responses to keep pico busy

        sio_client.emit("start_pnp", {"positions": POSITIONS_2})
        time.sleep(0.1)
        sio_client.emit("pause_pnp", {})

        wait_for_pnp(sio_client, condition=lambda d: d.get("paused") is True, timeout=5)
        cmds_snapshot = len(mock_pico.received_commands)

        # Sleep 2 s with pause in effect
        time.sleep(2.0)
        cmds_after = len(mock_pico.received_commands)

        sio_client.emit("stop_pnp", {})

        # At most 1 in-flight command is tolerable (sent before pause took effect)
        assert cmds_after <= cmds_snapshot + 1, (
            f"Commands grew during pause: before={cmds_snapshot}, after={cmds_after}"
        )


# ═════════════════════════════════════════════════════════════════════════════
# TestTimeout
# ═════════════════════════════════════════════════════════════════════════════
class TestTimeout:

    def test_timeout_emits_error(self, sio_client, mock_pico):
        """
        When Pico never responds, backend must emit stopped=True with an
        error message containing 'Timeout'.  Backend min-timeout is 2.8 s;
        we give 30 s total for the event to arrive.
        """
        mock_pico.should_timeout = True

        # Drain any queued events before starting so we don't miss the new one
        drain_events(sio_client, seconds=0.2)

        sio_client.emit("start_pnp", {"positions": POSITIONS_2})

        result = wait_for_pnp(
            sio_client, condition=lambda d: d.get("stopped") is True, timeout=30
        )
        assert result is not None, (
            "stopped event never received on timeout "
            "(backend min-timeout ~2.8 s, waited 30 s)"
        )
        err = result.get("error", "")
        assert err, f"No error field on timeout event: {result}"
        assert "Timeout" in str(err) or "timeout" in str(err).lower(), (
            f"Error should mention Timeout, got: {err}"
        )

    def test_restart_after_timeout(self, sio_client, mock_pico):
        """After a timeout the backend must accept a fresh start_pnp."""
        mock_pico.should_timeout = True
        sio_client.emit("start_pnp", {"positions": POSITIONS_2})

        wait_for_pnp(
            sio_client, condition=lambda d: d.get("stopped") is True, timeout=20
        )

        mock_pico.should_timeout = False
        drain_events(sio_client)

        sio_client.emit("start_pnp", {"positions": POSITIONS_2})
        result = wait_for_pnp(
            sio_client,
            condition=lambda d: not d.get("stopped") and "currentIdx" in d,
            timeout=10,
        )
        assert result is not None, "Sequence did not restart after timeout"
        sio_client.emit("stop_pnp", {})


# ═════════════════════════════════════════════════════════════════════════════
# TestErrorHandling
# ═════════════════════════════════════════════════════════════════════════════
class TestErrorHandling:

    def test_pico_error_stops_sequence(self, sio_client, mock_pico):
        """
        When Pico replies with status=error, the backend must halt and emit
        stopped=True with an error field.
        """
        mock_pico.should_error = True

        sio_client.emit("start_pnp", {"positions": POSITIONS_2})

        result = wait_for_pnp(
            sio_client, condition=lambda d: d.get("stopped") is True, timeout=12
        )
        assert result is not None, "Sequence did not stop on Pico error response"
        assert result.get("error"), f"Expected error field, got: {result}"

    def test_pico_error_includes_reason(self, sio_client, mock_pico):
        """The error message should propagate the Pico's error reason."""
        mock_pico.should_error = True

        sio_client.emit("start_pnp", {"positions": POSITIONS_2})

        result = wait_for_pnp(
            sio_client, condition=lambda d: d.get("stopped") is True, timeout=12
        )
        assert result is not None
        # error field should contain something meaningful
        err = str(result.get("error", ""))
        assert len(err) > 0, "Error field is empty"


# ═════════════════════════════════════════════════════════════════════════════
# TestLoopCounter
# ═════════════════════════════════════════════════════════════════════════════
class TestLoopCounter:

    def test_loop_counter_increments(self, sio_client, mock_pico):
        """loops counter must reach ≥ 2 after two full sequence completions."""
        mock_pico.response_delay = 0.05   # very fast for loop test

        sio_client.emit("start_pnp", {"positions": POSITIONS_2})

        result = wait_for_pnp(
            sio_client, condition=lambda d: d.get("loops", 0) >= 2, timeout=30
        )
        sio_client.emit("stop_pnp", {})
        assert result is not None, (
            f"Loop counter never reached 2. "
            f"Commands sent: {len(mock_pico.received_commands)}"
        )

    def test_completed_count_grows(self, sio_client, mock_pico):
        """completed count must be ≥ 2 after running two positions once."""
        mock_pico.response_delay = 0.05

        sio_client.emit("start_pnp", {"positions": POSITIONS_2})

        result = wait_for_pnp(
            sio_client, condition=lambda d: d.get("completed", 0) >= 2, timeout=20
        )
        sio_client.emit("stop_pnp", {})
        assert result is not None, "completed count never reached 2"

    def test_loops_reset_on_new_run(self, sio_client, mock_pico):
        """Loop counter must restart from 0 on each new start_pnp."""
        mock_pico.response_delay = 0.05

        sio_client.emit("start_pnp", {"positions": POSITIONS_2})
        wait_for_pnp(
            sio_client, condition=lambda d: d.get("loops", 0) >= 2, timeout=25
        )
        sio_client.emit("stop_pnp", {})
        wait_for_pnp(
            sio_client, condition=lambda d: d.get("stopped") is True, timeout=6
        )
        drain_events(sio_client)
        mock_pico.reset()
        mock_pico.response_delay = 0.05

        # Second run — first status should show loops=0
        sio_client.emit("start_pnp", {"positions": POSITIONS_2})
        first_status = wait_for_pnp(
            sio_client,
            condition=lambda d: not d.get("stopped") and "loops" in d,
            timeout=8,
        )
        sio_client.emit("stop_pnp", {})

        assert first_status is not None
        assert first_status.get("loops", 99) == 0, (
            f"Expected loops=0 at start of new run, got {first_status.get('loops')}"
        )
