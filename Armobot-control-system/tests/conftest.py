"""
conftest.py — shared fixtures for Pick & Place integration tests.

Architecture:
  ┌─────────────────────┐   WebSocket (polling+ws)  ┌──────────────────────┐
  │  pytest test cases  │ ◄──────────────────────► │  Node.js backend     │
  └─────────────────────┘                           │  (test port 3099)    │
                                                    └──────────┬───────────┘
                                                               │ TCP :8181
                                                    ┌──────────▼───────────┐
                                                    │  MockPicoServer       │
                                                    └──────────────────────┘
"""
from __future__ import annotations

import os
import socket
import subprocess
import threading
import time

import pytest
import requests
import socketio as sio_lib

# ─── ports ────────────────────────────────────────────────────────────────────
BACKEND_TEST_PORT = 3099
MOCK_PICO_PORT    = 8181

BACKEND_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "backend")
)

# ─── sample position sets (importable by test modules) ────────────────────────
POSITIONS_2 = [
    {"order": 1, "gripper_state": "open",  "s1":  0,  "s2": 0,  "s3":  0},
    {"order": 2, "gripper_state": "close", "s1": 10,  "s2": 5,  "s3":  5},
]
POSITIONS_3 = [
    {"order": 1, "gripper_state": "open",  "s1":  0,  "s2":  0, "s3":  0},
    {"order": 2, "gripper_state": "close", "s1": 20,  "s2": 10, "s3": 10},
    {"order": 3, "gripper_state": "open",  "s1": -10, "s2":  5, "s3": -5},
]
POSITIONS_ANGLES = [
    {"order": 1, "gripper_state": "open",  "s1": 30, "s2": 15, "s3": -20},
    {"order": 2, "gripper_state": "close", "s1": -10,"s2":  5, "s3":  10},
]


# ─── Mock Pico TCP server ─────────────────────────────────────────────────────
class MockPicoServer:
    """
    Listens on MOCK_PICO_PORT, accepts the backend's persistent TCP connection,
    and responds to /position_order= commands.

    Controls:
      response_delay  — seconds before replying (default 0.3)
      should_timeout  — if True, never reply (tests backend timeout handling)
      should_error    — if True, reply with status=error:test_error
    """

    def __init__(self, host: str = "127.0.0.1", port: int = MOCK_PICO_PORT):
        self.host = host
        self.port = port
        self._server_sock: socket.socket | None = None
        self._client_conn: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()

        self.response_delay = 0.3
        self.should_timeout = False
        self.should_error   = False
        self.received_commands: list[str] = []

    def start(self) -> None:
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self.host, self.port))
        self._server_sock.listen(5)
        self._server_sock.settimeout(2.0)
        self._running = True
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        for s in [self._client_conn, self._server_sock]:
            if s:
                try: s.close()
                except Exception: pass

    def reset(self) -> None:
        """Called before every test — clears log and restores default behaviour."""
        with self._lock:
            self.received_commands.clear()
        self.response_delay = 0.3
        self.should_timeout = False
        self.should_error   = False

    def _accept_loop(self) -> None:
        while self._running:
            try:
                conn, _ = self._server_sock.accept()
                self._client_conn = conn
                self._handle_client(conn)
            except socket.timeout:
                continue
            except OSError:
                break

    def _handle_client(self, conn: socket.socket) -> None:
        buf = ""
        conn.settimeout(0.15)
        while self._running:
            try:
                chunk = conn.recv(512).decode(errors="replace")
                if not chunk:
                    break
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line.startswith("/position_order="):
                        self._on_pnp_command(conn, line)
                    # START_ and STOP silently ignored in mock
            except socket.timeout:
                continue
            except OSError:
                break
        try: conn.close()
        except: pass
        self._client_conn = None

    def _on_pnp_command(self, conn: socket.socket, cmd: str) -> None:
        with self._lock:
            self.received_commands.append(cmd)

        order = 0
        for seg in cmd.split("/"):
            if seg.startswith("position_order="):
                try: order = int(seg.split("=", 1)[1])
                except ValueError: pass

        if self.should_timeout:
            return  # no reply → backend timeout fires

        time.sleep(self.response_delay)

        status = "error:test_error" if self.should_error else "complete"
        reply  = f"/position_order={order}/status={status}\n"
        try:
            conn.sendall(reply.encode())
        except OSError:
            pass  # client disconnected — that's OK


# ─── Helpers ──────────────────────────────────────────────────────────────────
def _backend_alive(port: int = BACKEND_TEST_PORT, timeout: float = 1.0) -> bool:
    """Return True if the backend HTTP endpoint is reachable."""
    try:
        r = requests.post(
            f"http://localhost:{port}/api/auth/login",
            json={"username": "x", "password": "x"},
            timeout=timeout,
        )
        return True  # any HTTP response → server is up
    except Exception:
        return False


def _wait_for_backend(port: int = BACKEND_TEST_PORT, timeout: float = 12.0) -> bool:
    """Poll until backend is accepting connections or *timeout* expires."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _backend_alive(port):
            return True
        time.sleep(0.4)
    return False


def make_sio_client() -> sio_lib.SimpleClient:
    """
    Create and connect a SimpleClient with polling transport first so it
    falls back gracefully on Windows where raw WebSocket upgrades can time out.
    """
    client = sio_lib.SimpleClient()
    client.connect(
        f"http://localhost:{BACKEND_TEST_PORT}",
        transports=["polling", "websocket"],  # polling first avoids WS handshake timeout
        wait_timeout=10,
    )
    return client


# ─── Session-scoped fixtures ──────────────────────────────────────────────────
@pytest.fixture(scope="session")
def mock_pico():
    server = MockPicoServer()
    server.start()
    time.sleep(0.3)
    yield server
    server.stop()


@pytest.fixture(scope="session")
def backend_process(mock_pico):
    env = {
        **os.environ,
        "PICO_IP":   "127.0.0.1",
        "PICO_PORT": str(MOCK_PICO_PORT),
        "PORT":      str(BACKEND_TEST_PORT),
    }
    log_file = open(os.path.join(os.path.dirname(__file__), "backend_test.log"), "wb")
    proc = subprocess.Popen(
        ["node", "server.js"],
        cwd=BACKEND_DIR,
        env=env,
        stdout=log_file,
        stderr=subprocess.STDOUT,
    )

    if not _wait_for_backend(BACKEND_TEST_PORT, timeout=15):
        proc.kill()
        pytest.fail("Backend did not start. See backend_test.log for details.")

    yield proc

    proc.terminate()
    try: proc.wait(timeout=5)
    except subprocess.TimeoutExpired: proc.kill()
    log_file.close()


# ─── Function-scoped fixtures ─────────────────────────────────────────────────
@pytest.fixture
def sio_client(backend_process):
    """
    Fresh Socket.IO client per test.
    Teardown: emit stop_pnp → wait for backend to settle → disconnect.
    """
    # Ensure backend is alive before trying to connect (it may still be
    # recovering from a previous test's picoSocket reconnect)
    assert _wait_for_backend(timeout=8), \
        "Backend not reachable at start of test — is server.js running?"

    client = make_sio_client()
    yield client

    # ── Teardown: clean stop so backend state is fresh for next test ──────────
    try:
        client.emit("stop_pnp", {})
    except Exception:
        pass
    # Give backend time to process stop and cancel all timers
    time.sleep(0.6)
    try:
        client.disconnect()
    except Exception:
        pass
    # Extra settle: let Node event loop flush before next test connects
    time.sleep(0.3)


@pytest.fixture(autouse=True)
def reset_mock(mock_pico):
    """Reset mock controls before every test."""
    mock_pico.reset()
    time.sleep(0.05)


# ─── Event helpers ────────────────────────────────────────────────────────────
def wait_for_pnp(
    client: sio_lib.SimpleClient,
    *,
    timeout: float = 20.0,
    condition,
) -> dict | None:
    """
    Drain pnp_status events until *condition(data)* is True or *timeout* expires.
    Only catches TimeoutError (expected on empty queue); propagates real errors.
    """
    import socketio.exceptions as sio_exc

    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            event = client.receive(timeout=min(0.5, deadline - time.time()))
        except (sio_exc.TimeoutError, TimeoutError):
            continue
        except Exception:
            # Connection lost — stop polling
            break
        if event and event[0] == "pnp_status":
            data = event[1] if len(event) > 1 else {}
            if condition(data):
                return data
    return None


def drain_events(client: sio_lib.SimpleClient, seconds: float = 0.4) -> None:
    """Discard all queued events for *seconds*."""
    import socketio.exceptions as sio_exc
    deadline = time.time() + seconds
    while time.time() < deadline:
        try:
            client.receive(timeout=0.05)
        except (sio_exc.TimeoutError, TimeoutError, Exception):
            break
