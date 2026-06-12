const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const net = require('net');
const sqlite3 = require('sqlite3').verbose();
const bcrypt = require('bcryptjs');
const jwt = require('jsonwebtoken');
const cors = require('cors');

const app = express();
const server = http.createServer(app);
const io = new Server(server, {
  cors: { origin: '*', methods: ['GET', 'POST'] }
});
app.use(cors());
app.use(express.json());

const PORT = parseInt(process.env.PORT) || 3000;
const JWT_SECRET = 'supersecret_armobot_key'; // Use env variable in production

// Initialize SQLite database
const db = new sqlite3.Database('./database.sqlite', (err) => {
  if (err) console.error('Database connection error:', err.message);
  else console.log('Connected to SQLite database.');
});

// Create Users Table
db.serialize(() => {
  db.run(`
    CREATE TABLE IF NOT EXISTS users (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      username TEXT UNIQUE,
      password_hash TEXT,
      role TEXT DEFAULT 'user',
      company TEXT,
      address TEXT,
      city TEXT,
      country TEXT,
      created_at DATETIME DEFAULT CURRENT_TIMESTAMP
    )
  `);

  // Auto-migrate schema for existing tables
  ['company', 'address', 'city', 'country'].forEach(col => {
    db.run(`ALTER TABLE users ADD COLUMN ${col} TEXT`, (err) => { });
  });
  db.run(`ALTER TABLE users ADD COLUMN created_at TEXT DEFAULT '2026-05-13 12:00:00'`, (err) => {
    if (err && !err.message.includes('duplicate column')) {
      console.log('Migration note (created_at):', err.message);
    }
  });

  // Create a default admin if none exists
  db.get(`SELECT * FROM users WHERE role = 'admin'`, async (err, row) => {
    if (!row) {
      const hash = await bcrypt.hash('admin123', 10);
      db.run(`INSERT INTO users (username, password_hash, role) VALUES (?, ?, ?)`, ['admin', hash, 'admin']);
      console.log('Default admin created: admin / admin123');
    }
  });
});

// --- Middleware ---
const authenticateToken = (req, res, next) => {
  const authHeader = req.headers['authorization'];
  const token = authHeader && authHeader.split(' ')[1];
  if (!token) return res.status(401).json({ error: 'Access denied' });

  jwt.verify(token, JWT_SECRET, (err, user) => {
    if (err) return res.status(403).json({ error: 'Invalid token' });
    req.user = user;
    next();
  });
};

const requireAdmin = (req, res, next) => {
  if (req.user.role !== 'admin') {
    return res.status(403).json({ error: 'Admin access required' });
  }
  next();
};

// --- Auth Routes ---
app.post('/api/users', authenticateToken, requireAdmin, async (req, res) => {
  const { username, password, role, company, address, city, country } = req.body;
  const userRole = role || 'user';
  try {
    const hash = await bcrypt.hash(password, 10);
    db.run(
      `INSERT INTO users (username, password_hash, role, company, address, city, country) VALUES (?, ?, ?, ?, ?, ?, ?)`,
      [username, hash, userRole, company, address, city, country],
      function (err) {
        if (err) {
          if (err.message.includes('UNIQUE constraint failed')) {
            return res.status(400).json({ error: 'Username already exists' });
          }
          return res.status(500).json({ error: 'Database error' });
        }
        res.json({ message: 'User created successfully', id: this.lastID });
      });
  } catch (err) {
    res.status(500).json({ error: 'Server error' });
  }
});

app.post('/api/auth/login', (req, res) => {
  const { username, password } = req.body;
  db.get(`SELECT * FROM users WHERE username = ?`, [username], async (err, user) => {
    if (err) return res.status(500).json({ error: 'Database error' });
    if (!user) return res.status(400).json({ error: 'Invalid credentials' });

    const isMatch = await bcrypt.compare(password, user.password_hash);
    if (!isMatch) return res.status(400).json({ error: 'Invalid credentials' });

    const token = jwt.sign({ id: user.id, username: user.username, role: user.role }, JWT_SECRET, { expiresIn: '12h' });
    res.json({ token, user: { id: user.id, username: user.username, role: user.role } });
  });
});

// --- Admin Routes ---
app.get('/api/users', authenticateToken, requireAdmin, (req, res) => {
  db.all(`SELECT id, username, role, company, address, city, country, created_at FROM users`, [], (err, rows) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(rows);
  });
});

app.delete('/api/users/:id', authenticateToken, requireAdmin, (req, res) => {
  const { id } = req.params;
  db.run(`DELETE FROM users WHERE id = ?`, [id], function (err) {
    if (err) return res.status(500).json({ error: err.message });
    res.json({ message: 'User deleted' });
  });
});

app.put('/api/users/:id', authenticateToken, requireAdmin, async (req, res) => {
  const { id } = req.params;
  const { username, password, role, company, address, city, country } = req.body;
  try {
    if (password) {
      const hash = await bcrypt.hash(password, 10);
      db.run(`UPDATE users SET username = ?, password_hash = ?, role = ?, company = ?, address = ?, city = ?, country = ? WHERE id = ?`,
        [username, hash, role, company, address, city, country, id], function (err) {
          if (err) return res.status(500).json({ error: err.message });
          res.json({ message: 'User updated' });
        });
    } else {
      db.run(`UPDATE users SET username = ?, role = ?, company = ?, address = ?, city = ?, country = ? WHERE id = ?`,
        [username, role, company, address, city, country, id], function (err) {
          if (err) return res.status(500).json({ error: err.message });
          res.json({ message: 'User updated' });
        });
    }
  } catch (err) {
    res.status(500).json({ error: 'Server error' });
  }
});

// --- Pico W TCP Bridge ---
const PICO_IP = process.env.PICO_IP || '192.168.137.50';
const PICO_PORT = parseInt(process.env.PICO_PORT) || 81;
let picoSocket = null;

// --- Pick & Place Orchestration State ---
let pnpRunning = false;
let pnpPositions = [];
let pnpCurrentIdx = 0;
let pnpLoops = 0;
let pnpCompleted = 0;
let pnpStopFlag = false;
let pnpPaused = false;   // pause flag — halts between steps (used in auto mode)
let pnpMode = 'auto';    // 'auto' or 'manual'
let pnpWaitingForManual = false; // true if manual mode step is completed and waiting for next command
let pnpStepTimeout = null;    // response-wait timeout for current step
let pnpInterStepTimer = null;  // 300ms inter-step delay (must be cancellable)
let pnpResponseCb = null;
let picoDataBuffer = '';

// Joint jog speeds (steps/sec) and steps_per_rev — mirrors Pico firmware constants
const JOINT_CONFIG = [
  { jog_sps: 600, steps_per_rev: 7971 }, // Axis 1
  { jog_sps: 550, steps_per_rev: 10500 }, // Axis 2
  { jog_sps: 1111, steps_per_rev: 10000 }, // Axis 3
];

/**
 * Calculate worst-case travel time (ms) for a position.
 * For each axis: |delta_deg| / deg_per_sec + 500 ms gripper settle.
 * deg_per_sec = (jog_sps / steps_per_rev) * 360
 * Axes are now moved CONCURRENTLY on the Pico (all target_deg() calls fire
 * at the same time), so total time = MAX of all axis travel times.
 * We add a 1.5x safety margin and a 2 s floor.
 */
function calcTimeout(fromState, toPos) {
  let maxMs = 0;
  const axes = [
    { from: fromState.s1, to: toPos.s1, cfg: JOINT_CONFIG[0] },
    { from: fromState.s2, to: toPos.s2, cfg: JOINT_CONFIG[1] },
    { from: fromState.s3, to: toPos.s3, cfg: JOINT_CONFIG[2] },
  ];
  for (const ax of axes) {
    const degPerSec = (ax.cfg.jog_sps / ax.cfg.steps_per_rev) * 360;
    const ms = (Math.abs(ax.to - ax.from) / degPerSec) * 1000;
    if (ms > maxMs) maxMs = ms;  // MAX — concurrent axis movement
  }
  return Math.max(2000, Math.ceil(maxMs * 1.5)) + 800; // +800 ms gripper settle
}

/** Current live robot state snapshot (updated from Pico TCP telemetry) */
let robotState = { s1: 0, s2: 0, s3: 0 };

function connectToPico() {
  if (picoSocket) return;
  picoSocket = new net.Socket();

  picoSocket.connect(PICO_PORT, PICO_IP, () => {
    console.log(`Connected to Pico W TCP Bridge at ${PICO_IP}:${PICO_PORT}`);
  });

  picoSocket.on('data', (data) => {
    picoDataBuffer += data.toString();
    const lines = picoDataBuffer.split('\n');
    picoDataBuffer = lines.pop(); // keep any incomplete line

    for (const rawLine of lines) {
      const line = rawLine.trim();
      if (!line) continue;

      // ── Pick & Place response lines (e.g. /position_order=1/status=complete) ──
      if (line.startsWith('/position_order=') && pnpResponseCb) {
        const cb = pnpResponseCb;
        pnpResponseCb = null;
        clearTimeout(pnpStepTimeout);
        cb(line);
        continue;
      }

      // ── Regular JSON telemetry → broadcast to frontend ──────────────────────
      try {
        const json = JSON.parse(line);
        robotState.s1 = json.s1 !== undefined ? json.s1 : robotState.s1;
        robotState.s2 = json.s2 !== undefined ? json.s2 : robotState.s2;
        robotState.s3 = json.s3 !== undefined ? json.s3 : robotState.s3;
        robotState.gripper_state = json.gripper_state !== undefined ? json.gripper_state : robotState.gripper_state;
        io.emit('robot_state', line);
      } catch (e) { /* not JSON — ignore */ }
    }
  });

  picoSocket.on('error', (err) => {
    console.error(`Pico TCP Error: ${err.message}`);
    picoSocket.destroy();
  });

  picoSocket.on('close', () => {
    console.log('Pico TCP connection closed. Retrying in 2 seconds...');
    picoSocket = null;
    setTimeout(connectToPico, 2000);
  });
}

// Start persistent connection
connectToPico();

// --- WebSocket Event Handling ---
io.on('connection', (socket) => {
  console.log(`Frontend client connected via WebSocket: ${socket.id}`);

  socket.on('jog_start', (data) => {
    if (picoSocket && !picoSocket.destroyed) {
      picoSocket.write(`START_${data.axis}_${data.dir}\n`);
    }
  });

  socket.on('jog_stop', () => {
    if (picoSocket && !picoSocket.destroyed) {
      picoSocket.write(`STOP\n`);
    }
  });

  // ── Pick & Place: Start ────────────────────────────────────────────────────
  socket.on('start_pnp', ({ positions, mode = 'auto' }) => {
    if (pnpRunning) return;
    if (!positions || positions.length < 2) {
      socket.emit('pnp_status', { stopped: true, error: 'Need at least 2 positions.' });
      return;
    }
    // Cancel any lingering inter-step timer from a previous run
    if (pnpInterStepTimer) { clearTimeout(pnpInterStepTimer); pnpInterStepTimer = null; }

    pnpRunning = true;
    pnpStopFlag = false;
    pnpPaused = false;
    pnpMode = mode;
    pnpWaitingForManual = false;
    pnpPositions = positions;
    pnpCurrentIdx = 0;
    pnpLoops = 0;
    pnpCompleted = 0;

    console.log(`[PnP] Starting sequence with ${positions.length} positions (looping until stop).`);
    runPnpStep();
  });

  // ── Pick & Place: Stop ────────────────────────────────────────────────────
  socket.on('stop_pnp', () => {
    console.log('[PnP] Stop requested.');
    pnpStopFlag = true;
    pnpPaused = false;
    pnpWaitingForManual = false;
    if (pnpInterStepTimer) { clearTimeout(pnpInterStepTimer); pnpInterStepTimer = null; }
    if (pnpStepTimeout) { clearTimeout(pnpStepTimeout); pnpStepTimeout = null; }
    pnpResponseCb = null;
    pnpRunning = false;
    if (picoSocket && !picoSocket.destroyed) {
      try { picoSocket.write('STOP\n'); } catch (_) { }
    }
    io.emit('pnp_status', { stopped: true, reason: 'user_stop' });
  });

  // ── Pick & Place: Pause ───────────────────────────────────────────────────
  socket.on('pause_pnp', () => {
    if (!pnpRunning || pnpPaused || pnpMode === 'manual') return;
    console.log('[PnP] Paused.');
    pnpPaused = true;
    // Cancel any pending inter-step timer so no new step fires while paused
    if (pnpInterStepTimer) { clearTimeout(pnpInterStepTimer); pnpInterStepTimer = null; }
    io.emit('pnp_status', {
      currentIdx: pnpCurrentIdx, completed: pnpCompleted,
      remaining: pnpPositions.length - pnpCurrentIdx - 1,
      loops: pnpLoops, stopped: false, paused: true,
      mode: pnpMode, waitingForManual: pnpWaitingForManual
    });
  });

  // ── Pick & Place: Resume ──────────────────────────────────────────────────
  socket.on('resume_pnp', () => {
    if (!pnpRunning || !pnpPaused || pnpMode === 'manual') return;
    console.log('[PnP] Resumed.');
    pnpPaused = false;
    io.emit('pnp_status', {
      currentIdx: pnpCurrentIdx, completed: pnpCompleted,
      remaining: pnpPositions.length - pnpCurrentIdx - 1,
      loops: pnpLoops, stopped: false, paused: false,
      mode: pnpMode, waitingForManual: pnpWaitingForManual
    });
    runPnpStep();
  });

  // ── Pick & Place: Manual Next/Prev ─────────────────────────────────────────
  socket.on('pnp_manual_step', ({ direction }) => {
    if (!pnpRunning || pnpMode !== 'manual' || !pnpWaitingForManual) return;
    
    if (direction === 'next') {
      pnpCurrentIdx++;
      if (pnpCurrentIdx >= pnpPositions.length) pnpCurrentIdx = 0;
    } else if (direction === 'prev') {
      pnpCurrentIdx--;
      if (pnpCurrentIdx < 0) pnpCurrentIdx = pnpPositions.length - 1;
    }
    
    console.log(`[PnP] Manual step ${direction} to idx ${pnpCurrentIdx}`);
    runPnpStep();
  });

  // ── Pick & Place: Manual Goto Index ────────────────────────────────────────
  socket.on('pnp_manual_goto', ({ index }) => {
    if (!pnpRunning || pnpMode !== 'manual' || !pnpWaitingForManual) return;
    if (index >= 0 && index < pnpPositions.length) {
      pnpCurrentIdx = index;
      console.log(`[PnP] Manual goto idx ${pnpCurrentIdx}`);
      runPnpStep();
    }
  });

  socket.on('disconnect', () => {
    console.log(`Client disconnected: ${socket.id}`);
  });
});

// ── PnP Step Runner (sequential, request-response) ────────────────────────────
function broadcastPnpStatus() {
  io.emit('pnp_status', {
    currentIdx: pnpCurrentIdx,
    completed: pnpCompleted,
    remaining: pnpPositions.length - pnpCurrentIdx - 1,
    loops: pnpLoops,
    stopped: false,
    paused: pnpPaused,
    mode: pnpMode,
    waitingForManual: pnpWaitingForManual
  });
}

function runPnpStep() {
  if (pnpStopFlag || !pnpRunning) return;

  // ── Pause: hold here until resumed ──────────────────────────────────────
  if (pnpPaused && pnpMode === 'auto') return;  // resume_pnp will call runPnpStep() again
  
  pnpWaitingForManual = false; // We are now executing a step

  const pos = pnpPositions[pnpCurrentIdx];
  const timeoutMs = calcTimeout(robotState, pos);

  console.log(`[PnP] Sending position ${pos.order} (idx ${pnpCurrentIdx}), timeout ${timeoutMs}ms`);

  broadcastPnpStatus();

  // Build Pico HTTP-style command
  const cmd = `/position_order=${pos.order}/axis_1=${pos.s1}/axis_2=${pos.s2}/axis_3=${pos.s3}/gripper=${pos.gripper_state}\n`;

  if (picoSocket && !picoSocket.destroyed) {
    picoSocket.write(cmd);
  } else {
    console.error('[PnP] Pico not connected — aborting.');
    pnpRunning = false;
    io.emit('pnp_status', { stopped: true, error: 'Pico not connected.' });
    return;
  }

  // Set timeout for this step only if in auto mode
  if (pnpMode === 'auto') {
    pnpStepTimeout = setTimeout(() => {
      console.error(`[PnP] Timeout on position ${pos.order}`);
      pnpResponseCb = null;
      pnpRunning = false;
      // Send STOP so Pico aborts any in-progress movement before next run
      if (picoSocket && !picoSocket.destroyed) {
        picoSocket.write('STOP\n');
      }
      io.emit('pnp_status', { stopped: true, error: `Timeout on position ${pos.order}` });
    }, timeoutMs);
  }

  // Register one-shot response callback
  pnpResponseCb = (responseLine) => {
    // responseLine format: /position_order=1/status=complete OR /position_order=1/status=error
    const statusMatch = responseLine.match(/\/status=([^/\s]+)/);
    const status = statusMatch ? statusMatch[1].toLowerCase() : 'error';

    if (status === 'complete') {
      pnpCompleted++;
      // Update robotState optimistically so next timeout is accurate
      robotState.s1 = pos.s1;
      robotState.s2 = pos.s2;
      robotState.s3 = pos.s3;

      if (pnpMode === 'auto') {
        pnpCurrentIdx++;

        if (pnpCurrentIdx >= pnpPositions.length) {
          // One full loop done — loop back
          pnpLoops++;
          pnpCurrentIdx = 0;
          console.log(`[PnP] Loop ${pnpLoops} complete. Restarting sequence.`);
          broadcastPnpStatus();
        }

        if (!pnpStopFlag) {
          // Inter-step delay then continue (cancellable so stop/pause works cleanly)
          pnpInterStepTimer = setTimeout(() => { pnpInterStepTimer = null; runPnpStep(); }, 1500);
        } else {
          pnpRunning = false;
          io.emit('pnp_status', { stopped: true, reason: 'user_stop' });
        }
      } else {
        // Manual mode: Wait here for next command
        pnpWaitingForManual = true;
        console.log(`[PnP] Manual step complete. Waiting for user.`);
        broadcastPnpStatus();
      }
    } else {
      // Error or emergency
      const reason = responseLine.replace(/\/position_order=\d+\/status=/, '');
      console.error(`[PnP] Error on position ${pos.order}: ${reason}`);
      pnpRunning = false;
      io.emit('pnp_status', { stopped: true, error: `Error at position ${pos.order}: ${reason}` });
    }
  };
}


server.listen(PORT, () => {
  console.log(`Backend server running on http://localhost:${PORT}`);
});
