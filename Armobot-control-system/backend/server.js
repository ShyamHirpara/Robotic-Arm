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

const PORT = 3000;
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
    db.run(`ALTER TABLE users ADD COLUMN ${col} TEXT`, (err) => {});
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
      function(err) {
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
  db.run(`DELETE FROM users WHERE id = ?`, [id], function(err) {
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
        [username, hash, role, company, address, city, country, id], function(err) {
        if (err) return res.status(500).json({ error: err.message });
        res.json({ message: 'User updated' });
      });
    } else {
      db.run(`UPDATE users SET username = ?, role = ?, company = ?, address = ?, city = ?, country = ? WHERE id = ?`, 
        [username, role, company, address, city, country, id], function(err) {
        if (err) return res.status(500).json({ error: err.message });
        res.json({ message: 'User updated' });
      });
    }
  } catch (err) {
    res.status(500).json({ error: 'Server error' });
  }
});

// --- Pico W TCP Bridge ---
const PICO_IP = '192.168.4.1';
const PICO_PORT = 81;
let picoSocket = null;

function connectToPico() {
  if (picoSocket) return;
  picoSocket = new net.Socket();
  
  picoSocket.connect(PICO_PORT, PICO_IP, () => {
    console.log(`Connected to Pico W TCP Bridge at ${PICO_IP}:${PICO_PORT}`);
  });

  picoSocket.on('data', (data) => {
    try {
      const stateStr = data.toString().trim();
      if (stateStr) {
        // Broadcast the raw string or parsed JSON to all connected clients
        io.emit('robot_state', stateStr);
      }
    } catch (e) {
      console.error('Error parsing Pico data:', e);
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
    // data: { axis: 1, dir: 0 }
    if (picoSocket && !picoSocket.destroyed) {
      picoSocket.write(`START_${data.axis}_${data.dir}\n`);
    }
  });

  socket.on('jog_stop', () => {
    if (picoSocket && !picoSocket.destroyed) {
      picoSocket.write(`STOP\n`);
    }
  });

  socket.on('disconnect', () => {
    console.log(`Client disconnected: ${socket.id}`);
  });
});


server.listen(PORT, () => {
  console.log(`Backend server running on http://localhost:${PORT}`);
});
