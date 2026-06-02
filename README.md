# 🤖 ARMOBOT — 3-Axis Robotic Arm Control System

A full-stack IoT project that controls a **3-axis robotic arm** via a **Raspberry Pi Pico W**. The arm is operated through a modern **React web dashboard** — themed with the *Steel & Fire* industrial dark design — that communicates in real-time with the microcontroller over Wi-Fi, no internet connection required.

---

## 📐 System Architecture

```mermaid
flowchart TB
    subgraph USER["🖥️ User's Device"]
        direction TB
        subgraph FE["React Frontend — Vite :5173"]
            FE1["🔐 Login / Admin Dashboard"]
            FE2["🎮 Control Panel — Jog & Angle"]
            FE3["📊 Live Dashboard"]
            FE4["🤖 3D Visualization"]
            FE5["📦 Pick & Place Sequencer"]
        end
        subgraph BE["Node.js Backend — Express :3000"]
            BE1["🔑 Auth API — SQLite + JWT"]
            BE2["👥 User Management — CRUD"]
            BE3["🔌 TCP Bridge ↔ Socket.IO"]
        end
        FE -->|HTTP REST| BE
        FE <-->|WebSocket| BE
    end

    subgraph PICO["📟 Raspberry Pi Pico W — MicroPython"]
        direction TB
        subgraph FW["main.py"]
            P1["🌐 HTTP Server — Port 80"]
            P2["📡 TCP Server — Port 81"]
            P3["⚙️ Stepper Library — Timer-driven"]
            P4["📐 Inverse Kinematics"]
            P5["🛡️ Safety — Limits, E-Stop, ISR"]
        end
        subgraph HW["Hardware"]
            H1["3× NEMA 17 Steppers"]
            H2["1× MG90S Servo Gripper"]
            H3["3× Limit Switches"]
            H4["2× Ultrasonic Sensors"]
            H5["1× Emergency Button"]
        end
        FW --> HW
    end

    BE <-->|"TCP :81 — Wi-Fi AP 192.168.4.1"| PICO
    FE -->|"HTTP Proxy /pico → :80"| PICO

    style USER fill:#1a1a2e,stroke:#e0d5c1,color:#fff
    style FE fill:#0f3460,stroke:#e0d5c1,color:#fff
    style BE fill:#16213e,stroke:#e0d5c1,color:#fff
    style PICO fill:#1b2631,stroke:#e0d5c1,color:#fff
    style FW fill:#2c3e50,stroke:#e0d5c1,color:#fff
    style HW fill:#34495e,stroke:#e0d5c1,color:#fff
```

---

## 📁 Project Structure

```
ARMOBOT-SHYAM/
│
├── start_armobot.bat                # One-click launcher (Windows)
│
└── Armobot-control-system/
    │
    ├── main.py                          # Pico W firmware (MicroPython)
    │
    ├── lib/
    │   └── stepper/
    │       └── __init__.py              # micropython-stepper library (Timer-based)
    │
    ├── backend/
    │   ├── server.js                    # Express + Socket.IO + SQLite backend
    │   ├── package.json
    │   └── database.sqlite              # Auto-generated user database
    │
    ├── frontend/
    │   ├── src/
    │   │   ├── App.jsx                  # Root component with routing & auth
    │   │   ├── main.jsx                 # React entry point
    │   │   ├── index.css                # Global styles (Steel & Fire theme tokens)
    │   │   └── components/
    │   │       ├── Login.jsx            # User login form
    │   │       ├── ConnectionSetup.jsx  # Wi-Fi connection prompt
    │   │       ├── Dashboard.jsx        # Live joint angle & gripper display
    │   │       ├── ControlPanel.jsx     # Stepper motor jog & angle control
    │   │       ├── GripperPanel.jsx     # Gripper open/close & calibration
    │   │       ├── PickAndPlacePanel.jsx # Full Pick & Place sequencer (auto/manual)
    │   │       ├── RobotVisualization.jsx # Real-time 3D arm model (Three.js)
    │   │       └── AdminDashboard.jsx   # User management (CRUD)
    │   ├── public/
    │   │   ├── armobot-logo.png         # ARNOBOT branding logo
    │   │   └── favicon.svg
    │   ├── vite.config.js               # Vite config with /pico proxy
    │   └── package.json
    │
    ├── Raspberry Pi Pico - Circuit Connection Diagram.fzz   # Fritzing schematic
    ├── Raspberry Pi Pico - Circuit Connection Diagram.svg   # Viewable circuit diagram
    └── Raspberry Pi Pico pinout diagram.svg                 # Pico W pin reference
```

---

## ⚙️ Hardware Requirements

| Component | Quantity | Purpose |
|---|---|---|
| Raspberry Pi Pico W | 1 | Main microcontroller (Wi-Fi) |
| NEMA 17 Stepper Motors | 3 | Joint 1 (Base), Joint 2 (Shoulder), Joint 3 (Elbow) |
| A4988 / DRV8825 Stepper Drivers | 3 | Motor driving |
| MG90S Servo Motor | 1 | Gripper open/close |
| Limit Switches | 3 | Mechanical homing and safety |
| HC-SR04 Ultrasonic Sensors | 2 | Distance measurement (left & right) |
| Push Button (NO) | 1 | Emergency stop |
| 12V Power Supply | 1 | Stepper motor power |
| 5V Power Supply | 1 | Pico W and servo power |

---

## 📏 Joint Angle Limits

| Joint | Min | Max | Notes |
|---|---|---|---|
| Joint 1 — Base | −170° | +155° | Fixed hardware limit |
| Joint 2 — Shoulder | Dynamic | Dynamic | Coupled via IK (`solve_d2`) |
| Joint 3 — Elbow | Dynamic | Dynamic | Coupled via IK (`solve_d3`) |

> Joint 2 and Joint 3 limits update dynamically based on each other's current position, preventing self-collision at all times.

---

## 🔌 Pin Mapping (Raspberry Pi Pico W)

| Function | GPIO Pin |
|---|---|
| **Joint 1** — Step | GP11 |
| **Joint 1** — Dir | GP9 |
| **Joint 2** — Step | GP13 |
| **Joint 2** — Dir | GP7 |
| **Joint 3** — Step | GP15 |
| **Joint 3** — Dir | GP14 |
| **Gripper Servo** (PWM) | GP26 |
| **Limit Switch 1** | GP3 |
| **Limit Switch 2** | GP2 |
| **Limit Switch 3** | GP4 |
| **Ultrasonic Right** — Trig | GP18 |
| **Ultrasonic Right** — Echo | GP19 |
| **Ultrasonic Left** — Trig | GP20 |
| **Ultrasonic Left** — Echo | GP21 |
| **Emergency Stop Button** | GP16 |

### Raspberry Pi Pico W Pinout Reference

![Pico W Pinout Diagram](Armobot-control-system/Raspberry%20Pi%20Pico%20pinout%20diagram.svg)

### Circuit Connection Diagram

![Circuit Connection Diagram](Armobot-control-system/Raspberry%20Pi%20Pico%20-%20Circuit%20Connection%20Diagram.svg)

---

## 🚀 Getting Started

### Prerequisites

- [Node.js](https://nodejs.org/) (v18+ recommended)
- [Thonny IDE](https://thonny.org/) (for flashing MicroPython to Pico W)
- MicroPython firmware installed on Raspberry Pi Pico W

### 1. Flash the Pico W Firmware

1. Install MicroPython on your Pico W ([official guide](https://www.raspberrypi.com/documentation/microcontrollers/micropython.html)).
2. Open **Thonny** and connect to the Pico W.
3. Install the stepper library on the Pico via the REPL:

   ```python
   import mip
   mip.install("github:redoxcode/micropython-stepper")
   ```

4. Upload the following to the Pico W's filesystem:
   - `main.py` → root (`/`)
   - `lib/stepper/__init__.py` → `/lib/stepper/__init__.py`
5. Restart the Pico W. It will:
   - Create a Wi-Fi Access Point: **`RoboticArm_AP`** (password: `12345678`)
   - Start an HTTP server on port **80**
   - Start a TCP bridge on port **81**

---

### 2. Quick Start — Windows (Recommended)

A one-click launcher script is included for Windows users. From the project root, double-click or run:

```bat
start_armobot.bat
```

This script automatically:
1. Starts the **backend** server in a dedicated terminal window
2. Starts the **frontend** dev server in a dedicated terminal window
3. Waits 5 seconds for both servers to initialize
4. Opens `http://localhost:5173` in your default browser

---

### 3. Manual Start

#### Backend

```bash
cd Armobot-control-system/backend
npm install
npm start
```

The backend runs on `http://localhost:3000` and:

- Provides JWT-based authentication API
- Manages users via SQLite database
- Creates a default admin account: **`admin`** / **`admin123`**
- Maintains a persistent TCP connection to the Pico W (`192.168.4.1:81`)
- Bridges real-time jog and telemetry data between the Pico and the frontend via Socket.IO

#### Frontend

```bash
cd Armobot-control-system/frontend
npm install
npm run dev
```

The frontend runs on `http://localhost:5173` and:

- Proxies `/pico/*` requests to the Pico W's HTTP server (`192.168.4.1:80`) via Vite's proxy
- Connects to the backend's Socket.IO for real-time position and gripper state updates

### 4. Connect & Control

1. Connect your PC to the **`RoboticArm_AP`** Wi-Fi network (password: `12345678`).
2. Open `http://localhost:5173` in your browser.
3. Log in with the default credentials (`admin` / `admin123`).
4. Confirm the Wi-Fi connection on the setup screen.
5. Start controlling the arm!

---

## 🎮 Features

### Control Panel

- **Angle Input**: Type a specific degree value and hit "Apply" to move any joint to an exact position.
- **Jog Buttons**: Press and hold on-screen buttons to smoothly move a joint in real-time. Release to stop. Uses WebSocket → TCP for low-latency control.
- **Keyboard Jog**: Use **arrow keys** to jog the selected axis — select an axis card first, then use ← → for Joint 1 (Base), ↑ ↓ for Joint 2 and 3.
- **Dynamic Range Limits**: Joint 2 and Joint 3 limits update dynamically based on each other's position using inverse kinematics, preventing self-collision.

### 3D Digital Twin Visualization

- **Real-Time Rendering**: View a live, interactive 3D model of the robotic arm using `three.js`.
- **Forward Kinematics (FK)**: The web UI calculates exact real-world Cartesian coordinates (O, 1J2, 2J3, E) in real-time based on live joint angles.
- **HUD & Telemetry**: Displays continuous X, Y, Z coordinates (in mm) for the end effector and joints, alongside a dynamic gripper state indicator.
- **Gripper Animation**: The 3D model's gripper opens and closes in real-time — even during autonomous Pick & Place sequences — via telemetry sync over the TCP bridge.
- **Corrected Coordinate System**: Accurate base rotation direction and physical box dimensions for a true-to-life representation.

### Pick & Place Panel

> Previously known as "Action Panel" — now split into dedicated **GripperPanel** and **PickAndPlacePanel** components.

- **Auto Mode**: Build a sequence of positions and run them fully autonomously with configurable loop count.
- **Manual Mode**: Step through positions one at a time using "Prev" / "Next" controls, or click any position card to jump directly to it.
- **Drag-and-Drop Sequencer**: Reorder positions dynamically with drag handles.
- **New Configuration**: Start fresh with a blank sequence at any time using the "New Configuration" option in the configuration selector.
- **Save & Load Configurations**: Save complex multi-step sequences to localStorage and reload them instantly.
- **Pause & Resume**: Safely pause the robot mid-sequence and resume when ready.
- **Loop Control**: Run the sequence a specific number of times or loop continuously.
- **Run HUD**: Live execution display showing current step, total steps, completed loops, and an animated progress bar.

### Gripper Panel

- **Open / Close**: Instantly toggle the gripper servo.
- **Calibrate**: Homes all three joints simultaneously by driving them to their limit switches, then returning to 0°.

### Admin Dashboard

- **User Management**: Create, update, and delete user accounts.
- **Role-based Access**: Only admins can access the admin panel.
- **User Metadata**: Store company, address, city, and country per user.
- **Tabbed Interface**: Switch between "Users", "Register", and "Update/Delete" tabs.

### UI & Theme

- **Steel & Fire Theme**: Industrial dark design using deep charcoal backgrounds, ARNOBOT crimson red (`#cc2e20`) as the primary accent, blue-steel highlights, and JetBrains Mono for all readouts.
- **ARNOBOT Logo**: Branding logo displayed in the header with a crimson drop-shadow glow.
- **Rajdhani + Inter + JetBrains Mono** typography for a premium industrial feel.

### Safety System

- **Emergency Stop**: Physical button instantly halts all motors. System blocks until released.
- **Limit Switches**: Hardware interrupts (ISR) immediately flag when a joint hits its mechanical limit. All movement is blocked until the next command clears the flag.
- **Kinematic Coupling**: `solve_d2()` and `solve_d3()` continuously recalculate safe ranges based on the arm's current configuration.

---

## 🌐 API Reference

### Pico W HTTP Endpoints (Port 80)

| Endpoint | Method | Description |
|---|---|---|
| `/stepper?motor=X&angle=Y` | GET | Move joint X to Y degrees |
| `/gripper?action=open\|close` | GET | Open or close gripper |
| `/calibrate` | GET | Home all joints |
| `/status` | GET | JSON: joint angles, distances, emergency state |
| `/get_range2` | GET | JSON: current min/max for Joint 2 |
| `/get_range3` | GET | JSON: current min/max for Joint 3 |

### Pico W TCP Bridge (Port 81)

| Command | Direction | Description |
|---|---|---|
| `START_{axis}_{dir}\n` | Client → Pico | Begin jogging axis (1–3), dir (0=neg, 1=pos) |
| `STOP\n` | Client → Pico | Stop jogging |
| `{"s1":..,"s2":..,"s3":..,"n2":..,"x2":..,"n3":..,"x3":..,"gripper_state":".."}` | Pico → Client | Real-time state broadcast (every ~100ms) |

> `gripper_state` is now included in the TCP telemetry stream so the 3D visualization can animate the gripper during autonomous sequences.

### Backend REST API (Port 3000)

| Endpoint | Method | Auth | Description |
|---|---|---|---|
| `/api/auth/login` | POST | No | Login → returns JWT token |
| `/api/users` | GET | Admin | List all users |
| `/api/users` | POST | Admin | Create user |
| `/api/users/:id` | PUT | Admin | Update user |
| `/api/users/:id` | DELETE | Admin | Delete user |

### Backend Socket.IO Events (Port 3000)

| Event | Direction | Description |
|---|---|---|
| `robot_state` | Server → Client | Real-time joint angles, ranges & gripper state |
| `jog_start` | Client → Server | Begin jogging `{ axis, dir }` |
| `jog_stop` | Client → Server | Stop jogging `{ axis, dir }` |
| `pnp_start` | Client → Server | Start Pick & Place sequence |
| `pnp_stop` | Client → Server | Stop sequence |
| `pnp_pause` | Client → Server | Pause sequence |
| `pnp_resume` | Client → Server | Resume paused sequence |
| `pnp_manual_step` | Client → Server | Step forward/back in manual mode |
| `pnp_manual_goto` | Client → Server | Jump to a specific step in manual mode |
| `pnp_status` | Server → Client | Running sequence state (step, loops, etc.) |

---

## 🔧 Tech Stack

### Firmware

![MicroPython](https://img.shields.io/badge/MicroPython-2B2728?style=for-the-badge&logo=micropython&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi%20Pico%20W-A22846?style=for-the-badge&logo=raspberrypi&logoColor=white)
[![micropython-stepper](https://img.shields.io/badge/micropython--stepper-Library-blue?style=for-the-badge&logo=github)](https://github.com/redoxcode/micropython-stepper)

### Backend

![Node.js](https://img.shields.io/badge/Node.js-339933?style=for-the-badge&logo=nodedotjs&logoColor=white)
![Express](https://img.shields.io/badge/Express-000000?style=for-the-badge&logo=express&logoColor=white)
![Socket.IO](https://img.shields.io/badge/Socket.IO-010101?style=for-the-badge&logo=socketdotio&logoColor=white)
![SQLite](https://img.shields.io/badge/SQLite-003B57?style=for-the-badge&logo=sqlite&logoColor=white)
![JWT](https://img.shields.io/badge/JWT-000000?style=for-the-badge&logo=jsonwebtokens&logoColor=white)

### Frontend

![React](https://img.shields.io/badge/React_19-61DAFB?style=for-the-badge&logo=react&logoColor=black)
![Vite](https://img.shields.io/badge/Vite-646CFF?style=for-the-badge&logo=vite&logoColor=white)
![React Router](https://img.shields.io/badge/React_Router_v7-CA4245?style=for-the-badge&logo=reactrouter&logoColor=white)
![Axios](https://img.shields.io/badge/Axios-5A29E4?style=for-the-badge&logo=axios&logoColor=white)
![Three.js](https://img.shields.io/badge/Three.js-000000?style=for-the-badge&logo=threedotjs&logoColor=white)

### Hardware

![Raspberry Pi](https://img.shields.io/badge/RP2040-A22846?style=for-the-badge&logo=raspberrypi&logoColor=white)
![NEMA 17](https://img.shields.io/badge/NEMA_17-Steppers-orange?style=for-the-badge)
![A4988](https://img.shields.io/badge/A4988%20/%20DRV8825-Drivers-green?style=for-the-badge)
![MG90S](https://img.shields.io/badge/MG90S-Servo-blue?style=for-the-badge)

---

## 📋 Changelog

### v2.0.0 — June 2026

**Frontend — UI Theme**
- Implemented full **Steel & Fire** industrial dark theme across all pages and components
- Added ARNOBOT branding logo to the header with crimson glow drop-shadow
- Introduced `Rajdhani`, `Inter`, and `JetBrains Mono` typography system
- Replaced all warm-brown / beige inline styles with CSS variables from the new design token system

**Frontend — Control Panel**
- Added **keyboard jog control** using arrow keys — select an axis card then use arrow keys to jog without touching the mouse
- Jog buttons now use the Steel & Fire themed `.btn-jog` CSS classes (crimson for forward, steel for backward) replacing the old green/red inline styles

**Frontend — Pick & Place**
- Split `ActionPanel.jsx` into `GripperPanel.jsx` (gripper + calibration) and `PickAndPlacePanel.jsx` (full sequencer)
- Added **Auto / Manual execution mode** toggle
- Added **Manual stepping controls** ("Prev", "Next") and click-to-goto on position cards
- Added **"New Configuration"** option to the configuration selector to start a fresh sequence
- Improved dynamic layout: Pick & Place panel auto-snaps to top during active sequences

**Frontend — 3D Visualization**
- Fixed coordinate system: corrected base rotation direction and physical box dimensions
- Gripper now animates in real-time during autonomous sequences via telemetry sync

**Backend**
- Added `pnp_manual_step` and `pnp_manual_goto` Socket.IO events for manual PnP control
- Disabled step timeouts during manual execution to allow indefinite waiting
- Updated `robotState` tracker to parse and forward `gripper_state` from Pico telemetry

**Firmware (`main.py`)**
- Added `gripper_state` to the real-time TCP telemetry JSON payload
- Fixed HTTP routing bug: inlined `limit_triggered` reset into the `/stepper` handler
- Cleaned up unused `is_motor_path` helper and `MOTOR_PATHS` constants
- Refined calibration values for more accurate homing
- Updated joint angle limits: Base max reduced from 160° → **155°**

**Tooling**
- Added `start_armobot.bat` — one-click Windows launcher that starts both servers and opens the browser automatically
- Added `.mp4` files to `.gitignore`
