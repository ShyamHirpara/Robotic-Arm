import React, { useState, useEffect } from 'react';
import { BrowserRouter as Router, Routes, Route, Navigate, Link } from 'react-router-dom';
import Dashboard from './components/Dashboard';
import ControlPanel from './components/ControlPanel';
import GripperPanel from './components/GripperPanel';
import PickAndPlacePanel from './components/PickAndPlacePanel';
import RobotVisualization from './components/RobotVisualization';
import ConnectionSetup from './components/ConnectionSetup';
import Login from './components/Login';
import Register from './components/Register';
import AdminDashboard from './components/AdminDashboard';
import './index.css';

function App() {
  const [auth, setAuth] = useState(null);
  const [isConnected, setIsConnected] = useState(false);
  const [isPnpRunning, setIsPnpRunning] = useState(false);
  const [systemState, setSystemState] = useState({
    s1: 0, s2: 0, s3: 0,
    min_s2: -20, max_s2: 70, min_s3: -90, max_s3: 90,
    gripper_state: 'closed', limit_triggered: false
  });

  useEffect(() => {
    const token = localStorage.getItem('token');
    const user = localStorage.getItem('user');
    if (token && user) {
      setAuth({ token, user: JSON.parse(user) });
    }
  }, []);

  const handleLogout = () => {
    localStorage.removeItem('token');
    localStorage.removeItem('user');
    setAuth(null);
  };

  return (
    <Router>
      <header className="hdr">
        <div>
          <h1>ARMOBOT</h1>
          <p>3-Axis Robotic Arm Control</p>
        </div>
        <div>
          {auth ? (
            <>
              {auth.user.role === 'admin' && (
                <Link to="/admin" style={{ marginRight: '15px', color: '#5c3d11', fontWeight: 'bold', textDecoration: 'none' }}>Admin</Link>
              )}
              <Link to="/" style={{ marginRight: '15px', color: '#5c3d11', fontWeight: 'bold', textDecoration: 'none' }}>Control</Link>
              <button onClick={handleLogout} className="btn bclose" style={{ padding: '5px 10px' }}>Logout</button>
            </>
          ) : (
            <Link to="/login" className="btn" style={{ background: '#d4a96a', color: '#fff', textDecoration: 'none' }}>Login</Link>
          )}
        </div>
      </header>

      <Routes>
        <Route path="/login" element={!auth ? <Login setAuth={setAuth} /> : <Navigate to="/" />} />
        <Route path="/admin" element={
          auth && auth.user.role === 'admin' ? <AdminDashboard auth={auth} /> : <Navigate to="/login" />
        } />
        <Route path="/" element={
          !auth ? <Navigate to="/login" /> :
          !isConnected ? <ConnectionSetup onConnect={() => setIsConnected(true)} /> :
          (
            <>
              <div className="sbar">
                <div className="dot"></div>
                <span className="otxt">SYSTEM ONLINE</span>
              </div>

              <main className="page">
                <p className="slbl">Live Dashboard</p>
                <Dashboard state={systemState} />
                <p className="slbl">Visualization</p>
                <div className="cp visualization-container">
                  <RobotVisualization state={systemState} />
                </div>
                
                <p className="slbl">Control Panel</p>
                <div style={{ display: 'flex', flexDirection: 'column', gap: '16px' }}>
                  <div style={{ order: isPnpRunning ? 2 : 1 }}>
                    <ControlPanel state={systemState} setState={setSystemState} />
                  </div>
                  <div style={{ order: isPnpRunning ? 3 : 2 }}>
                    <GripperPanel state={systemState} setState={setSystemState} isRunning={isPnpRunning} />
                  </div>
                  <div style={{ order: isPnpRunning ? 1 : 3 }}>
                    <PickAndPlacePanel state={systemState} onIsRunningChange={setIsPnpRunning} />
                  </div>
                </div>
              </main>
            </>
          )
        } />
      </Routes>
    </Router>
  );
}

export default App;
