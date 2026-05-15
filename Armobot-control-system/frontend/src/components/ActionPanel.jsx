import React from 'react';
import axios from 'axios';

const API_BASE = '/pico';

function ActionPanel({ state, setState }) {
  
  const gCmd = (action) => {
    setState(prev => ({ ...prev, gripper_state: action }));
    // Assuming there might be an endpoint for gripper, or just simulating UI
    axios.get(`${API_BASE}/gripper?action=${action}`).catch(console.error);
  };

  const cal = () => {
    // alert('Calibration preview — runs on real device only.');
    axios.get(`${API_BASE}/calibrate`).catch(console.error);
  };

  const executePickPlace = async () => {
    if (window.confirm('Start Pick and Place?')) {
      try {
        const response = await axios.get(`${API_BASE}/pickplace`);
        alert(response.data);
      } catch (err) {
        console.error('Pick and Place error', err);
        alert('Network error executing Pick & Place');
      }
    }
  };

  return (
    <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(200px, 1fr))', gap: '16px', marginBottom: '16px' }}>
      
      <div className="cp" style={{ marginBottom: 0 }}>
        <div className="pt">Gripper</div>
        <div className="gbtns">
          <button className="btn bopen"  onClick={() => gCmd('open')}>Open Gripper</button>
          <button className="btn bclose" onClick={() => gCmd('close')}>Close Gripper</button>
        </div>
      </div>
      
      <div className="cp" style={{ marginBottom: 0 }}>
        <div className="pt">Calibration</div>
        <div className="gbtns">
          <button className="btn bcal" onClick={cal}>Calibrate</button>
        </div>
      </div>

      <div className="cp" style={{ marginBottom: 0 }}>
        <div className="pt">Pick &amp; Place</div>
        <div className="gbtns">
          <button className="btn bcal" onClick={executePickPlace}>Execute</button>
        </div>
      </div>
      
    </div>
  );
}

export default ActionPanel;
