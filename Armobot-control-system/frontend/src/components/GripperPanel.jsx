
import axios from 'axios';

const API_BASE = '/pico';

function GripperPanel({ setState, isRunning }) {
  const gCmd = (action) => {
    setState(prev => ({ ...prev, gripper_state: action }));
    axios.get(`${API_BASE}/gripper?action=${action}`).catch(console.error);
  };
  const cal = () => axios.get(`${API_BASE}/calibrate`).catch(console.error);

  return (
    <div className="cp" style={{ marginBottom: 0 }}>
      <div className="pt">Gripper &amp; Calibration</div>
      <div style={{ display: 'flex', gap: '12px', flexWrap: 'wrap', alignItems: 'center' }}>
        <button className="btn bopen"  onClick={() => gCmd('open')}  disabled={isRunning}>🟢 Open Gripper</button>
        <button className="btn bclose" onClick={() => gCmd('close')} disabled={isRunning}>🔴 Close Gripper</button>
        <div style={{ width: '1px', height: '32px', background: 'rgba(255,255,255,0.15)', margin: '0 4px' }} />
        <button className="btn bcal"   onClick={cal}                  disabled={isRunning}>⚙️ Calibrate All Joints</button>
      </div>
    </div>
  );
}

export default GripperPanel;
