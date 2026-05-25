import React, { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import { io } from 'socket.io-client';

const API_BASE = '/pico';
const socket = io('http://localhost:3000');

function ControlPanel({ state, setState }) {
  const [inputs, setInputs] = useState({ i1: '', i2: '', i3: '' });
  const [selectedCard, setSelectedCard] = useState(1);
  const [activeBtn, setActiveBtn] = useState(null);

  useEffect(() => {
    socket.on('robot_state', (dataStr) => {
      try {
        const json = JSON.parse(dataStr);
        setState(prev => ({
          ...prev,
          s1:     json.s1  !== undefined ? json.s1  : prev.s1,
          s2:     json.s2  !== undefined ? json.s2  : prev.s2,
          s3:     json.s3  !== undefined ? json.s3  : prev.s3,
          // Range fields embedded in TCP state (n2/x2/n3/x3)
          min_s2: json.n2  !== undefined ? json.n2  : prev.min_s2,
          max_s2: json.x2  !== undefined ? json.x2  : prev.max_s2,
          min_s3: json.n3  !== undefined ? json.n3  : prev.min_s3,
          max_s3: json.x3  !== undefined ? json.x3  : prev.max_s3,
        }));
      } catch (e) { }
    });
    return () => socket.off('robot_state');
  }, [setState]);

  const handleJogStart = (axis, dir) => {
    if (activeBtn) return;
    setActiveBtn(`${axis}-${dir}`);
    socket.emit('jog_start', { axis, dir });
  };

  const handleJogStop = (axis, dir) => {
    if (activeBtn === `${axis}-${dir}`) {
      setActiveBtn(null);
      socket.emit('jog_stop', { axis, dir });
    }
  };

  const getBtnStyle = (axis, dir, colorType = dir) => {
    const key = `${axis}-${dir}`;
    const isActive = activeBtn === key;
    const isDisabled = activeBtn !== null && activeBtn !== key;
    
    let bg = '#ccc';
    if (!isDisabled) {
      if (colorType === 0) {
        bg = isActive ? '#1b5e20' : '#4caf50';
      } else {
        bg = isActive ? '#7f0000' : '#ef5350';
      }
    }

    return {
      padding: '8px 12px', borderRadius: '6px', border: 'none', 
      background: bg, color: '#fff', fontWeight: 'bold', 
      cursor: isDisabled ? 'not-allowed' : 'pointer', flex: 1, touchAction: 'none',
      transition: 'background 0.1s', opacity: isDisabled ? 0.5 : 1
    };
  };


  const handleInputChange = (axis, value) => {
    setInputs(prev => ({ ...prev, [`i${axis}`]: value }));
  };

  const handleApply = async (axis) => {
    const val = parseFloat(inputs[`i${axis}`]);
    if (isNaN(val)) {
      alert('Enter valid number');
      return;
    }
    await sendAngle(axis, val);
    setInputs(prev => ({ ...prev, [`i${axis}`]: '' }));
  };

  const sendAngle = async (axis, val) => {
    let min, max;
    if (axis === 1) { min = -175; max = 175; }
    if (axis === 2) { min = state.min_s2; max = state.max_s2; }
    if (axis === 3) { min = state.min_s3; max = state.max_s3; }

    if (val < min || val > max) {
      alert(`Between ${min} and ${max}`);
      return;
    }

    setState(prev => ({ ...prev, [`s${axis}`]: Math.round(val) }));

    try {
      await axios.get(`${API_BASE}/stepper?motor=${axis}&angle=${val}`, { timeout: 5000 });
      // Ranges update automatically via TCP robot_state broadcast (n2/x2/n3/x3)
    } catch (error) {
      console.error('Error sending angle', error);
    }
  };

  const getCardStyle = (axis) => ({
    border: selectedCard === axis ? '2px solid #d4a96a' : '1px solid #e8d5b0',
    boxShadow: selectedCard === axis ? '0 0 15px rgba(212, 169, 106, 0.5)' : 'none',
    cursor: 'pointer',
    transition: 'all 0.2s ease',
    transform: selectedCard === axis ? 'scale(1.02)' : 'scale(1)',
    zIndex: selectedCard === axis ? 2 : 1,
    position: 'relative'
  });
  
  return (
    <div className="cp">
      <div className="pt">Stepper Motor Control</div>
      <div className="agrid">

        {/* Axis 1 */}
        <div className="ac" style={getCardStyle(1)} onClick={() => setSelectedCard(1)}>
          <div className="at">Axis 1 Base <span className="acur">{state.s1}deg</span></div>
          <div className="ir">
            <input 
              type="number" className="ti" placeholder="-175 to 175" min="-175" max="175" 
              value={inputs.i1} onChange={(e) => handleInputChange(1, e.target.value)}
              onKeyDown={(e) => e.key === 'Enter' && handleApply(1)}
            />
            <button className="ap" onClick={() => handleApply(1)}>Apply</button>
          </div>
          <div className="ri">Range: -175 to 175 deg</div>
          <div style={{ display: 'flex', gap: '10px', marginTop: '10px' }}>
            <button style={getBtnStyle(1, 0)} disabled={activeBtn !== null && activeBtn !== '1-0'}
              onPointerDown={() => handleJogStart(1, 0)} onPointerUp={() => handleJogStop(1, 0)} onPointerLeave={() => handleJogStop(1, 0)}
            >&#9664; Left</button>
            <button style={getBtnStyle(1, 1)} disabled={activeBtn !== null && activeBtn !== '1-1'}
              onPointerDown={() => handleJogStart(1, 1)} onPointerUp={() => handleJogStop(1, 1)} onPointerLeave={() => handleJogStop(1, 1)}
            >Right &#9654;</button>
          </div>
        </div>

        {/* Axis 2 */}
        <div className="ac" style={getCardStyle(2)} onClick={() => setSelectedCard(2)}>
          <div className="at">Axis 2 Shoulder <span className="acur">{state.s2}deg</span></div>
          <div className="ir">
            <input 
              type="number" className="ti" placeholder="angle" min={state.min_s2} max={state.max_s2} 
              value={inputs.i2} onChange={(e) => handleInputChange(2, e.target.value)}
              onKeyDown={(e) => e.key === 'Enter' && handleApply(2)}
            />
            <button className="ap" onClick={() => handleApply(2)}>Apply</button>
          </div>
          <div className="ri">Range: {state.min_s2} to {state.max_s2} deg</div>
          <div style={{ display: 'flex', gap: '10px', marginTop: '10px' }}>
            <button style={getBtnStyle(2, 1, 0)} disabled={activeBtn !== null && activeBtn !== '2-1'}
              onPointerDown={() => handleJogStart(2, 1)} onPointerUp={() => handleJogStop(2, 1)} onPointerLeave={() => handleJogStop(2, 1)}
            >&#9660; Down</button>
            <button style={getBtnStyle(2, 0, 1)} disabled={activeBtn !== null && activeBtn !== '2-0'}
              onPointerDown={() => handleJogStart(2, 0)} onPointerUp={() => handleJogStop(2, 0)} onPointerLeave={() => handleJogStop(2, 0)}
            >Up &#9650;</button>
          </div>
        </div>

        {/* Axis 3 */}
        <div className="ac" style={getCardStyle(3)} onClick={() => setSelectedCard(3)}>
          <div className="at">Axis 3 Elbow <span className="acur">{state.s3}deg</span></div>
          <div className="ir">
            <input 
              type="number" className="ti" placeholder="angle" min={state.min_s3} max={state.max_s3} 
              value={inputs.i3} onChange={(e) => handleInputChange(3, e.target.value)}
              onKeyDown={(e) => e.key === 'Enter' && handleApply(3)}
            />
            <button className="ap" onClick={() => handleApply(3)}>Apply</button>
          </div>
          <div className="ri">Range: {state.min_s3} to {state.max_s3} deg</div>
          <div style={{ display: 'flex', gap: '10px', marginTop: '10px' }}>
            <button style={getBtnStyle(3, 0)} disabled={activeBtn !== null && activeBtn !== '3-0'}
              onPointerDown={() => handleJogStart(3, 0)} onPointerUp={() => handleJogStop(3, 0)} onPointerLeave={() => handleJogStop(3, 0)}
            >&#9660; Down</button>
            <button style={getBtnStyle(3, 1)} disabled={activeBtn !== null && activeBtn !== '3-1'}
              onPointerDown={() => handleJogStart(3, 1)} onPointerUp={() => handleJogStop(3, 1)} onPointerLeave={() => handleJogStop(3, 1)}
            >Up &#9650;</button>
          </div>
        </div>

      </div>
    </div>
  );
}

export default ControlPanel;
