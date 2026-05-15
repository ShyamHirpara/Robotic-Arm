import React, { useState, useRef, useEffect } from 'react';
import axios from 'axios';

const API_BASE = '/pico';

function ControlPanel({ state, setState }) {
  const [inputs, setInputs] = useState({ i1: '', i2: '', i3: '' });
  const [btnState, setBtnState] = useState({});
  const [selectedCard, setSelectedCard] = useState(1);
  const activeKeys = useRef(new Set());
  const holdTimers = useRef({});
  const isHolding = useRef({});

  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.target.tagName === 'INPUT') return;

      if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(e.key)) {
        e.preventDefault();
        
        if (activeKeys.current.has(e.key)) return;
        activeKeys.current.add(e.key);

        if (selectedCard === 1) {
          if (e.key === 'ArrowLeft') handleJogStart(1, 0);
          if (e.key === 'ArrowRight') handleJogStart(1, 1);
        } else if (selectedCard === 2) {
          if (e.key === 'ArrowDown') handleJogStart(2, 1);
          if (e.key === 'ArrowUp') handleJogStart(2, 0);
        } else if (selectedCard === 3) {
          if (e.key === 'ArrowDown') handleJogStart(3, 0);
          if (e.key === 'ArrowUp') handleJogStart(3, 1);
        }
      }
    };

    const handleKeyUp = (e) => {
      if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(e.key)) {
        e.preventDefault();
        activeKeys.current.delete(e.key);

        if (selectedCard === 1) {
          if (e.key === 'ArrowLeft') handleJogStop(1, 0);
          if (e.key === 'ArrowRight') handleJogStop(1, 1);
        } else if (selectedCard === 2) {
          if (e.key === 'ArrowDown') handleJogStop(2, 1);
          if (e.key === 'ArrowUp') handleJogStop(2, 0);
        } else if (selectedCard === 3) {
          if (e.key === 'ArrowDown') handleJogStop(3, 0);
          if (e.key === 'ArrowUp') handleJogStop(3, 1);
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [selectedCard]);

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
      const response = await axios.get(`${API_BASE}/stepper?motor=${axis}&angle=${val}`, { timeout: 5000 });
      if (typeof response.data === 'string' && response.data.includes('LIMIT TRIGGERED')) {
        setState(prev => ({ ...prev, limit_triggered: true }));
      }
      
      if (axis === 2 || axis === 3) {
        updatePositionsAndRanges();
      }
    } catch (error) {
      console.error('Error sending angle', error);
    }
  };

  const updatePositionsAndRanges = async () => {
    try {
      const stat = await axios.get(`${API_BASE}/status`);
      const r2 = await axios.get(`${API_BASE}/get_range2`);
      const r3 = await axios.get(`${API_BASE}/get_range3`);
      setState(prev => ({
        ...prev,
        s1: Math.round(stat.data.s1),
        s2: Math.round(stat.data.s2),
        s3: Math.round(stat.data.s3),
        min_s2: Math.round(r2.data.min),
        max_s2: Math.round(r2.data.max),
        min_s3: Math.round(r3.data.min),
        max_s3: Math.round(r3.data.max)
      }));
    } catch(err) {
      console.error('Error syncing data', err);
    }
  };

  const handleJogStart = (axis, dir) => {
    const key = `${axis}-${dir}`;
    setBtnState(prev => ({ ...prev, [key]: 'clicked' }));
    
    // Instantly step for a snappy response
    axios.get(`${API_BASE}/jog?motor=${axis}&dir=${dir}&type=step`).then(() => updatePositionsAndRanges());
    
    // Setup hold transition
    isHolding.current[axis] = false;
    clearTimeout(holdTimers.current[axis]);
    holdTimers.current[axis] = setTimeout(() => {
      isHolding.current[axis] = true;
      setBtnState(prev => ({ ...prev, [key]: 'holding' }));
      axios.get(`${API_BASE}/jog?motor=${axis}&dir=${dir}&type=start`);
    }, 400);
  };

  const handleJogStop = (axis, dir) => {
    const key = `${axis}-${dir}`;
    setBtnState(prev => ({ ...prev, [key]: 'idle' }));
    
    clearTimeout(holdTimers.current[axis]);
    if (isHolding.current[axis]) {
      axios.get(`${API_BASE}/jog?type=stop`).then(() => updatePositionsAndRanges());
      isHolding.current[axis] = false;
    }
  };

  const getBtnStyle = (axis, dir, colorType = dir) => {
    const key = `${axis}-${dir}`;
    const state = btnState[key] || 'idle';
    let bg = '';
    
    if (colorType === 0) {
      // Green palette for Left/Down
      if (state === 'idle') bg = '#4caf50';
      else if (state === 'clicked') bg = '#388e3c';
      else if (state === 'holding') bg = '#1b5e20';
    } else {
      // Red palette for Right/Up
      if (state === 'idle') bg = '#ef5350';
      else if (state === 'clicked') bg = '#c62828';
      else if (state === 'holding') bg = '#7f0000';
    }

    return {
      padding: '8px 12px', borderRadius: '6px', border: 'none', 
      background: bg, color: '#fff', fontWeight: 'bold', 
      cursor: 'pointer', flex: 1, touchAction: 'none',
      transition: 'background 0.1s'
    };
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
            <button style={getBtnStyle(1, 0)}
              onPointerDown={() => handleJogStart(1, 0)} onPointerUp={() => handleJogStop(1, 0)} onPointerLeave={() => handleJogStop(1, 0)}
            >&#9664; Left</button>
            <button style={getBtnStyle(1, 1)}
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
            <button style={getBtnStyle(2, 1, 0)}
              onPointerDown={() => handleJogStart(2, 1)} onPointerUp={() => handleJogStop(2, 1)} onPointerLeave={() => handleJogStop(2, 1)}
            >&#9660; Down</button>
            <button style={getBtnStyle(2, 0, 1)}
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
            <button style={getBtnStyle(3, 0)}
              onPointerDown={() => handleJogStart(3, 0)} onPointerUp={() => handleJogStop(3, 0)} onPointerLeave={() => handleJogStop(3, 0)}
            >&#9660; Down</button>
            <button style={getBtnStyle(3, 1)}
              onPointerDown={() => handleJogStart(3, 1)} onPointerUp={() => handleJogStop(3, 1)} onPointerLeave={() => handleJogStop(3, 1)}
            >Up &#9650;</button>
          </div>
        </div>

      </div>
    </div>
  );
}

export default ControlPanel;
