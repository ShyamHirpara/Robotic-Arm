import React, { useState, useRef, useEffect } from 'react';
import axios from 'axios';

const API_BASE = '/pico';

function ControlPanel({ state, setState }) {
  const [inputs, setInputs] = useState({ i1: '', i2: '', i3: '' });
  const [selectedCard, setSelectedCard] = useState(1);


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
        </div>

      </div>
    </div>
  );
}

export default ControlPanel;
