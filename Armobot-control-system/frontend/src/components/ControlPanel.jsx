import { useState, useEffect, useRef } from 'react';
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
          gripper_state: json.gripper_state !== undefined ? json.gripper_state : prev.gripper_state,
          dr: json.dr !== undefined ? json.dr : prev.dr,
          dl: json.dl !== undefined ? json.dl : prev.dl,
        }));
      } catch (e) {
        console.warn('Failed to parse robot_state payload:', e);
      }
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

  // ── Keyboard jog (arrow keys) ─────────────────────────────────────────────
  // selectedCard ref so the effect closure always sees the latest value
  const selectedCardRef = useRef(selectedCard);
  useEffect(() => { selectedCardRef.current = selectedCard; }, [selectedCard]);
  const activeBtnRef = useRef(activeBtn);
  useEffect(() => { activeBtnRef.current = activeBtn; }, [activeBtn]);

  useEffect(() => {
    // Axis-1: ArrowLeft / ArrowRight
    // Axis-2: ArrowUp (dir 0 = Up) / ArrowDown (dir 1 = Down)
    // Axis-3: ArrowUp (dir 1 = Up) / ArrowDown (dir 0 = Down)  — mirrors Axis-2 visually
    const KEY_MAP = {
      ArrowLeft:  { axis: 1, dir: 0 },
      ArrowRight: { axis: 1, dir: 1 },
      ArrowUp:    { axis: null, dirFor2: 0, dirFor3: 1 },
      ArrowDown:  { axis: null, dirFor2: 1, dirFor3: 0 },
    };

    const resolveKey = (key) => {
      const card = selectedCardRef.current;
      if (key === 'ArrowLeft'  && card === 1) return { axis: 1, dir: 0 };
      if (key === 'ArrowRight' && card === 1) return { axis: 1, dir: 1 };
      if (key === 'ArrowUp'    && card === 2) return { axis: 2, dir: 0 };
      if (key === 'ArrowDown'  && card === 2) return { axis: 2, dir: 1 };
      if (key === 'ArrowUp'    && card === 3) return { axis: 3, dir: 1 };
      if (key === 'ArrowDown'  && card === 3) return { axis: 3, dir: 0 };
      return null;
    };

    const isInputFocused = () => {
      const tag = document.activeElement?.tagName;
      return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT';
    };

    const onKeyDown = (e) => {
      if (isInputFocused()) return;
      const mapped = resolveKey(e.key);
      if (!mapped) return;
      e.preventDefault(); // stop page scroll
      if (activeBtnRef.current) return; // another jog already active
      const { axis, dir } = mapped;
      setActiveBtn(`${axis}-${dir}`);
      socket.emit('jog_start', { axis, dir });
    };

    const onKeyUp = (e) => {
      if (isInputFocused()) return;
      const mapped = resolveKey(e.key);
      if (!mapped) return;
      const { axis, dir } = mapped;
      if (activeBtnRef.current === `${axis}-${dir}`) {
        setActiveBtn(null);
        socket.emit('jog_stop', { axis, dir });
      }
    };

    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup',   onKeyUp);
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup',   onKeyUp);
    };
  }, []); // runs once — reads latest values via refs

  const getBtnClass = (axis, dir, type = dir) => {
    const key = `${axis}-${dir}`;
    const isActive = activeBtn === key;
    let cls = 'btn-jog ';
    // colorType 0 = bwd (steel), 1 = fwd (fire)
    cls += type === 0 ? 'bwd ' : 'fwd ';
    if (isActive) cls += 'active ';
    return cls;
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
    if (axis === 1) { min = -170; max = 150; }
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
              type="number" className="ti" placeholder="-170 to 150" min="-170" max="150" 
              value={inputs.i1} onChange={(e) => handleInputChange(1, e.target.value)}
              onKeyDown={(e) => e.key === 'Enter' && handleApply(1)}
            />
            <button className="ap" onClick={() => handleApply(1)}>Apply</button>
          </div>
          <div className="ri">Range: -170 to 150 deg</div>
          <div style={{ display: 'flex', gap: '10px', marginTop: '10px' }}>
            <button className={getBtnClass(1, 0)} disabled={activeBtn !== null && activeBtn !== '1-0'}
              onPointerDown={() => handleJogStart(1, 0)} onPointerUp={() => handleJogStop(1, 0)} onPointerLeave={() => handleJogStop(1, 0)}
            >&#9664; Left</button>
            <button className={getBtnClass(1, 1)} disabled={activeBtn !== null && activeBtn !== '1-1'}
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
            <button className={getBtnClass(2, 1, 0)} disabled={activeBtn !== null && activeBtn !== '2-1'}
              onPointerDown={() => handleJogStart(2, 1)} onPointerUp={() => handleJogStop(2, 1)} onPointerLeave={() => handleJogStop(2, 1)}
            >&#9660; Down</button>
            <button className={getBtnClass(2, 0, 1)} disabled={activeBtn !== null && activeBtn !== '2-0'}
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
            <button className={getBtnClass(3, 0)} disabled={activeBtn !== null && activeBtn !== '3-0'}
              onPointerDown={() => handleJogStart(3, 0)} onPointerUp={() => handleJogStop(3, 0)} onPointerLeave={() => handleJogStop(3, 0)}
            >&#9660; Down</button>
            <button className={getBtnClass(3, 1)} disabled={activeBtn !== null && activeBtn !== '3-1'}
              onPointerDown={() => handleJogStart(3, 1)} onPointerUp={() => handleJogStop(3, 1)} onPointerLeave={() => handleJogStop(3, 1)}
            >Up &#9650;</button>
          </div>
        </div>

      </div>
    </div>
  );
}

export default ControlPanel;
