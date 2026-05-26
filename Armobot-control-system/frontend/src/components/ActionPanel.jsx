import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import { io } from 'socket.io-client';

const API_BASE = '/pico';
const socket = io('http://localhost:3000');

// Absolute joint limits (mirrors Pico firmware constants)
const JOINT_LIMITS = {
  s1: { min: -175, max: 175 },
  s2: { min: -20,  max: 90  },
  s3: { min: -85,  max: 85  },
};

// ── Position Card (with edit + drag handle) ───────────────────────────────────
function PositionCard({ pos, index, isActive, isRunning, onUpdate, onDelete, onDragStart, onDragOver, onDrop, onDragEnd, isDraggingOver }) {
  const [editing, setEditing] = useState(false);
  const [draft,   setDraft]   = useState({ ...pos });

  const startEdit = (e) => { e.stopPropagation(); setDraft({ ...pos }); setEditing(true); };
  const cancelEdit = () => setEditing(false);
  const saveEdit = () => { onUpdate(index, draft); setEditing(false); };

  const setDraftField = (field, val) => setDraft(prev => ({ ...prev, [field]: val }));

  return (
    <div
      className={`pnp-card${isActive ? ' pnp-card-active' : ''}${isDraggingOver ? ' pnp-card-drag-over' : ''}`}
      draggable={!isRunning && !editing}
      onDragStart={(e) => onDragStart(e, index)}
      onDragOver={(e)  => onDragOver(e, index)}
      onDrop={(e)      => onDrop(e, index)}
      onDragEnd={onDragEnd}
    >
      {/* Drag handle + order */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 4 }}>
        <div className="pnp-card-order">#{pos.order}</div>
        {!isRunning && (
          <span className="pnp-drag-handle" title="Drag to reorder" aria-hidden="true" />
        )}
      </div>

      {/* ── View mode ──────────────────────────────────────────────────────── */}
      {!editing ? (
        <>
          <div className="pnp-card-row">
            <span className="pnp-card-label">Gripper</span>
            <span className={`pnp-card-val pnp-gripper-${pos.gripper_state}`}>
              {pos.gripper_state === 'open' ? '🟢 Open' : '🔴 Closed'}
            </span>
          </div>
          <div className="pnp-card-row">
            <span className="pnp-card-label">Axis 1</span>
            <span className="pnp-card-val">{pos.s1}°</span>
          </div>
          <div className="pnp-card-row">
            <span className="pnp-card-label">Axis 2</span>
            <span className="pnp-card-val">{pos.s2}°</span>
          </div>
          <div className="pnp-card-row">
            <span className="pnp-card-label">Axis 3</span>
            <span className="pnp-card-val">{pos.s3}°</span>
          </div>
          {!isRunning && (
            <div style={{ display: 'flex', justifyContent: 'flex-end', marginTop: 6, gap: 6 }}>
              <button className="pnp-edit-btn" onClick={() => onDelete(index)} title="Delete position" style={{ color: '#ff6b6b' }}>🗑️</button>
              <button className="pnp-edit-btn" onClick={startEdit} title="Edit position">✏️</button>
            </div>
          )}
        </>
      ) : (
        /* ── Edit mode ──────────────────────────────────────────────────── */
        <>
          <div className="pnp-card-row" style={{ flexDirection: 'column', alignItems: 'flex-start', gap: 4 }}>
            <span className="pnp-card-label">Gripper</span>
            <select
              className="pnp-edit-select"
              value={draft.gripper_state}
              onChange={e => setDraftField('gripper_state', e.target.value)}
            >
              <option value="open">🟢 Open</option>
              <option value="close">🔴 Closed</option>
            </select>
          </div>

          {[
            { key: 's1', label: 'Axis 1', ...JOINT_LIMITS.s1 },
            { key: 's2', label: 'Axis 2', ...JOINT_LIMITS.s2 },
            { key: 's3', label: 'Axis 3', ...JOINT_LIMITS.s3 },
          ].map(ax => (
            <div key={ax.key} className="pnp-card-row" style={{ flexDirection: 'column', alignItems: 'flex-start', gap: 2 }}>
              <span className="pnp-card-label">{ax.label} <span style={{ color: '#aaa', fontWeight: 400 }}>({ax.min}° to {ax.max}°)</span></span>
              <input
                className="pnp-edit-input"
                type="number"
                min={ax.min}
                max={ax.max}
                value={draft[ax.key]}
                onChange={e => {
                  const v = parseFloat(e.target.value);
                  if (!isNaN(v)) setDraftField(ax.key, Math.max(ax.min, Math.min(ax.max, v)));
                }}
              />
            </div>
          ))}

          <div style={{ display: 'flex', gap: 6, marginTop: 8, justifyContent: 'flex-end' }}>
            <button className="pnp-edit-cancel-btn" onClick={cancelEdit}>✕</button>
            <button className="pnp-edit-save-btn" onClick={saveEdit}>✔</button>
          </div>
        </>
      )}
    </div>
  );
}


// ── Main ActionPanel ─────────────────────────────────────────────────────────
function ActionPanel({ state, setState }) {
  // ── Pick & Place state ────────────────────────────────────────────────────
  const [pnpExpanded, setPnpExpanded] = useState(false);
  const [positions,   setPositions]   = useState([]);
  const [isRunning,   setIsRunning]   = useState(false);
  const [isPaused,    setIsPaused]    = useState(false);
  const [runStats,    setRunStats]    = useState(null);
  const [savedNames,  setSavedNames]  = useState([]);
  const [loadedName,  setLoadedName]  = useState('');

  // Drag state
  const dragIndexRef     = useRef(null);
  const [dragOverIndex,  setDragOverIndex] = useState(null);

  // ── Gripper / Calibration ─────────────────────────────────────────────────
  const gCmd = (action) => {
    setState(prev => ({ ...prev, gripper_state: action }));
    axios.get(`${API_BASE}/gripper?action=${action}`).catch(console.error);
  };
  const cal = () => axios.get(`${API_BASE}/calibrate`).catch(console.error);

  // ── LocalStorage ──────────────────────────────────────────────────────────
  const refreshSavedNames = () => {
    setSavedNames(Object.keys(localStorage).filter(k => k.startsWith('pnp_')).map(k => k.slice(4)));
  };
  useEffect(() => { refreshSavedNames(); }, []);

  const handleSaveConfig = () => {
    const name = window.prompt('Enter a name for this configuration:');
    if (!name || !name.trim()) return;
    localStorage.setItem('pnp_' + name.trim(), JSON.stringify(positions));
    refreshSavedNames();
    alert(`Configuration "${name.trim()}" saved!`);
  };

  const handleLoadConfig = (name) => {
    if (!name) return;
    const raw = localStorage.getItem('pnp_' + name);
    if (raw) { setPositions(JSON.parse(raw)); setLoadedName(name); }
  };

  // ── Save current robot position ───────────────────────────────────────────
  const handleSavePosition = () => {
    setPositions(prev => [...prev, {
      order:         prev.length + 1,
      gripper_state: state.gripper_state,
      s1: state.s1,
      s2: state.s2,
      s3: state.s3,
    }]);
  };

  // ── Edit a card's values ──────────────────────────────────────────────────
  const handleUpdateCard = (index, draft) => {
    setPositions(prev => prev.map((p, i) => i === index ? { ...p, ...draft } : p));
  };

  const handleDeleteCard = (index) => {
    setPositions(prev => {
      const next = prev.filter((_, i) => i !== index);
      // Renumber orders
      return next.map((p, i) => ({ ...p, order: i + 1 }));
    });
  };

  // ── Drag-to-reorder ───────────────────────────────────────────────────────
  const handleDragStart = (e, index) => {
    dragIndexRef.current = index;
    e.dataTransfer.effectAllowed = 'move';
  };
  const handleDragOver = (e, index) => {
    e.preventDefault();
    e.dataTransfer.dropEffect = 'move';
    setDragOverIndex(index);
  };
  const handleDrop = (e, dropIndex) => {
    e.preventDefault();
    const dragIndex = dragIndexRef.current;
    if (dragIndex === null || dragIndex === dropIndex) { setDragOverIndex(null); return; }
    setPositions(prev => {
      const next = [...prev];
      const [moved] = next.splice(dragIndex, 1);
      next.splice(dropIndex, 0, moved);
      // Renumber orders
      return next.map((p, i) => ({ ...p, order: i + 1 }));
    });
    dragIndexRef.current = null;
    setDragOverIndex(null);
  };
  const handleDragEnd = () => { dragIndexRef.current = null; setDragOverIndex(null); };

  const handleDeleteAll = () => {
    if (window.confirm('Remove all saved positions?')) {
      if (loadedName) {
        localStorage.removeItem('pnp_' + loadedName);
        refreshSavedNames();
      }
      setPositions([]);
      setLoadedName('');
    }
  };

  // ── Run / Pause / Resume / Stop ───────────────────────────────────────────
  useEffect(() => {
    socket.on('pnp_status', (data) => {
      setRunStats(data);
      if (data.stopped) {
        setIsRunning(false);
        setIsPaused(false);
        setRunStats(null);
      } else {
        setIsPaused(!!data.paused);
      }
    });
    return () => socket.off('pnp_status');
  }, []);

  const handleRun = () => {
    if (positions.length < 2) return;
    setIsRunning(true);
    setIsPaused(false);
    setRunStats({ currentIdx: 0, completed: 0, remaining: positions.length, loops: 0, stopped: false, paused: false });
    socket.emit('start_pnp', { positions });
  };

  const handlePauseResume = () => {
    if (isPaused) {
      socket.emit('resume_pnp');
      setIsPaused(false);
    } else {
      socket.emit('pause_pnp');
      setIsPaused(true);
    }
  };

  const handleStop = () => {
    socket.emit('stop_pnp');
    setIsRunning(false);
    setIsPaused(false);
    setRunStats(null);
  };

  // ── Elapsed timer ─────────────────────────────────────────────────────────
  const timerRef     = useRef(null);
  const startTimeRef = useRef(null);
  const pausedAtRef  = useRef(0);
  const [elapsed, setElapsed] = useState(0);

  useEffect(() => {
    if (isRunning && !isPaused) {
      startTimeRef.current = Date.now() - pausedAtRef.current * 1000;
      timerRef.current = setInterval(() => {
        const secs = Math.floor((Date.now() - startTimeRef.current) / 1000);
        setElapsed(secs);
        pausedAtRef.current = secs;
      }, 1000);
    } else {
      clearInterval(timerRef.current);
    }
    if (!isRunning) { setElapsed(0); pausedAtRef.current = 0; }
    return () => clearInterval(timerRef.current);
  }, [isRunning, isPaused]);

  const fmtTime = (secs) => {
    const m = Math.floor(secs / 60).toString().padStart(2, '0');
    const s = (secs % 60).toString().padStart(2, '0');
    return `${m}:${s}`;
  };

  return (
    <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(200px, 1fr))', gap: '16px', marginBottom: '16px' }}>

      {/* ── Gripper ──────────────────────────────────────────────────────────── */}
      <div className="cp" style={{ marginBottom: 0 }}>
        <div className="pt">Gripper</div>
        <div className="gbtns">
          <button className="btn bopen"  onClick={() => gCmd('open')}  disabled={isRunning}>Open Gripper</button>
          <button className="btn bclose" onClick={() => gCmd('close')} disabled={isRunning}>Close Gripper</button>
        </div>
      </div>

      {/* ── Calibration ──────────────────────────────────────────────────────── */}
      <div className="cp" style={{ marginBottom: 0 }}>
        <div className="pt">Calibration</div>
        <div className="gbtns">
          <button className="btn bcal" onClick={cal} disabled={isRunning}>Calibrate</button>
        </div>
      </div>

      {/* ── Pick & Place ─────────────────────────────────────────────────────── */}
      <div
        className={`cp pnp-container${pnpExpanded ? ' pnp-expanded' : ''}`}
        style={{ marginBottom: 0, gridColumn: pnpExpanded ? '1 / -1' : 'auto' }}
      >
        {/* Header */}
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <div className="pt" style={{ marginBottom: 0, borderBottom: 'none', paddingBottom: 0 }}>
            Pick &amp; Place
          </div>
          <button
            id="pnp-toggle-btn"
            className="btn pnp-toggle-btn"
            onClick={() => setPnpExpanded(prev => !prev)}
            disabled={isRunning}
          >
            {pnpExpanded ? '▲ Collapse' : '▼ Configure'}
          </button>
        </div>

        {/* Expanded content */}
        {pnpExpanded && (
          <div className="pnp-body">

            {/* Load saved config */}
            {savedNames.length > 0 && (
              <div className="pnp-load-row">
                <label className="pnp-load-label">Load saved:</label>
                <select
                  id="pnp-load-select"
                  className="pnp-select"
                  value={loadedName}
                  onChange={e => handleLoadConfig(e.target.value)}
                  disabled={isRunning}
                >
                  <option value="">-- select --</option>
                  {savedNames.map(n => <option key={n} value={n}>{n}</option>)}
                </select>
              </div>
            )}

            {/* Save current position */}
            <button
              id="pnp-save-position-btn"
              className="btn pnp-save-pos-btn"
              onClick={handleSavePosition}
              disabled={isRunning}
            >
              + Save Current Position ({positions.length})
            </button>

            {/* Position cards with drag-to-reorder */}
            {positions.length > 0 && (
              <div className="pnp-cards-grid">
                {positions.map((pos, i) => (
                  <PositionCard
                    key={i}
                    pos={pos}
                    index={i}
                    isActive={runStats && runStats.currentIdx === i && isRunning}
                    isRunning={isRunning}
                    onUpdate={handleUpdateCard}
                    onDelete={handleDeleteCard}
                    onDragStart={handleDragStart}
                    onDragOver={handleDragOver}
                    onDrop={handleDrop}
                    onDragEnd={handleDragEnd}
                    isDraggingOver={dragOverIndex === i}
                  />
                ))}
              </div>
            )}

            {/* Action buttons */}
            {positions.length >= 2 && (
              <div className="pnp-actions">
                <button id="pnp-save-btn"   className="btn pnp-btn-save"   onClick={handleSaveConfig}  disabled={isRunning}>💾 Save</button>
                <button id="pnp-run-btn"    className="btn pnp-btn-run"    onClick={handleRun}         disabled={isRunning}>▶ Run</button>
                <button id="pnp-delete-btn" className="btn pnp-btn-delete" onClick={handleDeleteAll}   disabled={isRunning}>🗑 Delete</button>
                {isRunning && (
                  <>
                    <button
                      id="pnp-pause-btn"
                      className={`btn ${isPaused ? 'pnp-btn-resume' : 'pnp-btn-pause'}`}
                      onClick={handlePauseResume}
                    >
                      {isPaused ? '▶ Resume' : '⏸ Pause'}
                    </button>
                    <button id="pnp-stop-btn" className="btn pnp-btn-stop" onClick={handleStop}>⏹ Stop</button>
                  </>
                )}
              </div>
            )}

            {/* Live Run HUD */}
            {isRunning && runStats && (
              <div className={`pnp-hud${isPaused ? ' pnp-hud-paused' : ''}`}>
                <div className="pnp-hud-title">
                  {isPaused ? '⏸ Pick & Place Paused' : '🤖 Pick & Place Running'}
                </div>
                <div className="pnp-hud-grid">
                  <div className="pnp-hud-stat">
                    <div className="pnp-hud-val">{fmtTime(elapsed)}</div>
                    <div className="pnp-hud-lbl">Elapsed</div>
                  </div>
                  <div className="pnp-hud-stat">
                    <div className="pnp-hud-val pnp-hud-cur">#{runStats.currentIdx + 1}</div>
                    <div className="pnp-hud-lbl">Current Pos</div>
                  </div>
                  <div className="pnp-hud-stat">
                    <div className="pnp-hud-val pnp-hud-done">{runStats.completed}</div>
                    <div className="pnp-hud-lbl">Completed</div>
                  </div>
                  <div className="pnp-hud-stat">
                    <div className="pnp-hud-val">{runStats.remaining}</div>
                    <div className="pnp-hud-lbl">Remaining</div>
                  </div>
                  <div className="pnp-hud-stat">
                    <div className="pnp-hud-val pnp-hud-loops">{runStats.loops}</div>
                    <div className="pnp-hud-lbl">Loops Done</div>
                  </div>
                </div>
                <div className="pnp-progress-bar-bg">
                  <div
                    className="pnp-progress-bar-fill"
                    style={{ width: `${(runStats.currentIdx / positions.length) * 100}%` }}
                  />
                </div>
                <div className="pnp-progress-label">
                  Position {runStats.currentIdx + 1} of {positions.length}
                </div>
              </div>
            )}

          </div>
        )}
      </div>

    </div>
  );
}

export default ActionPanel;
