function Dashboard({ state }) {
  const gIcon = state.gripper_state === 'open' ? 'OPEN' : 'CLOSED';
  const gColor = state.gripper_state === 'open' ? '#44bb66' : '#ff6666';

  return (
    <div className="dgrid">
      {/* Row 1 — Joint angles */}
      <div className="dc">
        <div className="dcl">Axis 1 — Base</div>
        <div className="dcv">{state.s1}<span style={{fontSize:'0.4em', color:'var(--txt-muted)', marginLeft:'4px'}}>deg</span></div>
      </div>
      <div className="dc">
        <div className="dcl">Axis 2 — Shoulder</div>
        <div className="dcv">{state.s2}<span style={{fontSize:'0.4em', color:'var(--txt-muted)', marginLeft:'4px'}}>deg</span></div>
      </div>
      <div className="dc">
        <div className="dcl">Axis 3 — Elbow</div>
        <div className="dcv">{state.s3}<span style={{fontSize:'0.4em', color:'var(--txt-muted)', marginLeft:'4px'}}>deg</span></div>
      </div>

      {/* Row 2 — Sensors & Gripper */}
      <div className="dc">
        <div className="dcl">Left Sensor</div>
        <div className="dcv">{state.dl}<span style={{fontSize:'0.4em', color:'var(--txt-muted)', marginLeft:'4px'}}>cm</span></div>
      </div>
      <div className="dc gc">
        <div className="dcl">Gripper</div>
        <div className="gv" style={{ color: gColor }}>{gIcon}</div>
      </div>
      <div className="dc">
        <div className="dcl">Right Sensor</div>
        <div className="dcv">{state.dr}<span style={{fontSize:'0.4em', color:'var(--txt-muted)', marginLeft:'4px'}}>cm</span></div>
      </div>
    </div>
  );
}

export default Dashboard;
