import React from 'react';

function Dashboard({ state }) {
  const gIcon = state.gripper_state === 'open' ? 'OPEN' : 'CLOSED';
  const gColor = state.gripper_state === 'open' ? '#2e7d32' : '#b71c1c';

  return (
    <div className="dgrid">
      <div className="dc">
        <div className="dcl">Axis 1 - Base</div>
        <div className="dcv">{state.s1}</div>
      </div>
      <div className="dc">
        <div className="dcl">Axis 2 - Shoulder</div>
        <div className="dcv">{state.s2}</div>
      </div>
      <div className="dc">
        <div className="dcl">Axis 3 - Elbow</div>
        <div className="dcv">{state.s3}</div>
      </div>
      <div className="dc gc">
        <div className="dcl">Gripper</div>
        <div className="gv" style={{ color: gColor }}>{gIcon}</div>
      </div>
    </div>
  );
}

export default Dashboard;
