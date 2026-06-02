import React from 'react';

function ConnectionSetup({ onConnect }) {
  return (
    <main className="page" style={{ maxWidth: '600px', marginTop: '40px' }}>
      <div className="cp" style={{ textAlign: 'center' }}>
        <h2 className="pt" style={{ fontSize: '1.2em' }}>Connect to Robotic Arm</h2>
        <p style={{ marginBottom: '15px', color: 'var(--txt-muted)', fontSize: '0.9em' }}>
          Please connect your PC to the Raspberry Pi Pico's Access Point before proceeding.
        </p>
        <div style={{ background: 'var(--bg2)', border: '1px solid var(--border)', borderRadius: 'var(--radius-sm)', padding: '15px', marginBottom: '20px', textAlign: 'left' }}>
          <p style={{ color: 'var(--txt)' }}><strong style={{ color: 'var(--txt-muted)' }}>SSID:</strong> RoboticArm_AP</p>
          <p style={{ color: 'var(--txt)' }}><strong style={{ color: 'var(--txt-muted)' }}>Password:</strong> 12345678</p>
        </div>
        <button className="ap" onClick={onConnect} style={{ fontSize: '1em', padding: '10px 20px' }}>
          I am connected
        </button>
      </div>
    </main>
  );
}

export default ConnectionSetup;
