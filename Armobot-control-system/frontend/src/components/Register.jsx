import React, { useState } from 'react';
import axios from 'axios';
import { useNavigate, Link } from 'react-router-dom';

function Register() {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const navigate = useNavigate();

  const handleSubmit = async (e) => {
    e.preventDefault();
    try {
      await axios.post('http://localhost:3000/api/auth/register', { username, password });
      alert('Registration successful. Please login.');
      navigate('/login');
    } catch (err) {
      setError(err.response?.data?.error || 'Registration failed');
    }
  };

  return (
    <div className="page" style={{ maxWidth: '400px', marginTop: '40px' }}>
      <div className="cp">
        <h2 className="pt">Register</h2>
        {error && <p style={{ color: 'red', fontSize: '0.9em', marginBottom: '10px' }}>{error}</p>}
        <form onSubmit={handleSubmit} style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
          <input 
            type="text" className="ti" placeholder="Username" 
            value={username} onChange={(e) => setUsername(e.target.value)} required 
          />
          <input 
            type="password" className="ti" placeholder="Password" 
            value={password} onChange={(e) => setPassword(e.target.value)} required 
          />
          <button type="submit" className="ap" style={{ marginTop: '10px' }}>Register</button>
        </form>
        <p style={{ marginTop: '15px', fontSize: '0.8em', textAlign: 'center' }}>
          Already have an account? <Link to="/login">Login here</Link>
        </p>
      </div>
    </div>
  );
}

export default Register;
