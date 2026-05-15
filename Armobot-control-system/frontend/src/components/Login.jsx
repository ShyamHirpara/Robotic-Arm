import React, { useState } from 'react';
import axios from 'axios';
import { useNavigate, Link } from 'react-router-dom';

function Login({ setAuth }) {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const navigate = useNavigate();

  const handleSubmit = async (e) => {
    e.preventDefault();
    try {
      const res = await axios.post('http://localhost:3000/api/auth/login', { username, password });
      localStorage.setItem('token', res.data.token);
      localStorage.setItem('user', JSON.stringify(res.data.user));
      setAuth({ token: res.data.token, user: res.data.user });
      navigate('/');
    } catch (err) {
      setError(err.response?.data?.error || 'Login failed');
    }
  };

  return (
    <div className="page" style={{ maxWidth: '400px', marginTop: '40px' }}>
      <div className="cp">
        <h2 className="pt">Login</h2>
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
          <button type="submit" className="ap" style={{ marginTop: '10px' }}>Login</button>
        </form>
      </div>
    </div>
  );
}

export default Login;
