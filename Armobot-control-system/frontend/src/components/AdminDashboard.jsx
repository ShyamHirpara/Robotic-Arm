import React, { useEffect, useState } from 'react';
import axios from 'axios';

function AdminDashboard({ auth }) {
  const [users, setUsers] = useState([]);
  const [error, setError] = useState('');
  const [activeTab, setActiveTab] = useState('users'); // 'users', 'register', 'update'
  
  // Form State
  const [formData, setFormData] = useState({ 
    id: null, username: '', password: '', role: 'user', 
    company: '', address: '', city: '', country: '' 
  });
  
  const [editData, setEditData] = useState({ 
    id: null, username: '', password: '', role: 'user', 
    company: '', address: '', city: '', country: '' 
  });
  const [isEditingId, setIsEditingId] = useState(null);

  useEffect(() => {
    fetchUsers();
  }, []);

  const fetchUsers = async () => {
    try {
      const res = await axios.get('http://localhost:3000/api/users', {
        headers: { Authorization: `Bearer ${auth.token}` }
      });
      setUsers(res.data);
    } catch (err) {
      setError(err.response?.data?.error || 'Failed to fetch users');
    }
  };

  const handleDelete = async (id) => {
    if (window.confirm('Delete this user? This cannot be undone.')) {
      try {
        await axios.delete(`http://localhost:3000/api/users/${id}`, {
          headers: { Authorization: `Bearer ${auth.token}` }
        });
        fetchUsers();
        if (isEditingId === id) setIsEditingId(null);
      } catch (err) {
        alert('Failed to delete user');
      }
    }
  };

  const handleRegister = async (e) => {
    e.preventDefault();
    try {
      await axios.post('http://localhost:3000/api/users', formData, {
        headers: { Authorization: `Bearer ${auth.token}` }
      });
      alert('User created successfully');
      setFormData({ id: null, username: '', password: '', role: 'user', company: '', address: '', city: '', country: '' });
      fetchUsers();
      setActiveTab('users');
    } catch (err) {
      alert(err.response?.data?.error || 'Registration failed');
    }
  };

  const handleUpdate = async (e) => {
    e.preventDefault();
    try {
      await axios.put(`http://localhost:3000/api/users/${editData.id}`, editData, {
        headers: { Authorization: `Bearer ${auth.token}` }
      });
      alert('User updated successfully');
      setIsEditingId(null);
      fetchUsers();
    } catch (err) {
      alert(err.response?.data?.error || 'Update failed');
    }
  };

  const startEdit = (u) => {
    setEditData({ 
      id: u.id, username: u.username, password: '', role: u.role,
      company: u.company || '', address: u.address || '', 
      city: u.city || '', country: u.country || '' 
    });
    setIsEditingId(u.id);
  };

  const tabButtonStyle = (tabName) => ({
    padding: '8px 16px',
    borderRadius: '8px',
    border: '1px solid #d4a96a',
    background: activeTab === tabName ? '#d4a96a' : 'transparent',
    color: activeTab === tabName ? '#fff' : '#d4a96a',
    cursor: 'pointer',
    fontWeight: 'bold',
    transition: 'all 0.2s'
  });

  return (
    <div className="page">
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '20px' }}>
        <h2 className="slbl" style={{ fontSize: '1.2em', margin: 0 }}>Admin Dashboard</h2>
        <div style={{ display: 'flex', gap: '10px' }}>
          <button style={tabButtonStyle('users')} onClick={() => setActiveTab('users')}>👥 Users</button>
          <button style={tabButtonStyle('register')} onClick={() => setActiveTab('register')}>➕ Register</button>
          <button style={tabButtonStyle('update')} onClick={() => setActiveTab('update')}>✏️ Update</button>
        </div>
      </div>
      
      {error && <p style={{ color: 'red' }}>{error}</p>}

      {/* ===== USERS TAB ===== */}
      {activeTab === 'users' && (
        <div className="cp" style={{ animation: 'fadeIn 0.3s' }}>
          <h3 className="pt" style={{ marginBottom: '5px' }}>All Users</h3>
          <p style={{ fontSize: '0.85em', color: '#888', marginBottom: '20px' }}>
            ● {users.length} user{users.length !== 1 ? 's' : ''} registered
          </p>
          
          <table style={{ width: '100%', borderCollapse: 'collapse', textAlign: 'left', fontSize: '0.9em' }}>
            <thead>
              <tr style={{ borderBottom: '1px solid #e8d5b0', color: '#5c3d11' }}>
                <th style={{ padding: '10px' }}>ID</th>
                <th style={{ padding: '10px' }}>Username</th>
                <th style={{ padding: '10px' }}>Company</th>
                <th style={{ padding: '10px' }}>Address</th>
                <th style={{ padding: '10px' }}>City</th>
                <th style={{ padding: '10px' }}>Country</th>
                <th style={{ padding: '10px' }}>Registered On</th>
              </tr>
            </thead>
            <tbody>
              {users.map(u => (
                <tr key={u.id} style={{ borderBottom: '1px solid #fdf6ec' }}>
                  <td style={{ padding: '10px' }}>#{u.id}</td>
                  <td style={{ padding: '10px', fontWeight: 'bold' }}>{u.username}</td>
                  <td style={{ padding: '10px', color: '#666' }}>{u.company || '—'}</td>
                  <td style={{ padding: '10px', color: '#666' }}>{u.address || '—'}</td>
                  <td style={{ padding: '10px', color: '#666' }}>{u.city || '—'}</td>
                  <td style={{ padding: '10px', color: '#666' }}>{u.country || '—'}</td>
                  <td style={{ padding: '10px', color: '#666' }}>{new Date(u.created_at).toLocaleDateString() || '—'}</td>
                </tr>
              ))}
              {users.length === 0 && (
                <tr><td colSpan="7" style={{ textAlign: 'center', padding: '30px', color: '#888' }}>🔍 No users found.</td></tr>
              )}
            </tbody>
          </table>
        </div>
      )}

      {/* ===== REGISTER TAB ===== */}
      {activeTab === 'register' && (
        <div className="cp" style={{ animation: 'fadeIn 0.3s' }}>
          <h3 className="pt" style={{ marginBottom: '20px' }}>Register New User</h3>
          <form onSubmit={handleRegister} style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '15px' }}>
            <div style={{ display: 'flex', flexDirection: 'column', gap: '5px' }}>
              <label style={{ fontSize: '0.85em', fontWeight: 'bold', color: '#5c3d11' }}>Username</label>
              <input type="text" className="ti" value={formData.username} onChange={e => setFormData({...formData, username: e.target.value})} required />
            </div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: '5px' }}>
              <label style={{ fontSize: '0.85em', fontWeight: 'bold', color: '#5c3d11' }}>Password</label>
              <input type="password" className="ti" value={formData.password} onChange={e => setFormData({...formData, password: e.target.value})} required />
            </div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: '5px', gridColumn: '1 / -1' }}>
              <label style={{ fontSize: '0.85em', fontWeight: 'bold', color: '#5c3d11' }}>Company Name</label>
              <input type="text" className="ti" value={formData.company} onChange={e => setFormData({...formData, company: e.target.value})} />
            </div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: '5px', gridColumn: '1 / -1' }}>
              <label style={{ fontSize: '0.85em', fontWeight: 'bold', color: '#5c3d11' }}>Address</label>
              <input type="text" className="ti" value={formData.address} onChange={e => setFormData({...formData, address: e.target.value})} />
            </div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: '5px' }}>
              <label style={{ fontSize: '0.85em', fontWeight: 'bold', color: '#5c3d11' }}>City</label>
              <input type="text" className="ti" value={formData.city} onChange={e => setFormData({...formData, city: e.target.value})} />
            </div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: '5px' }}>
              <label style={{ fontSize: '0.85em', fontWeight: 'bold', color: '#5c3d11' }}>Country</label>
              <input type="text" className="ti" value={formData.country} onChange={e => setFormData({...formData, country: e.target.value})} />
            </div>
            <div style={{ gridColumn: '1 / -1', marginTop: '10px' }}>
              <button type="submit" className="ap" style={{ width: '100%', padding: '12px' }}>➕ Create User Account</button>
            </div>
          </form>
        </div>
      )}

      {/* ===== UPDATE TAB ===== */}
      {activeTab === 'update' && (
        <div className="cp" style={{ animation: 'fadeIn 0.3s' }}>
          <h3 className="pt" style={{ marginBottom: '20px' }}>Update / Delete User</h3>
          <div style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
            {users.map(u => (
              <div key={u.id}>
                <div style={{ 
                  display: 'flex', justifyContent: 'space-between', alignItems: 'center', 
                  padding: '12px 16px', background: '#fdf6ec', borderRadius: '8px', border: '1px solid #e8d5b0',
                  boxShadow: isEditingId === u.id ? '0 0 0 2px #d4a96a' : 'none'
                }}>
                  <div>
                    <div style={{ fontWeight: 'bold', fontSize: '1.05em' }}>{u.username}</div>
                    <div style={{ fontSize: '0.8em', color: '#888' }}>
                      ID #{u.id} {u.company ? `· ${u.company}` : ''} {u.city ? `· ${u.city}` : ''} {u.country ? `, ${u.country}` : ''}
                    </div>
                  </div>
                  <div style={{ display: 'flex', gap: '8px' }}>
                    <button className="btn bopen" style={{ padding: '6px 12px' }} onClick={() => startEdit(u)}>✏️ Edit</button>
                    {u.role !== 'admin' && (
                      <button className="btn bclose" style={{ padding: '6px 12px' }} onClick={() => handleDelete(u.id)}>🗑️ Delete</button>
                    )}
                  </div>
                </div>

                {isEditingId === u.id && (
                  <form onSubmit={handleUpdate} style={{ 
                    marginTop: '10px', padding: '15px', background: '#fff', borderRadius: '8px', border: '1px solid #d4a96a',
                    display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '10px'
                  }}>
                    <div style={{ gridColumn: '1 / -1', fontWeight: 'bold', color: '#d4a96a', marginBottom: '5px' }}>Editing: {u.username}</div>
                    
                    <input type="text" className="ti" placeholder="Username" value={editData.username} onChange={e => setEditData({...editData, username: e.target.value})} required />
                    <input type="password" className="ti" placeholder="New Password (leave blank to keep)" value={editData.password} onChange={e => setEditData({...editData, password: e.target.value})} />
                    
                    <input type="text" className="ti" style={{ gridColumn: '1 / -1' }} placeholder="Company Name" value={editData.company} onChange={e => setEditData({...editData, company: e.target.value})} />
                    <input type="text" className="ti" style={{ gridColumn: '1 / -1' }} placeholder="Address" value={editData.address} onChange={e => setEditData({...editData, address: e.target.value})} />
                    
                    <input type="text" className="ti" placeholder="City" value={editData.city} onChange={e => setEditData({...editData, city: e.target.value})} />
                    <input type="text" className="ti" placeholder="Country" value={editData.country} onChange={e => setEditData({...editData, country: e.target.value})} />

                    <div style={{ gridColumn: '1 / -1', display: 'flex', gap: '10px', marginTop: '10px' }}>
                      <button type="submit" className="ap" style={{ flex: 1 }}>Save Changes</button>
                      <button type="button" className="btn bclose" style={{ flex: 1 }} onClick={() => setIsEditingId(null)}>Cancel</button>
                    </div>
                  </form>
                )}
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
}

export default AdminDashboard;
