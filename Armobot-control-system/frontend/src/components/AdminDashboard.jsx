import React, { useEffect, useState } from 'react';
import axios from 'axios';

function AdminDashboard({ auth }) {
  const [users, setUsers] = useState([]);
  const [error, setError] = useState('');
  
  // Form State
  const [formData, setFormData] = useState({ id: null, username: '', password: '', role: 'user' });
  const [isEditing, setIsEditing] = useState(false);

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
    if (window.confirm('Delete this user?')) {
      try {
        await axios.delete(`http://localhost:3000/api/users/${id}`, {
          headers: { Authorization: `Bearer ${auth.token}` }
        });
        fetchUsers();
      } catch (err) {
        alert('Failed to delete user');
      }
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    try {
      if (isEditing) {
        await axios.put(`http://localhost:3000/api/users/${formData.id}`, formData, {
          headers: { Authorization: `Bearer ${auth.token}` }
        });
        alert('User updated successfully');
      } else {
        await axios.post('http://localhost:3000/api/users', formData, {
          headers: { Authorization: `Bearer ${auth.token}` }
        });
        alert('User created successfully');
      }
      setFormData({ id: null, username: '', password: '', role: 'user' });
      setIsEditing(false);
      fetchUsers();
    } catch (err) {
      alert(err.response?.data?.error || 'Operation failed');
    }
  };

  const editUser = (u) => {
    setFormData({ id: u.id, username: u.username, password: '', role: u.role });
    setIsEditing(true);
  };

  const cancelEdit = () => {
    setFormData({ id: null, username: '', password: '', role: 'user' });
    setIsEditing(false);
  }

  return (
    <div className="page">
      <h2 className="slbl" style={{ fontSize: '1.2em' }}>Admin Dashboard</h2>
      {error && <p style={{ color: 'red' }}>{error}</p>}
      
      <div className="agrid" style={{ marginBottom: '20px' }}>
        <div className="cp" style={{ marginBottom: 0 }}>
          <h3 className="pt">{isEditing ? 'Edit User' : 'Add New User'}</h3>
          <form onSubmit={handleSubmit} style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
            <input 
              type="text" className="ti" placeholder="Username" 
              value={formData.username} onChange={(e) => setFormData({...formData, username: e.target.value})} required 
            />
            <input 
              type="password" className="ti" placeholder={isEditing ? "New Password (leave blank to keep)" : "Password"}
              value={formData.password} onChange={(e) => setFormData({...formData, password: e.target.value})} 
              required={!isEditing} 
            />
            <select 
              className="ti" 
              value={formData.role} onChange={(e) => setFormData({...formData, role: e.target.value})}
            >
              <option value="user">User</option>
              <option value="admin">Admin</option>
            </select>
            <div style={{ display: 'flex', gap: '10px', marginTop: '10px' }}>
              <button type="submit" className="ap" style={{ flex: 1 }}>{isEditing ? 'Update User' : 'Create User'}</button>
              {isEditing && (
                <button type="button" className="btn bclose" style={{ flex: 1 }} onClick={cancelEdit}>Cancel</button>
              )}
            </div>
          </form>
        </div>
      </div>

      <div className="cp">
        <h3 className="pt">User Management List</h3>
        <table style={{ width: '100%', borderCollapse: 'collapse', textAlign: 'left' }}>
          <thead>
            <tr style={{ borderBottom: '1px solid #e8d5b0' }}>
              <th style={{ padding: '10px' }}>ID</th>
              <th style={{ padding: '10px' }}>Username</th>
              <th style={{ padding: '10px' }}>Role</th>
              <th style={{ padding: '10px' }}>Actions</th>
            </tr>
          </thead>
          <tbody>
            {users.map(u => (
              <tr key={u.id} style={{ borderBottom: '1px solid #fdf6ec' }}>
                <td style={{ padding: '10px' }}>{u.id}</td>
                <td style={{ padding: '10px' }}>{u.username}</td>
                <td style={{ padding: '10px' }}>{u.role}</td>
                <td style={{ padding: '10px' }}>
                  <button className="btn bopen" onClick={() => editUser(u)} style={{ padding: '5px 10px', fontSize: '0.8em', marginRight: '5px' }}>Edit</button>
                  {u.role !== 'admin' && (
                    <button className="btn bclose" onClick={() => handleDelete(u.id)} style={{ padding: '5px 10px', fontSize: '0.8em' }}>Delete</button>
                  )}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}

export default AdminDashboard;
