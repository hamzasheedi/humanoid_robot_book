import React, { useState, useEffect } from 'react';
import ProfileEditor from '../components/Auth/ProfileEditor';
import './Profile.css';

const ProfilePage = () => {
  const [userId, setUserId] = useState(null);
  const [user, setUser] = useState(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // Get user ID from local storage
    const storedUserId = localStorage.getItem('user_id');
    if (storedUserId) {
      setUserId(storedUserId);
    } else {
      // Try to get user info from session
      const fetchUserInfo = async () => {
        try {
          const token = localStorage.getItem('auth_token');
          if (!token) {
            window.location.href = '/signin';
            return;
          }

          const response = await fetch('/api/auth/session', {
            headers: {
              'Authorization': `Bearer ${token}`
            }
          });

          if (response.ok) {
            const sessionData = await response.json();
            if (sessionData.valid && sessionData.user) {
              setUserId(sessionData.user.id);
              setUser(sessionData.user);
            } else {
              window.location.href = '/signin';
            }
          } else {
            window.location.href = '/signin';
          }
        } catch (error) {
          console.error('Error fetching user info:', error);
          window.location.href = '/signin';
        } finally {
          setIsLoading(false);
        }
      };

      fetchUserInfo();
    }
  }, []);

  const handleProfileSave = (profileData) => {
    setUser(prev => ({
      ...prev,
      ...profileData
    }));
  };

  if (isLoading) {
    return <div className="profile-loading">Loading profile...</div>;
  }

  if (!userId) {
    return <div className="profile-error">Please sign in to view your profile.</div>;
  }

  return (
    <div className="profile-page-container">
      <div className="profile-content">
        <h1>Your Profile</h1>
        <p>Manage your hardware specifications, experience level, and learning preferences</p>

        <div className="profile-section">
          <h2>Profile Information</h2>
          {user && (
            <div className="user-info">
              <p><strong>Email:</strong> {user.email}</p>
              <p><strong>Member Since:</strong> {user.created_at ? new Date(user.created_at).toLocaleDateString() : 'Unknown'}</p>
            </div>
          )}
        </div>

        <div className="profile-section">
          <h2>Edit Profile</h2>
          <ProfileEditor
            userId={userId}
            onSave={handleProfileSave}
          />
        </div>
      </div>
    </div>
  );
};

export default ProfilePage;