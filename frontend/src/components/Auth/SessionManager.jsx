import React, { useEffect, useState } from 'react';

const SessionManager = ({ onSessionChange, children }) => {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [user, setUser] = useState(null);
  const [isLoading, setIsLoading] = useState(true);

  // Check session status on component mount
  useEffect(() => {
    checkSessionStatus();
  }, []);

  // Check if user has a valid session
  const checkSessionStatus = async () => {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        setIsAuthenticated(false);
        setIsLoading(false);
        onSessionChange({ isAuthenticated: false, user: null });
        return;
      }

      // Verify session with backend
      const response = await fetch('/api/auth/session', {
        headers: {
          'Authorization': `Bearer ${token}`
        }
      });

      if (response.ok) {
        const sessionData = await response.json();
        if (sessionData.valid) {
          setIsAuthenticated(true);
          setUser(sessionData.user);
          onSessionChange({ isAuthenticated: true, user: sessionData.user });
        } else {
          // Token is invalid, clear it
          localStorage.removeItem('auth_token');
          setIsAuthenticated(false);
        }
      } else {
        // Session verification failed, clear token
        localStorage.removeItem('auth_token');
        setIsAuthenticated(false);
      }
    } catch (error) {
      console.error('Error checking session status:', error);
      setIsAuthenticated(false);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle user sign in
  const handleSignin = (userData) => {
    if (userData.token) {
      localStorage.setItem('auth_token', userData.token);
      setIsAuthenticated(true);
      setUser(userData.user || null);
      onSessionChange({ isAuthenticated: true, user: userData.user });
    }
  };

  // Handle user sign out
  const handleSignout = async () => {
    try {
      const token = localStorage.getItem('auth_token');
      if (token) {
        await fetch('/api/auth/signout', {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        });
      }
    } catch (error) {
      console.error('Error during signout:', error);
    } finally {
      localStorage.removeItem('auth_token');
      setIsAuthenticated(false);
      setUser(null);
      onSessionChange({ isAuthenticated: false, user: null });
    }
  };

  // Refresh session token
  const refreshSession = async () => {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) return false;

      const response = await fetch('/api/auth/refresh', {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json'
        }
      });

      if (response.ok) {
        const data = await response.json();
        if (data.token) {
          localStorage.setItem('auth_token', data.token);
          return true;
        }
      }
    } catch (error) {
      console.error('Error refreshing session:', error);
    }
    return false;
  };

  if (isLoading) {
    return <div>Loading...</div>;
  }

  return (
    <div className="session-manager">
      {children({
        isAuthenticated,
        user,
        handleSignin,
        handleSignout,
        refreshSession,
        checkSessionStatus
      })}
    </div>
  );
};

export default SessionManager;