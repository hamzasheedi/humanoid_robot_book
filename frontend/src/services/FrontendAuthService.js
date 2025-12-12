// FrontendAuthService.js
// Service for handling authentication API calls
import { initializeApiConfig, API_CONFIG } from '../config/apiConfig';

// Initialize the API config with the backend URL from environment if available
// Wrap in try-catch to handle cases where process is not available during build
try {
  if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL) {
    initializeApiConfig(process.env.REACT_APP_BACKEND_URL);
  }
} catch (error) {
  // If process is not defined (e.g., during browser execution), continue with default config
  console.warn('Environment variable access failed, using default config:', error.message);
}

class FrontendAuthService {
  constructor() {
    this.baseUrl = API_CONFIG.BASE_URL;
  }

  // Sign up a new user
  async signup(userData) {
    try {
      const response = await fetch(`${this.baseUrl}/api/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(userData),
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Signup failed');
      }

      return result;
    } catch (error) {
      console.error('Signup error:', error);
      throw error;
    }
  }

  // Sign in an existing user
  async signin(credentials) {
    try {
      const response = await fetch(`${this.baseUrl}/api/auth/signin`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(credentials),
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Signin failed');
      }

      return result;
    } catch (error) {
      console.error('Signin error:', error);
      throw error;
    }
  }

  // Sign out the current user
  async signout() {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        return { success: true };
      }

      const response = await fetch(`${this.baseUrl}/api/auth/signout`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
      });

      const result = await response.json();

      if (!response.ok) {
        console.error('Signout failed:', result.message);
      }

      // Clear local storage regardless of backend response
      localStorage.removeItem('auth_token');
      localStorage.removeItem('user_id');

      return { success: true, ...result };
    } catch (error) {
      console.error('Signout error:', error);
      // Clear local storage even if backend call fails
      localStorage.removeItem('auth_token');
      localStorage.removeItem('user_id');
      throw error;
    }
  }

  // Check current session status
  async checkSession() {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        return { valid: false, user: null };
      }

      const response = await fetch(`${this.baseUrl}/api/auth/session-status`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Session check failed');
      }

      return result;
    } catch (error) {
      console.error('Session check error:', error);
      return { valid: false, user: null };
    }
  }

  // Refresh the current session
  async refreshSession() {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        throw new Error('No token available for refresh');
      }

      const response = await fetch(`${this.baseUrl}/api/auth/refresh-session`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Session refresh failed');
      }

      // Update the token in local storage
      if (result.token) {
        localStorage.setItem('auth_token', result.token);
      }

      return result;
    } catch (error) {
      console.error('Session refresh error:', error);
      throw error;
    }
  }

  // Get current user ID from local storage
  getCurrentUserId() {
    return localStorage.getItem('user_id');
  }

  // Set current user ID in local storage
  setCurrentUserId(userId) {
    if (userId) {
      localStorage.setItem('user_id', userId);
    } else {
      localStorage.removeItem('user_id');
    }
  }

  // Check if user is authenticated
  isAuthenticated() {
    const token = localStorage.getItem('auth_token');
    return !!token;
  }
}

export default new FrontendAuthService();