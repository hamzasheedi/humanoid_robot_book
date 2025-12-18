// apiConfig.js
// Configuration for API endpoints in Docusaurus

// Global variable to hold the backend URL
// Check if process is available (Node.js environment) or use default
let BACKEND_URL = 'https://backend-z4du.onrender.com';

// Try to get backend URL from environment if available
if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL) {
  BACKEND_URL = process.env.REACT_APP_BACKEND_URL;
}

// Also check if window is available (browser environment) and if there's a config available there
if (typeof window !== 'undefined' && window.REACT_APP_BACKEND_URL) {
  BACKEND_URL = window.REACT_APP_BACKEND_URL;
}

// Function to initialize the config with the proper backend URL
export const initializeApiConfig = (backendUrl) => {
  if (backendUrl) {
    BACKEND_URL = backendUrl;
  }
};

// Export the configuration object
export const API_CONFIG = {
  get BASE_URL() {
    return BACKEND_URL;
  },
  ENDPOINTS: {
    AUTH: {
      SIGNUP: '/api/auth/signup',
      SIGNIN: '/api/auth/signin',
      SIGNOUT: '/api/auth/signout',
      SESSION_STATUS: '/api/auth/session-status',
      REFRESH_SESSION: '/api/auth/refresh-session'
    },
    PROFILE: {
      GET_PROFILE: '/api/profile',
      UPDATE_PROFILE: '/api/profile',
    },
    PERSONALIZATION: {
      GET_PERSONALIZATION: '/api/personalization',
      UPDATE_PERSONALIZATION: '/api/personalization',
    },
    CHAT: {
      ASK: '/chat/ask',
      CONTEXT: '/chat/context',
      HISTORY: '/chat/history'
    },
    TEXTBOOK: {
      GET_CONTENT: '/textbook'
    }
  }
};
