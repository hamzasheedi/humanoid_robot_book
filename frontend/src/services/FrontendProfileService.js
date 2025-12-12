// FrontendProfileService.js
// Service for handling profile API calls
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

class FrontendProfileService {
  constructor() {
    this.baseUrl = API_CONFIG.BASE_URL;
  }

  // Get user profile by user ID
  async getProfile(userId) {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        throw new Error('Authentication token required');
      }

      const response = await fetch(`${this.baseUrl}/api/profile/${userId}`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Failed to fetch profile');
      }

      return result;
    } catch (error) {
      console.error('Get profile error:', error);
      throw error;
    }
  }

  // Update user profile
  async updateProfile(profileData) {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        throw new Error('Authentication token required');
      }

      const response = await fetch(`${this.baseUrl}/api/profile/`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify(profileData),
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Failed to update profile');
      }

      return result;
    } catch (error) {
      console.error('Update profile error:', error);
      throw error;
    }
  }

  // Create user profile (for new users)
  async createProfile(profileData) {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        throw new Error('Authentication token required');
      }

      const response = await fetch(`${this.baseUrl}/api/profile/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify(profileData),
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Failed to create profile');
      }

      return result;
    } catch (error) {
      console.error('Create profile error:', error);
      throw error;
    }
  }

  // Delete user profile
  async deleteProfile(userId) {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        throw new Error('Authentication token required');
      }

      const response = await fetch(`${this.baseUrl}/api/profile/${userId}`, {
        method: 'DELETE',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Failed to delete profile');
      }

      return result;
    } catch (error) {
      console.error('Delete profile error:', error);
      throw error;
    }
  }

  // Get profile fields validation schema
  getValidationSchema() {
    return {
      os: { required: false, type: 'string' },
      cpu: { required: false, type: 'string' },
      gpu: { required: false, type: 'string' },
      ram_gb: { required: false, type: 'number' },
      programming_experience: {
        required: true,
        type: 'string',
        enum: ['beginner', 'intermediate', 'advanced', 'expert']
      },
      robotics_experience: {
        required: true,
        type: 'string',
        enum: ['none', 'beginner', 'intermediate', 'advanced']
      },
      development_environment: { required: false, type: 'string' },
      primary_language: { required: false, type: 'string' },
      learning_goals: { required: false, type: 'array', items: { type: 'string' } }
    };
  }

  // Validate profile data
  validateProfileData(profileData) {
    const schema = this.getValidationSchema();
    const errors = {};

    // Validate each field according to schema
    for (const [field, rules] of Object.entries(schema)) {
      const value = profileData[field];

      // Check required fields
      if (rules.required && (value === undefined || value === null || value === '')) {
        errors[field] = `${field} is required`;
        continue;
      }

      // Skip validation for optional fields that are not provided
      if (value === undefined || value === null || value === '') {
        continue;
      }

      // Validate type
      if (rules.type === 'string' && typeof value !== 'string') {
        errors[field] = `${field} must be a string`;
      } else if (rules.type === 'number' && typeof value !== 'number') {
        errors[field] = `${field} must be a number`;
      } else if (rules.type === 'array' && !Array.isArray(value)) {
        errors[field] = `${field} must be an array`;
      }

      // Validate enum values
      if (rules.enum && !rules.enum.includes(value)) {
        errors[field] = `${field} must be one of: ${rules.enum.join(', ')}`;
      }

      // Validate array items if specified
      if (rules.type === 'array' && rules.items && Array.isArray(value)) {
        for (let i = 0; i < value.length; i++) {
          if (rules.items.type === 'string' && typeof value[i] !== 'string') {
            errors[`${field}[${i}]`] = `${field}[${i}] must be a string`;
          }
        }
      }
    }

    return {
      isValid: Object.keys(errors).length === 0,
      errors
    };
  }
}

export default new FrontendProfileService();