// FrontendPersonalizationService.js
// Service for handling personalization API calls

class FrontendPersonalizationService {
  constructor() {
    this.baseUrl = process.env.REACT_APP_BACKEND_URL || '';
  }

  // Get personalization settings for a user
  async getPersonalization(userId) {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        throw new Error('Authentication token required');
      }

      const response = await fetch(`${this.baseUrl}/api/personalization/${userId}`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Failed to fetch personalization settings');
      }

      return result;
    } catch (error) {
      console.error('Get personalization error:', error);
      throw error;
    }
  }

  // Update personalization settings for a user
  async updatePersonalization(personalizationData) {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        throw new Error('Authentication token required');
      }

      const response = await fetch(`${this.baseUrl}/api/personalization/`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify(personalizationData),
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Failed to update personalization settings');
      }

      return result;
    } catch (error) {
      console.error('Update personalization error:', error);
      throw error;
    }
  }

  // Create personalization settings for a user
  async createPersonalization(personalizationData) {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        throw new Error('Authentication token required');
      }

      const response = await fetch(`${this.baseUrl}/api/personalization/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify(personalizationData),
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Failed to create personalization settings');
      }

      return result;
    } catch (error) {
      console.error('Create personalization error:', error);
      throw error;
    }
  }

  // Delete personalization settings for a user
  async deletePersonalization(userId) {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        throw new Error('Authentication token required');
      }

      const response = await fetch(`${this.baseUrl}/api/personalization/${userId}`, {
        method: 'DELETE',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.message || 'Failed to delete personalization settings');
      }

      return result;
    } catch (error) {
      console.error('Delete personalization error:', error);
      throw error;
    }
  }

  // Get content difficulty based on user profile
  getContentDifficulty(userProfile) {
    if (!userProfile) return 'intermediate';

    const { programming_experience, robotics_experience } = userProfile;

    if (programming_experience === 'beginner' || robotics_experience === 'none') {
      return 'beginner';
    } else if (programming_experience === 'advanced' || robotics_experience === 'advanced') {
      return 'advanced';
    } else if (programming_experience === 'expert' || robotics_experience === 'expert') {
      return 'expert';
    }

    return 'intermediate';
  }

  // Get hardware-specific examples based on user profile
  getHardwareExamples(userProfile) {
    if (!userProfile) return [];

    const examples = [];

    if (userProfile.gpu && userProfile.gpu.toLowerCase().includes('nvidia')) {
      examples.push('CUDA-specific examples');
      examples.push('GPU acceleration techniques');
    }

    if (userProfile.gpu && userProfile.gpu.toLowerCase().includes('rtx')) {
      examples.push('Real-time rendering examples');
      examples.push('AI inference optimization');
    }

    if (userProfile.ram_gb && userProfile.ram_gb >= 32) {
      examples.push('Memory-intensive operations');
      examples.push('Large dataset processing');
    }

    return examples;
  }

  // Get response complexity based on user experience
  getResponseComplexity(userProfile) {
    if (!userProfile) return 'moderate';

    const { programming_experience, robotics_experience } = userProfile;

    if (programming_experience === 'beginner' || robotics_experience === 'none') {
      return 'simple';
    } else if (programming_experience === 'advanced' || robotics_experience === 'advanced') {
      return 'detailed';
    } else if (programming_experience === 'expert' || robotics_experience === 'expert') {
      return 'comprehensive';
    }

    return 'moderate';
  }

  // Get preferred content format based on learning goals
  getPreferredContentFormat(userProfile) {
    if (!userProfile || !userProfile.learning_goals) return 'balanced';

    const goals = userProfile.learning_goals;

    if (goals.includes('simulation')) {
      return 'simulation-focused';
    } else if (goals.includes('hardware')) {
      return 'hardware-focused';
    } else if (goals.includes('AI')) {
      return 'AI-focused';
    } else if (goals.includes('robotics')) {
      return 'robotics-focused';
    }

    return 'balanced';
  }

  // Apply content personalization
  applyContentPersonalization(content, userProfile) {
    if (!content || !userProfile) return content;

    let adaptedContent = content;

    // Adjust content difficulty
    const difficulty = this.getContentDifficulty(userProfile);

    switch (difficulty) {
      case 'beginner':
        adaptedContent = this.simplifyContent(adaptedContent);
        break;
      case 'advanced':
        adaptedContent = this.enhanceContent(adaptedContent);
        break;
      case 'expert':
        adaptedContent = this.expertifyContent(adaptedContent);
        break;
      default:
        // Moderate content for intermediate level
        break;
    }

    // Add hardware-specific examples
    const hardwareExamples = this.getHardwareExamples(userProfile);
    if (hardwareExamples.length > 0) {
      adaptedContent = this.addHardwareExamples(adaptedContent, hardwareExamples);
    }

    return adaptedContent;
  }

  // Helper function to simplify content
  simplifyContent(content) {
    // Remove advanced concepts, add more explanations
    // This would contain logic to make content more accessible
    return content;
  }

  // Helper function to enhance content
  enhanceContent(content) {
    // Add more advanced examples and in-depth analysis
    // This would contain logic to add depth to content
    return content;
  }

  // Helper function to make content expert-level
  expertifyContent(content) {
    // Add expert-level analysis, research citations, and advanced techniques
    // This would contain logic to make content suitable for experts
    return content;
  }

  // Helper function to add hardware examples
  addHardwareExamples(content, examples) {
    // Inject hardware-specific examples into content
    // This would contain logic to insert relevant examples
    return content;
  }

  // Get personalization validation schema
  getValidationSchema() {
    return {
      content_difficulty: {
        required: false,
        type: 'string',
        enum: ['beginner', 'intermediate', 'advanced', 'expert']
      },
      preferred_examples: {
        required: false,
        type: 'array',
        items: { type: 'string' }
      },
      response_complexity: {
        required: false,
        type: 'string',
        enum: ['simple', 'moderate', 'detailed', 'comprehensive']
      },
      content_format: {
        required: false,
        type: 'string',
        enum: ['balanced', 'simulation-focused', 'hardware-focused', 'AI-focused', 'robotics-focused']
      },
      include_advanced_topics: {
        required: false,
        type: 'boolean'
      },
      include_practical_exercises: {
        required: false,
        type: 'boolean'
      }
    };
  }

  // Validate personalization data
  validatePersonalizationData(personalizationData) {
    const schema = this.getValidationSchema();
    const errors = {};

    for (const [field, rules] of Object.entries(schema)) {
      const value = personalizationData[field];

      // Skip validation for optional fields that are not provided
      if (value === undefined || value === null) {
        continue;
      }

      // Validate type
      if (rules.type === 'string' && typeof value !== 'string') {
        errors[field] = `${field} must be a string`;
      } else if (rules.type === 'number' && typeof value !== 'number') {
        errors[field] = `${field} must be a number`;
      } else if (rules.type === 'array' && !Array.isArray(value)) {
        errors[field] = `${field} must be an array`;
      } else if (rules.type === 'boolean' && typeof value !== 'boolean') {
        errors[field] = `${field} must be a boolean`;
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

export default new FrontendPersonalizationService();