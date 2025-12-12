import React, { useState, useEffect } from 'react';

const ProfileEditor = ({ userId, onSave }) => {
  const [profileData, setProfileData] = useState({
    os: '',
    cpu: '',
    gpu: '',
    ram_gb: '',
    programming_experience: 'beginner',
    robotics_experience: 'none',
    development_environment: '',
    primary_language: '',
    learning_goals: []
  });
  const [errors, setErrors] = useState({});
  const [isLoading, setIsLoading] = useState(false);
  const [isLoaded, setIsLoaded] = useState(false);

  // Load profile data when component mounts
  useEffect(() => {
    const loadProfile = async () => {
      try {
        const response = await fetch(`/api/profile/${userId}`, {
          headers: {
            'Authorization': `Bearer ${localStorage.getItem('auth_token')}`
          }
        });

        if (response.ok) {
          const profile = await response.json();
          setProfileData({
            os: profile.os || '',
            cpu: profile.cpu || '',
            gpu: profile.gpu || '',
            ram_gb: profile.ram_gb || '',
            programming_experience: profile.programming_experience || 'beginner',
            robotics_experience: profile.robotics_experience || 'none',
            development_environment: profile.development_environment || '',
            primary_language: profile.primary_language || '',
            learning_goals: profile.learning_goals || []
          });
        }
        setIsLoaded(true);
      } catch (error) {
        console.error('Error loading profile:', error);
        setIsLoaded(true);
      }
    };

    if (userId) {
      loadProfile();
    }
  }, [userId]);

  const handleChange = (e) => {
    const { name, value, type, checked } = e.target;

    if (type === 'checkbox') {
      // Handle learning goals checkboxes
      if (checked) {
        setProfileData(prev => ({
          ...prev,
          learning_goals: [...prev.learning_goals, value]
        }));
      } else {
        setProfileData(prev => ({
          ...prev,
          learning_goals: prev.learning_goals.filter(goal => goal !== value)
        }));
      }
    } else {
      setProfileData(prev => ({
        ...prev,
        [name]: value
      }));
    }

    // Clear error when user starts typing
    if (errors[name]) {
      setErrors(prev => ({
        ...prev,
        [name]: ''
      }));
    }
  };

  const validateForm = () => {
    const newErrors = {};

    // RAM validation
    if (profileData.ram_gb && isNaN(Number(profileData.ram_gb))) {
      newErrors.ram_gb = 'RAM must be a number';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setIsLoading(true);

    try {
      const response = await fetch('/api/profile/', {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('auth_token')}`
        },
        body: JSON.stringify({
          os: profileData.os,
          cpu: profileData.cpu,
          gpu: profileData.gpu,
          ram_gb: parseInt(profileData.ram_gb) || null,
          programming_experience: profileData.programming_experience,
          robotics_experience: profileData.robotics_experience,
          development_environment: profileData.development_environment,
          primary_language: profileData.primary_language,
          learning_goals: profileData.learning_goals
        })
      });

      const result = await response.json();

      if (response.ok) {
        onSave(result); // Notify parent component of successful save
      } else {
        setErrors({ submit: result.message || 'Profile update failed' });
      }
    } catch (error) {
      setErrors({ submit: 'Network error occurred' });
    } finally {
      setIsLoading(false);
    }
  };

  if (!isLoaded) {
    return <div>Loading profile...</div>;
  }

  return (
    <div className="profile-editor-container">
      <h2>Edit Your Profile</h2>

      <form onSubmit={handleSubmit} className="profile-form">
        <div className="form-section">
          <h3>Hardware & Software Information</h3>

          <div className="form-group">
            <label htmlFor="os">Operating System:</label>
            <select
              id="os"
              name="os"
              value={profileData.os}
              onChange={handleChange}
            >
              <option value="">Select your OS</option>
              <option value="Windows 10">Windows 10</option>
              <option value="Windows 11">Windows 11</option>
              <option value="macOS">macOS</option>
              <option value="Linux">Linux</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="cpu">CPU:</label>
            <input
              type="text"
              id="cpu"
              name="cpu"
              value={profileData.cpu}
              onChange={handleChange}
              placeholder="e.g., Intel i7-12700K"
            />
          </div>

          <div className="form-group">
            <label htmlFor="gpu">GPU:</label>
            <input
              type="text"
              id="gpu"
              name="gpu"
              value={profileData.gpu}
              onChange={handleChange}
              placeholder="e.g., NVIDIA RTX 3080"
            />
          </div>

          <div className="form-group">
            <label htmlFor="ram_gb">RAM (GB):</label>
            <input
              type="number"
              id="ram_gb"
              name="ram_gb"
              value={profileData.ram_gb}
              onChange={handleChange}
              placeholder="e.g., 16"
              min="1"
              max="128"
              className={errors.ram_gb ? 'error' : ''}
            />
            {errors.ram_gb && <span className="error-message">{errors.ram_gb}</span>}
          </div>
        </div>

        <div className="form-section">
          <h3>Experience Level</h3>

          <div className="form-group">
            <label htmlFor="programming_experience">Programming Experience:</label>
            <select
              id="programming_experience"
              name="programming_experience"
              value={profileData.programming_experience}
              onChange={handleChange}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
              <option value="expert">Expert</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="robotics_experience">Robotics Experience:</label>
            <select
              id="robotics_experience"
              name="robotics_experience"
              value={profileData.robotics_experience}
              onChange={handleChange}
            >
              <option value="none">None</option>
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="development_environment">Development Environment:</label>
            <input
              type="text"
              id="development_environment"
              name="development_environment"
              value={profileData.development_environment}
              onChange={handleChange}
              placeholder="e.g., VS Code, PyCharm"
            />
          </div>

          <div className="form-group">
            <label htmlFor="primary_language">Primary Programming Language:</label>
            <input
              type="text"
              id="primary_language"
              name="primary_language"
              value={profileData.primary_language}
              onChange={handleChange}
              placeholder="e.g., Python, C++, JavaScript"
            />
          </div>
        </div>

        <div className="form-section">
          <h3>Learning Goals</h3>

          <div className="form-group checkboxes">
            <label>
              <input
                type="checkbox"
                name="learning_goals"
                value="robotics"
                checked={profileData.learning_goals.includes('robotics')}
                onChange={handleChange}
              />
              Robotics
            </label>

            <label>
              <input
                type="checkbox"
                name="learning_goals"
                value="AI"
                checked={profileData.learning_goals.includes('AI')}
                onChange={handleChange}
              />
              Artificial Intelligence
            </label>

            <label>
              <input
                type="checkbox"
                name="learning_goals"
                value="simulation"
                checked={profileData.learning_goals.includes('simulation')}
                onChange={handleChange}
              />
              Simulation
            </label>

            <label>
              <input
                type="checkbox"
                name="learning_goals"
                value="hardware"
                checked={profileData.learning_goals.includes('hardware')}
                onChange={handleChange}
              />
              Hardware Integration
            </label>
          </div>
        </div>

        {errors.submit && <div className="error-message submit-error">{errors.submit}</div>}

        <button type="submit" disabled={isLoading} className="save-profile-button">
          {isLoading ? 'Saving...' : 'Save Profile'}
        </button>
      </form>
    </div>
  );
};

export default ProfileEditor;