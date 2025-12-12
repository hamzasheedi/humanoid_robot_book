import React, { useState } from 'react';
import './SignupForm.css';

const SignupForm = ({ onSignup }) => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
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

  const handleChange = (e) => {
    const { name, value, type, checked } = e.target;

    if (type === 'checkbox') {
      // Handle learning goals checkboxes
      if (checked) {
        setFormData(prev => ({
          ...prev,
          learning_goals: [...prev.learning_goals, value]
        }));
      } else {
        setFormData(prev => ({
          ...prev,
          learning_goals: prev.learning_goals.filter(goal => goal !== value)
        }));
      }
    } else {
      setFormData(prev => ({
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

    // Email validation
    if (!formData.email) {
      newErrors.email = 'Email is required';
    } else if (!/\S+@\S+\.\S+/.test(formData.email)) {
      newErrors.email = 'Email is invalid';
    }

    // Password validation
    if (!formData.password) {
      newErrors.password = 'Password is required';
    } else if (formData.password.length < 8) {
      newErrors.password = 'Password must be at least 8 characters';
    }

    // Confirm password validation
    if (formData.password !== formData.confirmPassword) {
      newErrors.confirmPassword = 'Passwords do not match';
    }

    // RAM validation
    if (formData.ram_gb && isNaN(Number(formData.ram_gb))) {
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
      // Call the signup API
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          email: formData.email,
          password: formData.password,
          profile: {
            os: formData.os,
            cpu: formData.cpu,
            gpu: formData.gpu,
            ram_gb: parseInt(formData.ram_gb) || null,
            programming_experience: formData.programming_experience,
            robotics_experience: formData.robotics_experience,
            development_environment: formData.development_environment,
            primary_language: formData.primary_language,
            learning_goals: formData.learning_goals
          }
        })
      });

      const result = await response.json();

      if (response.ok) {
        onSignup(result); // Pass the result to parent component
      } else {
        setErrors({ submit: result.message || 'Signup failed' });
      }
    } catch (error) {
      setErrors({ submit: 'Network error occurred' });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="signup-form-container">
      <h2>Create Your Account</h2>

      <form onSubmit={handleSubmit} className="signup-form">
        {/* Basic Authentication Fields */}
        <div className="form-group">
          <label htmlFor="email">Email:</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            className={errors.email ? 'error' : ''}
          />
          {errors.email && <span className="error-message">{errors.email}</span>}
        </div>

        <div className="form-group">
          <label htmlFor="password">Password:</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            className={errors.password ? 'error' : ''}
          />
          {errors.password && <span className="error-message">{errors.password}</span>}
        </div>

        <div className="form-group">
          <label htmlFor="confirmPassword">Confirm Password:</label>
          <input
            type="password"
            id="confirmPassword"
            name="confirmPassword"
            value={formData.confirmPassword}
            onChange={handleChange}
            className={errors.confirmPassword ? 'error' : ''}
          />
          {errors.confirmPassword && <span className="error-message">{errors.confirmPassword}</span>}
        </div>

        {/* Profile Information Fields */}
        <div className="form-section">
          <h3>Hardware & Software Profile (Optional)</h3>

          <div className="form-group">
            <label htmlFor="os">Operating System:</label>
            <select
              id="os"
              name="os"
              value={formData.os}
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
              value={formData.cpu}
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
              value={formData.gpu}
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
              value={formData.ram_gb}
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
              value={formData.programming_experience}
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
              value={formData.robotics_experience}
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
              value={formData.development_environment}
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
              value={formData.primary_language}
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
                checked={formData.learning_goals.includes('robotics')}
                onChange={handleChange}
              />
              Robotics
            </label>

            <label>
              <input
                type="checkbox"
                name="learning_goals"
                value="AI"
                checked={formData.learning_goals.includes('AI')}
                onChange={handleChange}
              />
              Artificial Intelligence
            </label>

            <label>
              <input
                type="checkbox"
                name="learning_goals"
                value="simulation"
                checked={formData.learning_goals.includes('simulation')}
                onChange={handleChange}
              />
              Simulation
            </label>

            <label>
              <input
                type="checkbox"
                name="learning_goals"
                value="hardware"
                checked={formData.learning_goals.includes('hardware')}
                onChange={handleChange}
              />
              Hardware Integration
            </label>
          </div>
        </div>

        {errors.submit && <div className="error-message submit-error">{errors.submit}</div>}

        <button type="submit" disabled={isLoading} className="signup-button">
          {isLoading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;