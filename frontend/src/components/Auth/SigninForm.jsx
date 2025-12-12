import React, { useState } from 'react';

const SigninForm = ({ onSignin }) => {
  const [formData, setFormData] = useState({
    email: '',
    password: ''
  });
  const [errors, setErrors] = useState({});
  const [isLoading, setIsLoading] = useState(false);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));

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
      // Call the signin API
      const response = await fetch('/api/auth/signin', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          email: formData.email,
          password: formData.password
        })
      });

      const result = await response.json();

      if (response.ok) {
        onSignin(result); // Pass the result to parent component
      } else {
        setErrors({ submit: result.message || 'Signin failed' });
      }
    } catch (error) {
      setErrors({ submit: 'Network error occurred' });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="signin-form-container">
      <h2>Sign In to Your Account</h2>

      <form onSubmit={handleSubmit} className="signin-form">
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

        {errors.submit && <div className="error-message submit-error">{errors.submit}</div>}

        <button type="submit" disabled={isLoading} className="signin-button">
          {isLoading ? 'Signing In...' : 'Sign In'}
        </button>
      </form>

      <div className="form-footer">
        <p>Don't have an account? <a href="/signup">Sign up here</a></p>
      </div>
    </div>
  );
};

export default SigninForm;