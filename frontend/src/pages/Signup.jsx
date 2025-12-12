import React, { useState } from 'react';
import SignupForm from '../components/Auth/SignupForm';
import './Signup.css';

const SignupPage = () => {
  const [isSignedUp, setIsSignedUp] = useState(false);
  const [signupResult, setSignupResult] = useState(null);

  const handleSignupSuccess = (result) => {
    // Store authentication tokens in localStorage
    if (result.auth_token) {
      localStorage.setItem('auth_token', result.auth_token);
    }
    if (result.user_id) {
      localStorage.setItem('user_id', result.user_id);
    }

    setSignupResult(result);
    setIsSignedUp(true);
  };

  if (isSignedUp) {
    return (
      <div className="signup-success-container">
        <div className="signup-success-content">
          <div className="signup-success-icon">
            <svg width="64" height="64" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M12 12C14.2091 12 16 10.2091 16 8C16 5.79086 14.2091 4 12 4C9.79086 4 8 5.79086 8 8C8 10.2091 9.79086 12 12 12Z" stroke="#28a745" strokeWidth="2"/>
              <path d="M6 20C6 17.7909 7.79086 16 10 16H14C16.2091 16 18 17.7909 18 20C18 21.1046 17.1046 22 16 22H8C6.89543 22 6 21.1046 6 20Z" stroke="#28a745" strokeWidth="2"/>
              <path d="M15 11L12 14L9 11" stroke="#28a745" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          </div>
          <h1>Welcome to Physical AI & Humanoid Robotics Textbook!</h1>
          <p>Your account has been created successfully.</p>
          <p>Please check your email to verify your account.</p>
          <div className="success-actions">
            <button
              onClick={() => window.location.href = '/signin'}
              className="primary-button"
            >
              Go to Sign In
            </button>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="signup-page-container">
      <div className="signup-content">
        <div className="signup-header">
          <div className="signup-icon">
            <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M16 21V19C16 17.9391 15.5786 16.9217 14.8284 16.1716C14.0783 15.4214 13.0609 15 12 15H8C6.93913 15 5.92172 15.4214 5.17157 16.1716C4.42143 16.9217 4 17.9391 4 19V21" stroke="#6f42c1" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              <path d="M12 11C14.2091 11 16 9.20914 16 7C16 4.79086 14.2091 3 12 3C9.79086 3 8 4.79086 8 7C8 9.20914 9.79086 11 12 11Z" stroke="#6f42c1" strokeWidth="2"/>
            </svg>
          </div>
          <h1>Create Your Account</h1>
          <p>Join our Physical AI & Humanoid Robotics learning platform</p>
        </div>

        <SignupForm onSignup={handleSignupSuccess} />

        <div className="auth-links">
          <p>
            Already have an account? <a href="/signin">Sign In</a>
          </p>
        </div>

        <div className="back-to-home">
          <a href="/">← Back to Homepage</a>
        </div>
      </div>
    </div>
  );
};

export default SignupPage;