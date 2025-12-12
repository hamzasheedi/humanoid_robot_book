import React, { useState } from 'react';
import SignupForm from '../components/Auth/SignupForm';
import './Signup.css';

const SignupPage = () => {
  const [isSignedUp, setIsSignedUp] = useState(false);
  const [signupResult, setSignupResult] = useState(null);

  const handleSignupSuccess = (result) => {
    setSignupResult(result);
    setIsSignedUp(true);
  };

  if (isSignedUp) {
    return (
      <div className="signup-success-container">
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
    );
  }

  return (
    <div className="signup-page-container">
      <div className="signup-content">
        <h1>Create Your Account</h1>
        <p>Join our Physical AI & Humanoid Robotics learning platform</p>

        <SignupForm onSignupSuccess={handleSignupSuccess} />

        <div className="auth-links">
          <p>
            Already have an account? <a href="/signin">Sign In</a>
          </p>
        </div>
      </div>
    </div>
  );
};

export default SignupPage;