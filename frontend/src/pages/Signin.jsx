import React, { useState } from 'react';
import SigninForm from '../components/Auth/SigninForm';
import './Signin.css';

const SigninPage = () => {
  const [isSignedIn, setIsSignedIn] = useState(false);
  const [signinResult, setSigninResult] = useState(null);

  const handleSigninSuccess = (result) => {
    // Store authentication tokens in localStorage
    if (result.auth_token) {
      localStorage.setItem('auth_token', result.auth_token);
    }
    if (result.user_id) {
      localStorage.setItem('user_id', result.user_id);
    }

    setSigninResult(result);
    setIsSignedIn(true);

    // Redirect after a short delay to show success message
    setTimeout(() => {
      window.location.href = '/';
    }, 2000);
  };

  if (isSignedIn) {
    return (
      <div className="signin-success-container">
        <div className="signin-success-content">
          <h1>Welcome Back!</h1>
          <p>You have been signed in successfully.</p>
          <p>Redirecting to the homepage...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="signin-page-container">
      <div className="signin-content">
        <div className="signin-header">
          <div className="signin-icon">
            <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M12 12C14.2091 12 16 10.2091 16 8C16 5.79086 14.2091 4 12 4C9.79086 4 8 5.79086 8 8C8 10.2091 9.79086 12 12 12Z" stroke="#007bff" strokeWidth="2"/>
              <path d="M6 20C6 17.7909 7.79086 16 10 16H14C16.2091 16 18 17.7909 18 20C18 21.1046 17.1046 22 16 22H8C6.89543 22 6 21.1046 6 20Z" stroke="#007bff" strokeWidth="2"/>
            </svg>
          </div>
          <h1>Sign In to Your Account</h1>
          <p>Access your personalized Physical AI & Humanoid Robotics learning experience</p>
        </div>

        <SigninForm onSignin={handleSigninSuccess} />

        <div className="auth-links">
          <p>
            Don't have an account? <a href="/signup">Sign Up</a>
          </p>
        </div>

        <div className="back-to-home">
          <a href="/">← Back to Homepage</a>
        </div>
      </div>
    </div>
  );
};

export default SigninPage;