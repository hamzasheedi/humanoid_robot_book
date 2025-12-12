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
        <h1>Welcome Back!</h1>
        <p>You have been signed in successfully.</p>
        <p>Redirecting to the homepage...</p>
      </div>
    );
  }

  return (
    <div className="signin-page-container">
      <div className="signin-content">
        <h1>Sign In to Your Account</h1>
        <p>Access your personalized Physical AI & Humanoid Robotics learning experience</p>

        <SigninForm onSignin={handleSigninSuccess} />

        <div className="auth-links">
          <p>
            Don't have an account? <a href="/signup">Sign Up</a>
          </p>
        </div>
      </div>
    </div>
  );
};

export default SigninPage;