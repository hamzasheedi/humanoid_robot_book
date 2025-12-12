import React, { useEffect, useState } from 'react';

const ChatPersonalizer = ({ userId, onPersonalizationUpdate }) => {
  const [userProfile, setUserProfile] = useState(null);
  const [isLoading, setIsLoading] = useState(true);

  // Load user profile when component mounts
  useEffect(() => {
    const loadUserProfile = async () => {
      try {
        const response = await fetch(`/api/profile/${userId}`, {
          headers: {
            'Authorization': `Bearer ${localStorage.getItem('auth_token')}`
          }
        });

        if (response.ok) {
          const profile = await response.json();
          setUserProfile(profile);
        }
      } catch (error) {
        console.error('Error loading user profile for personalization:', error);
      } finally {
        setIsLoading(false);
      }
    };

    if (userId) {
      loadUserProfile();
    }
  }, [userId]);

  // Generate personalization context for chatbot
  const getPersonalizationContext = () => {
    if (!userProfile) return '';

    let context = '';

    // Add experience level context
    if (userProfile.programming_experience || userProfile.robotics_experience) {
      context += `User has ${userProfile.programming_experience} programming experience and ${userProfile.robotics_experience} robotics experience. `;
    }

    // Add hardware context
    if (userProfile.gpu || userProfile.cpu || userProfile.ram_gb) {
      context += `User hardware: `;
      if (userProfile.gpu) context += `GPU: ${userProfile.gpu}, `;
      if (userProfile.cpu) context += `CPU: ${userProfile.cpu}, `;
      if (userProfile.ram_gb) context += `RAM: ${userProfile.ram_gb}GB. `;
    }

    // Add development environment context
    if (userProfile.development_environment) {
      context += `User development environment: ${userProfile.development_environment}. `;
    }

    // Add primary language context
    if (userProfile.primary_language) {
      context += `User primary programming language: ${userProfile.primary_language}. `;
    }

    // Add learning goals context
    if (userProfile.learning_goals && userProfile.learning_goals.length > 0) {
      context += `User learning goals: ${userProfile.learning_goals.join(', ')}. `;
    }

    return context.trim();
  };

  // Apply personalization when user profile changes
  useEffect(() => {
    if (userProfile) {
      const context = getPersonalizationContext();
      onPersonalizationUpdate(context);
    }
  }, [userProfile, onPersonalizationUpdate]);

  if (isLoading) {
    return <div>Preparing personalized chat experience...</div>;
  }

  return null; // This component doesn't render anything, it just manages personalization context
};

export default ChatPersonalizer;