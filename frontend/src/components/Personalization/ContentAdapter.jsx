import React, { useEffect, useState } from 'react';

const ContentAdapter = ({ content, userId }) => {
  const [personalizedContent, setPersonalizedContent] = useState(content);
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
        console.error('Error loading user profile:', error);
      } finally {
        setIsLoading(false);
      }
    };

    if (userId) {
      loadUserProfile();
    }
  }, [userId]);

  // Apply personalization when content or user profile changes
  useEffect(() => {
    if (!userProfile) return;

    const applyPersonalization = () => {
      // Apply content adaptation based on user profile
      let adaptedContent = content;

      // Adjust content difficulty based on user experience
      if (userProfile.programming_experience === 'beginner' || userProfile.robotics_experience === 'none') {
        adaptedContent = simplifyContent(adaptedContent);
      } else if (userProfile.programming_experience === 'advanced' || userProfile.robotics_experience === 'advanced') {
        adaptedContent = enhanceContent(adaptedContent);
      }

      // Add hardware-specific examples based on user's system
      if (userProfile.gpu) {
        adaptedContent = addHardwareExamples(adaptedContent, userProfile.gpu);
      }

      setPersonalizedContent(adaptedContent);
    };

    applyPersonalization();
  }, [content, userProfile]);

  // Helper functions for content adaptation
  const simplifyContent = (content) => {
    // Simplify complex concepts, add more explanations
    // This would contain logic to make content more accessible
    return content;
  };

  const enhanceContent = (content) => {
    // Add more advanced examples and in-depth analysis
    // This would contain logic to add depth to content
    return content;
  };

  const addHardwareExamples = (content, gpu) => {
    // Inject hardware-specific examples based on user's GPU
    // For example, if user has RTX card, add CUDA-specific examples
    return content;
  };

  if (isLoading) {
    return <div>Adapting content for your experience...</div>;
  }

  return (
    <div className="content-adapter">
      <div
        className="personalized-content"
        dangerouslySetInnerHTML={{ __html: personalizedContent }}
      />
    </div>
  );
};

export default ContentAdapter;