import React, { useEffect } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const SetDarkModeDefault = () => {
  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      // Set theme preference to dark in localStorage
      localStorage.setItem('theme', 'dark');
      
      // Apply dark theme immediately to avoid flash of light theme
      document.documentElement.setAttribute('data-theme', 'dark');
      document.documentElement.style.colorScheme = 'dark';
      
      // Also update any theme-switcher elements if they exist
      const themeElements = document.querySelectorAll('[data-theme-toggle]');
      themeElements.forEach(element => {
        element.setAttribute('aria-pressed', 'true');
      });
    }
  }, []);

  return null; // This component doesn't render anything
};

export default SetDarkModeDefault;