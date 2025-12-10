import React, { useState, useEffect } from 'react';
import ChatbotWindow from '../components/ChatbotWindow';

const Root = ({ children }) => {
  const [showChatbot, setShowChatbot] = useState(false);

  // Handle keyboard shortcut to open chatbot (Ctrl/Cmd + K)
  useEffect(() => {
    const handleKeyDown = (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
        e.preventDefault();
        setShowChatbot(prev => !prev);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, []);

  return (
    <>
      {children}
      <ChatbotWindow
        isOpen={showChatbot}
        onClose={() => setShowChatbot(prev => !prev)} // This toggles the chatbot state
      />
    </>
  );
};

export default Root;