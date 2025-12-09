import React, { useState } from 'react';
import ChatbotWindow from '../components/ChatbotWindow';

const ChatbotPage = () => {
  const [sessionId, setSessionId] = useState(null);

  return (
    <div className="chatbot-page">
      <h1>AI Textbook Assistant</h1>
      <p>Ask questions about the Physical AI & Humanoid Robotics textbook content.</p>
      
      <ChatbotWindow 
        sessionId={sessionId} 
        onSessionIdChange={setSessionId} 
      />
      
      <div className="instructions">
        <h3>How to use:</h3>
        <ul>
          <li>Type your question in the chat window and press Enter/Click Send</li>
          <li>Select text in the textbook and click the floating chat button to ask about specific content</li>
          <li>Use the copy button to copy answers to your clipboard</li>
        </ul>
      </div>
    </div>
  );
};

export default ChatbotPage;