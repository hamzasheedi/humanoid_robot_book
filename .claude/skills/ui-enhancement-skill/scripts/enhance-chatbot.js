const fs = require('fs');
const path = require('path');

/**
 * Enhances the chatbot window with better floating button,
 * more readable chat UI, updated icon, and matching color theme
 */
function enhanceChatbot(frontendDir = './frontend') {
  const chatbotDir = path.join(frontendDir, 'src', 'components');
  const chatbotComponentPath = path.join(chatbotDir, 'ChatbotWindow.jsx');
  const chatbotCssPath = path.join(chatbotDir, 'ChatbotWindow.css');

  // Create the components directory if it doesn't exist
  if (!fs.existsSync(chatbotDir)) {
    fs.mkdirSync(chatbotDir, { recursive: true });
  }

  // Create enhanced Chatbot component
  const chatbotComponent = `import React, { useState, useRef, useEffect } from 'react';
import './ChatbotWindow.css';

const ChatbotWindow = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your Physical AI & Humanoid Robotics assistant. How can I help you today?",
      sender: 'bot',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Simulate API call - replace with actual backend call
      // const response = await fetch('/api/chat', {
      //   method: 'POST',
      //   headers: { 'Content-Type': 'application/json' },
      //   body: JSON.stringify({ message: inputValue })
      // });
      // const data = await response.json();

      // For now, simulate a response after a delay
      setTimeout(() => {
        const botMessage = {
          id: Date.now() + 1,
          text: "This is a simulated response. In the actual implementation, this would connect to the backend API to provide answers about Physical AI & Humanoid Robotics.",
          sender: 'bot',
          timestamp: new Date()
        };
        setMessages(prev => [...prev, botMessage]);
        setIsLoading(false);
      }, 1000);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I encountered an error. Please try again.",
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
      setIsLoading(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const formatTime = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <>
      {/* Floating chat button */}
      <button
        className="chatbot-float-button"
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        <div className="chatbot-icon">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
            <path d="M20 2H4c-1.1 0-1.99.9-1.99 2L2 22l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm-2 12H6v-2h12v2zm0-3H6V9h12v2zm0-3H6V6h12v2z"/>
          </svg>
        </div>
        {isLoading && <div className="chatbot-typing-indicator"></div>}
      </button>

      {/* Chat window */}
      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <div className="chatbot-header-content">
              <div className="chatbot-icon-large">
                <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                  <path d="M20 2H4c-1.1 0-1.99.9-1.99 2L2 22l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm-2 12H6v-2h12v2zm0-3H6V9h12v2zm0-3H6V6h12v2z"/>
                </svg>
              </div>
              <div className="chatbot-title">
                <h3>Physical AI Assistant</h3>
                <p className="chatbot-status">Online • Ready to help</p>
              </div>
            </div>
            <button
              className="chatbot-close-button"
              onClick={toggleChat}
              aria-label="Close chat"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z"/>
              </svg>
            </button>
          </div>

          <div className="chatbot-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={\`message \${message.sender === 'user' ? 'message-user' : 'message-bot'}\`}
              >
                <div className="message-content">
                  <div className="message-text">{message.text}</div>
                  <div className="message-time">{formatTime(message.timestamp)}</div>
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="message message-bot">
                <div className="message-content">
                  <div className="message-typing">
                    <div className="typing-dot"></div>
                    <div className="typing-dot"></div>
                    <div className="typing-dot"></div>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form className="chatbot-input-form" onSubmit={handleSendMessage}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask about Physical AI & Humanoid Robotics..."
              className="chatbot-input"
              disabled={isLoading}
            />
            <button
              type="submit"
              className="chatbot-send-button"
              disabled={!inputValue.trim() || isLoading}
              aria-label="Send message"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z"/>
              </svg>
            </button>
          </form>
        </div>
      )}
    </>
  );
};

export default ChatbotWindow;
`;

  // Create enhanced Chatbot CSS
  const chatbotCss = `/* Enhanced Chatbot Styles */
.chatbot-float-button {
  position: fixed;
  bottom: 30px;
  right: 30px;
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: linear-gradient(135deg, var(--ifm-color-primary) 0%, #667eea 100%);
  color: white;
  border: none;
  cursor: pointer;
  box-shadow: 0 4px 20px rgba(37, 194, 160, 0.4);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
  transition: all 0.3s ease;
  overflow: visible;
}

.chatbot-float-button:hover {
  transform: scale(1.1) translateY(-2px);
  box-shadow: 0 6px 25px rgba(37, 194, 160, 0.6);
}

.chatbot-icon {
  width: 30px;
  height: 30px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.chatbot-icon svg {
  width: 24px;
  height: 24px;
}

.chatbot-typing-indicator {
  position: absolute;
  top: -5px;
  right: -5px;
  width: 12px;
  height: 12px;
  background-color: #ff9800;
  border-radius: 50%;
  animation: pulse 1.5s infinite;
}

@keyframes pulse {
  0% { transform: scale(1); opacity: 1; }
  50% { transform: scale(1.2); opacity: 0.7; }
  100% { transform: scale(1); opacity: 1; }
}

.chatbot-window {
  position: fixed;
  bottom: 100px;
  right: 30px;
  width: 380px;
  height: 500px;
  background: var(--ifm-background-color);
  border-radius: 16px;
  box-shadow: 0 10px 40px rgba(0, 0, 0, 0.15);
  display: flex;
  flex-direction: column;
  z-index: 1000;
  border: 1px solid var(--ifm-toc-border-color);
  overflow: hidden;
}

.chatbot-header {
  background: linear-gradient(135deg, var(--ifm-color-primary) 0%, #667eea 100%);
  color: white;
  padding: 16px;
  display: flex;
  align-items: center;
  justify-content: space-between;
}

.chatbot-header-content {
  display: flex;
  align-items: center;
  gap: 12px;
}

.chatbot-icon-large {
  width: 36px;
  height: 36px;
  background: rgba(255, 255, 255, 0.2);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
}

.chatbot-icon-large svg {
  width: 20px;
  height: 20px;
}

.chatbot-title h3 {
  margin: 0;
  font-size: 16px;
  font-weight: 600;
}

.chatbot-status {
  margin: 0;
  font-size: 12px;
  opacity: 0.9;
}

.chatbot-close-button {
  background: none;
  border: none;
  color: white;
  cursor: pointer;
  padding: 4px;
  border-radius: 4px;
  transition: background-color 0.2s ease;
}

.chatbot-close-button:hover {
  background: rgba(255, 255, 255, 0.2);
}

.chatbot-messages {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
  display: flex;
  flex-direction: column;
  gap: 12px;
  background-color: var(--ifm-card-background-color);
}

.message {
  max-width: 80%;
  display: flex;
}

.message-user {
  align-self: flex-end;
}

.message-bot {
  align-self: flex-start;
}

.message-content {
  padding: 12px 16px;
  border-radius: 18px;
  position: relative;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.message-user .message-content {
  background: var(--ifm-color-primary);
  color: white;
  border-bottom-right-radius: 4px;
}

.message-bot .message-content {
  background: var(--ifm-color-emphasis-100);
  color: var(--ifm-font-color-base);
  border-bottom-left-radius: 4px;
}

.message-text {
  margin-bottom: 4px;
  line-height: 1.4;
}

.message-time {
  font-size: 11px;
  opacity: 0.7;
  text-align: right;
}

.message-typing {
  display: flex;
  gap: 4px;
  align-items: center;
}

.typing-dot {
  width: 8px;
  height: 8px;
  background: var(--ifm-color-emphasis-600);
  border-radius: 50%;
  animation: typing 1.4s infinite ease-in-out;
}

.typing-dot:nth-child(1) { animation-delay: 0s; }
.typing-dot:nth-child(2) { animation-delay: 0.2s; }
.typing-dot:nth-child(3) { animation-delay: 0.4s; }

@keyframes typing {
  0%, 60%, 100% { transform: translateY(0); }
  30% { transform: translateY(-5px); }
}

.chatbot-input-form {
  display: flex;
  padding: 16px;
  background: var(--ifm-background-color);
  border-top: 1px solid var(--ifm-toc-border-color);
  gap: 8px;
}

.chatbot-input {
  flex: 1;
  padding: 12px 16px;
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 24px;
  background: var(--ifm-input-background-color);
  color: var(--ifm-font-color-base);
  font-size: 14px;
  outline: none;
  transition: border-color 0.2s ease;
}

.chatbot-input:focus {
  border-color: var(--ifm-color-primary);
  box-shadow: 0 0 0 2px rgba(37, 194, 160, 0.2);
}

.chatbot-send-button {
  width: 44px;
  height: 44px;
  border-radius: 50%;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.2s ease;
}

.chatbot-send-button:hover:not(:disabled) {
  background: var(--ifm-color-primary-dark);
  transform: scale(1.05);
}

.chatbot-send-button:disabled {
  background: var(--ifm-color-emphasis-300);
  cursor: not-allowed;
}

/* Responsive design */
@media (max-width: 768px) {
  .chatbot-window {
    width: calc(100vw - 40px);
    height: 50vh;
    right: 20px;
    bottom: 20px;
  }

  .chatbot-float-button {
    bottom: 20px;
    right: 20px;
    width: 50px;
    height: 50px;
  }

  .chatbot-icon {
    width: 24px;
    height: 24px;
  }

  .chatbot-icon svg {
    width: 20px;
    height: 20px;
  }
}

/* Dark mode specific adjustments */
html[data-theme="dark"] .chatbot-window {
  background: #1e1e1e;
  border: 1px solid #333333;
}

html[data-theme="dark"] .chatbot-messages {
  background-color: #1e1e1e;
}

html[data-theme="dark"] .message-bot .message-content {
  background: #2d2d2d;
  color: #e0e0e0;
}

/* Scrollbar styling */
.chatbot-messages::-webkit-scrollbar {
  width: 6px;
}

.chatbot-messages::-webkit-scrollbar-track {
  background: transparent;
}

.chatbot-messages::-webkit-scrollbar-thumb {
  background: var(--ifm-color-emphasis-300);
  border-radius: 3px;
}

.chatbot-messages::-webkit-scrollbar-thumb:hover {
  background: var(--ifm-color-emphasis-500);
}
`;

  // Write the files
  fs.writeFileSync(chatbotComponentPath, chatbotComponent);
  fs.writeFileSync(chatbotCssPath, chatbotCss);

  console.log('Chatbot window enhanced successfully!');
  console.log(`Created ${chatbotComponentPath}`);
  console.log(`Created ${chatbotCssPath}`);

  // Update docusaurus.config.js to include the chatbot component in the layout
  const configPath = path.join(frontendDir, 'docusaurus.config.js');
  if (fs.existsSync(configPath)) {
    let configContent = fs.readFileSync(configPath, 'utf8');

    // Add import for the chatbot component if not already present
    if (!configContent.includes('./src/components/ChatbotWindow')) {
      // We'll add the chatbot to the layout in the theme configuration
      if (configContent.includes('themeConfig: {')) {
        // Add a custom layout or modify the existing one to include the chatbot
        // For now, we'll just note that the component is available
        console.log('Chatbot component created. You can import and use it in your layouts as needed.');
      }
    }
  }

  // Update App.js or main layout to include the chatbot
  const appPath = path.join(frontendDir, 'src', 'App.js');
  if (fs.existsSync(appPath)) {
    let appContent = fs.readFileSync(appPath, 'utf8');

    if (!appContent.includes('ChatbotWindow')) {
      // Add import and component to App.js if it exists
      appContent = `import ChatbotWindow from './components/ChatbotWindow';\n` + appContent;

      // Add the component to the render output (this is a basic approach)
      // In a Docusaurus app, this would typically be added to a layout component
      if (appContent.includes('</div>') && !appContent.includes('<ChatbotWindow')) {
        appContent = appContent.replace(
          /(<\/div>)/,
          '  <ChatbotWindow />\n$1'
        );
      }

      fs.writeFileSync(appPath, appContent);
      console.log('Added ChatbotWindow to App.js');
    }
  } else {
    // Create a layout wrapper that includes the chatbot
    const layoutPath = path.join(frontendDir, 'src', 'components', 'LayoutWrapper.jsx');
    const layoutContent = `import React from 'react';
import ChatbotWindow from './ChatbotWindow';

const LayoutWrapper = ({ children }) => {
  return (
    <>
      {children}
      <ChatbotWindow />
    </>
  );
};

export default LayoutWrapper;
`;

    fs.writeFileSync(layoutPath, layoutContent);
    console.log(`Created layout wrapper at ${layoutPath}`);
  }
}

// Export the function
module.exports = { enhanceChatbot };

// If running directly
if (require.main === module) {
  const frontendDir = process.argv[2] || './frontend';
  enhanceChatbot(frontendDir);
}