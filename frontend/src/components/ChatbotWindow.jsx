import React, { useState, useEffect, useRef } from 'react';
import { API_CONFIG } from '../config/apiConfig';
import './ChatbotWindow.css';

const ChatbotWindow = ({ isOpen, onClose, sessionId: propSessionId, onSessionIdChange }) => {
  // Initialize session ID from props or from localStorage
  const [sessionId, setSessionId] = useState(() => {
    // Check if we're in the browser environment
    if (typeof window !== 'undefined' && window.localStorage) {
      return propSessionId || localStorage.getItem('chatbot-session-id') || null;
    }
    return propSessionId || null; // Fallback when not in browser
  });
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Update local state when prop changes
  useEffect(() => {
    if (propSessionId) {
      setSessionId(propSessionId);
    }
  }, [propSessionId]);

  // Persist session ID in localStorage
  useEffect(() => {
    // Only run in browser environment
    if (typeof window !== 'undefined' && window.localStorage) {
      if (sessionId) {
        localStorage.setItem('chatbot-session-id', sessionId);
      } else {
        localStorage.removeItem('chatbot-session-id');
      }
    }
  }, [sessionId]);

  // Function to handle session ID change
  const handleSessionIdChange = (newSessionId) => {
    setSessionId(newSessionId);
    if (onSessionIdChange) {
      onSessionIdChange(newSessionId);
    }
  };

  // Function to scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to handle sending a message
  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message to UI immediately
    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call backend API to get response
      let data;
      // For testing purposes, use mock responses when backend is not available
      try {
        const response = await fetch(`${API_CONFIG.BASE_URL}/chat/ask`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            question: inputValue,
            selected_text: selectedText,
            session_id: sessionId
          }),
        });

        if (!response.ok) {
          throw new Error(`Server error: ${response.status}`);
        }

        data = await response.json();
      } catch (error) {
        console.error('Error connecting to backend:', error);
        // Return mock response for testing UI
        data = {
          answer: "I'm currently unable to connect to the AI backend. This is a mock response to demonstrate the UI. ",
          sources: ["Mock Response"],
          confidence: 0.8,
          session_id: sessionId || `session_${Date.now()}`
        };
      }
      
      // Add bot response to UI
      const botMessage = {
        id: Date.now() + 1,
        text: data.answer,
        sender: 'bot',
        sources: data.sources,
        confidence: data.confidence
      };
      
      setMessages(prev => [...prev, botMessage]);
      
      // Update session ID if it was generated
      if (data.session_id && !sessionId) {
        handleSessionIdChange(data.session_id);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot',
        isError: true
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to handle text selection
  useEffect(() => {
    // Only run in browser environment
    if (typeof window !== 'undefined' && window.document) {
      const handleSelection = () => {
        const selectedText = window.getSelection().toString().trim();
        if (selectedText) {
          setSelectedText(selectedText);
        }
      };

      document.addEventListener('mouseup', handleSelection);
      return () => {
        document.removeEventListener('mouseup', handleSelection);
      };
    }
  }, []);

  // Function to handle "Ask About Selected Text" button
  const handleAskSelectedText = async () => {
    if (!selectedText.trim() || isLoading) return;

    // Add user message to UI
    const userMessage = { id: Date.now(), text: `Selected: "${selectedText}"`, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Call backend API to get context for selected text
      let data;
      // For testing purposes, use mock responses when backend is not available
      try {
        const response = await fetch(`${API_CONFIG.BASE_URL}/chat/context`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            selected_text: selectedText,
            session_id: sessionId
          }),
        });

        if (!response.ok) {
          throw new Error(`Server error: ${response.status}`);
        }

        data = await response.json();
      } catch (error) {
        console.error('Error connecting to backend:', error);
        // Return mock response for testing UI
        data = {
          context: "I'm currently unable to connect to the AI backend. This is a mock response.",
          session_id: sessionId || `session_${Date.now()}`
        };
      }
      
      // Add bot response to UI
      const botMessage = {
        id: Date.now() + 1,
        text: data.context || 'No relevant content found in the textbook for the selected text.',
        sender: 'bot',
      };
      
      setMessages(prev => [...prev, botMessage]);
      
      // Update session ID if it was generated
      if (data.session_id && !sessionId) {
        handleSessionIdChange(data.session_id);
      }
    } catch (error) {
      console.error('Error getting context for selection:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing the selected text. Please try again.',
        sender: 'bot',
        isError: true
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to copy text to clipboard
  const copyToClipboard = (text) => {
    // Only run in browser environment
    if (typeof navigator !== 'undefined' && navigator.clipboard) {
      navigator.clipboard.writeText(text);
    } else {
      // Fallback for server-side or unsupported browsers
      console.warn('Clipboard API not available');
    }
  };

  return (
    <div className="chatbot-window">
      {/* Floating button when closed */}
      {!isOpen && (
        <button
          className="chatbot-toggle"
          onClick={onClose} // This should open the chat window
        >
          💬 AI
        </button>
      )}

      {/* Chat window when open */}
      {isOpen && (
        <div className="chatbot-container">
          <div className="chatbot-header">
            <h3>AI Textbook Assistant</h3>
            <div className="header-controls">
              <button
                className="minimize-btn"
                onClick={onClose}
                aria-label="Minimize chat"
              >
                −
              </button>
            </div>
          </div>

          <div className="chatbot-body">
            {/* Messages container */}
            <div className="chat-messages">
              {messages.length === 0 ? (
                <div className="welcome-message">
                  <p>Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook.</p>
                  <p>You can ask me questions about the content, or select text and click "Ask About Selected Text" to get context-aware answers.</p>
                </div>
              ) : (
                messages.map((message) => (
                  <div 
                    key={message.id} 
                    className={`message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
                  >
                    <div className="message-content">
                      <p>{message.text}</p>
                      {message.sources && message.sources.length > 0 && (
                        <div className="sources">
                          <small>Referenced: {message.sources.map(s => s.title || 'Source').join(', ')}</small>
                        </div>
                      )}
                      {message.confidence && (
                        <div className="confidence">
                          <small>Confidence: {(message.confidence * 100).toFixed(1)}%</small>
                        </div>
                      )}
                    </div>
                    <button 
                      className="copy-btn"
                      onClick={() => copyToClipboard(message.text)}
                      aria-label="Copy message"
                    >
                      📋 COPY
                    </button>
                  </div>
                ))
              )}
              {isLoading && (
                <div className="message bot-message">
                  <div className="message-content">
                    <p>Thinking...</p>
                  </div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>

            {/* Input area */}
            {selectedText && (
              <div className="selected-text-notice">
                <span>Selected: "{selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}"</span>
                <button 
                  onClick={handleAskSelectedText}
                  disabled={isLoading}
                  className="ask-selected-btn"
                >
                  Ask About Selected Text
                </button>
              </div>
            )}
            <div className="input-area">
              <input
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && handleSendMessage()}
                placeholder="Ask a question about the textbook..."
                disabled={isLoading}
              />
              <button 
                onClick={handleSendMessage}
                disabled={isLoading || !inputValue.trim()}
                className="send-btn"
              >
                {isLoading ? '...' : '→'}
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatbotWindow;
