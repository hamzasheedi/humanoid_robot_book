import React, { useState, useEffect, useRef } from 'react';
import './ChatbotWindow.css';

// Set backend URL from environment variable
const BACKEND_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';

const ChatbotWindow = ({ isOpen, onClose, sessionId: propSessionId, onSessionIdChange }) => {
  const [sessionId, setSessionId] = useState(() => {
    if (typeof window !== 'undefined' && window.localStorage) {
      return propSessionId || localStorage.getItem('chatbot-session-id') || null;
    }
    return propSessionId || null;
  });
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  useEffect(() => { if (propSessionId) setSessionId(propSessionId); }, [propSessionId]);

  useEffect(() => {
    if (typeof window !== 'undefined' && window.localStorage) {
      if (sessionId) localStorage.setItem('chatbot-session-id', sessionId);
      else localStorage.removeItem('chatbot-session-id');
    }
  }, [sessionId]);

  const handleSessionIdChange = (newSessionId) => {
    setSessionId(newSessionId);
    if (onSessionIdChange) onSessionIdChange(newSessionId);
  };

  const scrollToBottom = () => { messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' }); };
  useEffect(() => { scrollToBottom(); }, [messages]);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      let data;
      try {
        const response = await fetch(`${BACKEND_URL}/chat/ask`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            question: inputValue,
            selected_text: selectedText,
            session_id: sessionId
          }),
        });

        if (!response.ok) throw new Error(`Server error: ${response.status}`);
        data = await response.json();
      } catch (error) {
        console.error('Error connecting to backend:', error);
        data = {
          answer: "Unable to connect to backend. This is a mock response.",
          sources: ["Mock Response"],
          confidence: 0.5,
          session_id: sessionId || `session_${Date.now()}`
        };
      }

      const botMessage = {
        id: Date.now() + 1,
        text: data.answer,
        sender: 'bot',
        sources: data.sources,
        confidence: data.confidence
      };
      setMessages(prev => [...prev, botMessage]);

      if (data.session_id && !sessionId) handleSessionIdChange(data.session_id);
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages(prev => [...prev, { id: Date.now()+1, text: 'Error processing request.', sender: 'bot', isError: true }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleAskSelectedText = async () => {
    if (!selectedText.trim() || isLoading) return;

    const userMessage = { id: Date.now(), text: `Selected: "${selectedText}"`, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      let data;
      try {
        const response = await fetch(`${BACKEND_URL}/chat/context`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ selected_text: selectedText, session_id: sessionId }),
        });

        if (!response.ok) throw new Error(`Server error: ${response.status}`);
        data = await response.json();
      } catch (error) {
        console.error('Error connecting to backend:', error);
        data = { context: "Unable to connect to backend. This is a mock response.", session_id: sessionId || `session_${Date.now()}` };
      }

      const botMessage = {
        id: Date.now() + 1,
        text: data.context || 'No relevant content found.',
        sender: 'bot',
      };
      setMessages(prev => [...prev, botMessage]);

      if (data.session_id && !sessionId) handleSessionIdChange(data.session_id);
    } catch (error) {
      console.error('Error getting context:', error);
      setMessages(prev => [...prev, { id: Date.now()+1, text: 'Error processing selected text.', sender: 'bot', isError: true }]);
    } finally {
      setIsLoading(false);
    }
  };

  const copyToClipboard = (text) => { if (navigator?.clipboard) navigator.clipboard.writeText(text); };

  return (
    <div className="chatbot-window">
      {!isOpen && <button className="chatbot-toggle" onClick={onClose}>💬 AI Tutor</button>}

      {isOpen && (
        <div className="chatbot-container">
          <div className="chatbot-header">
            <h3>AI Textbook Assistant</h3>
            <div className="header-controls">
              <button className="minimize-btn" onClick={onClose}>−</button>
            </div>
          </div>

          <div className="chatbot-body">
            <div className="chat-messages">
              {messages.length === 0 ? (
                <div className="welcome-message">
                  <p>Hello! I'm your AI assistant for the textbook.</p>
                </div>
              ) : messages.map(message => (
                <div key={message.id} className={`message ${message.sender==='user'?'user-message':'bot-message'}`}>
                  <div className="message-content">
                    <p>{message.text}</p>
                  </div>
                  <button className="copy-btn" onClick={()=>copyToClipboard(message.text)}>📋</button>
                </div>
              ))}
              {isLoading && <div className="message bot-message"><p>Thinking...</p></div>}
              <div ref={messagesEndRef}/>
            </div>

            {selectedText && (
              <div className="selected-text-notice">
                <span>Selected: "{selectedText.substring(0,60)}{selectedText.length>60?'...':''}"</span>
                <button onClick={handleAskSelectedText} disabled={isLoading}>Ask About Selected Text</button>
              </div>
            )}

            <div className="input-area">
              <input
                type="text"
                value={inputValue}
                onChange={e=>setInputValue(e.target.value)}
                onKeyPress={e=>e.key==='Enter' && handleSendMessage()}
                placeholder="Ask a question..."
                disabled={isLoading}
              />
              <button onClick={handleSendMessage} disabled={isLoading||!inputValue.trim()}>{isLoading?'...':'→'}</button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatbotWindow;
