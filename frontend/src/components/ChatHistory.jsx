import React, { useState, useEffect } from 'react';
import './ChatHistory.css';

const ChatHistory = ({ sessionId, onClose }) => {
  const [history, setHistory] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [searchTerm, setSearchTerm] = useState('');

  useEffect(() => {
    if (sessionId) {
      fetchHistory();
    }
  }, [sessionId]);

  const fetchHistory = async () => {
    try {
      setLoading(true);
      // In a real implementation, this would call the backend API
      // For now, using mock data
      const mockHistory = [
        {
          id: 1,
          question: "What is ROS 2?",
          answer: "ROS 2 (Robot Operating System 2) is a set of libraries and tools that help you build robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.",
          timestamp: new Date(Date.now() - 3600000), // 1 hour ago
          sources: [{ title: "ROS 2 Introduction", module: "ROS 2 Fundamentals" }]
        },
        {
          id: 2,
          question: "How does the navigation stack work?",
          answer: "The ROS 2 Navigation Stack is a collection of packages that implement navigation functionality for mobile robots. It includes modules for localization, mapping, path planning, and obstacle avoidance.",
          timestamp: new Date(Date.now() - 1800000), // 30 minutes ago
          sources: [{ title: "Navigation in ROS 2", module: "ROS 2 Applications" }]
        },
        {
          id: 3,
          question: "What sensors are used in humanoid robots?",
          answer: "Humanoid robots typically use various sensors including cameras for vision, IMUs for balance and orientation, force/torque sensors for touch, and LiDAR for environment mapping.",
          timestamp: new Date(Date.now() - 600000), // 10 minutes ago
          sources: [{ title: "Sensors in Robotics", module: "Sensory Systems" }]
        }
      ];
      
      setHistory(mockHistory);
    } catch (err) {
      setError('Failed to load chat history');
      console.error('Error fetching chat history:', err);
    } finally {
      setLoading(false);
    }
  };

  const filteredHistory = history.filter(item => 
    item.question.toLowerCase().includes(searchTerm.toLowerCase()) ||
    item.answer.toLowerCase().includes(searchTerm.toLowerCase())
  );

  if (loading) {
    return <div className="chat-history">Loading chat history...</div>;
  }

  if (error) {
    return <div className="chat-history error">Error: {error}</div>;
  }

  return (
    <div className="chat-history">
      <div className="history-header">
        <h3>Chat History</h3>
        <button className="close-btn" onClick={onClose}>×</button>
      </div>
      
      <div className="history-search">
        <input
          type="text"
          placeholder="Search in chat history..."
          value={searchTerm}
          onChange={(e) => setSearchTerm(e.target.value)}
        />
      </div>
      
      <div className="history-items">
        {filteredHistory.length > 0 ? (
          filteredHistory.map((item) => (
            <div key={item.id} className="history-item">
              <div className="history-question">
                <strong>Q:</strong> {item.question}
                <div className="timestamp">{item.timestamp.toLocaleString()}</div>
              </div>
              <div className="history-answer">
                <strong>A:</strong> {item.answer}
                {item.sources && item.sources.length > 0 && (
                  <div className="sources">
                    Sources: {item.sources.map((source, idx) => 
                      <span key={idx} className="source-tag">
                        {source.module ? `${source.module} - ` : ''}{source.title}
                      </span>
                    )}
                  </div>
                )}
              </div>
            </div>
          ))
        ) : (
          <div className="no-history">No chat history found</div>
        )}
      </div>
    </div>
  );
};

export default ChatHistory;