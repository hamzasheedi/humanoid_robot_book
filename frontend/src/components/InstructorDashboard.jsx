import React, { useState, useEffect } from 'react';

const InstructorDashboard = ({ sessionId }) => {
  const [stats, setStats] = useState({});
  const [questions, setQuestions] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // In a real implementation, this would fetch from the backend
    // For now, we'll use mock data
    const mockStats = {
      totalQuestions: 124,
      accuracyRate: 87.5,
      avgResponseTime: 1.2, // seconds
      mostAskedTopics: ['ROS 2 Architecture', 'Gazebo Simulation', 'Path Planning'],
      studentSatisfaction: 4.2 // out of 5
    };

    const mockQuestions = [
      { id: 1, text: "Explain the difference between ROS 1 and ROS 2", accuracy: 95, difficulty: 'high' },
      { id: 2, text: "How does the path planning algorithm work?", accuracy: 89, difficulty: 'high' },
      { id: 3, text: "What are the basic components of a robot?", accuracy: 98, difficulty: 'low' },
    ];

    setStats(mockStats);
    setQuestions(mockQuestions);
    setLoading(false);
  }, [sessionId]);

  if (loading) {
    return <div>Loading instructor dashboard...</div>;
  }

  return (
    <div className="instructor-dashboard">
      <h2>Instructor Dashboard</h2>
      
      <div className="dashboard-stats">
        <div className="stat-card">
          <h3>{stats.totalQuestions}</h3>
          <p>Total Questions Asked</p>
        </div>
        <div className="stat-card">
          <h3>{stats.accuracyRate}%</h3>
          <p>Accuracy Rate</p>
        </div>
        <div className="stat-card">
          <h3>{stats.avgResponseTime}s</h3>
          <p>Avg. Response Time</p>
        </div>
        <div className="stat-card">
          <h3>{stats.studentSatisfaction}/5</h3>
          <p>Student Satisfaction</p>
        </div>
      </div>

      <div className="most-asked-topics">
        <h3>Most Asked Topics</h3>
        <ul>
          {stats.mostAskedTopics.map((topic, index) => (
            <li key={index}>{topic}</li>
          ))}
        </ul>
      </div>

      <div className="question-analysis">
        <h3>Recent Questions Analysis</h3>
        <table>
          <thead>
            <tr>
              <th>Question</th>
              <th>Accuracy</th>
              <th>Difficulty</th>
            </tr>
          </thead>
          <tbody>
            {questions.map((question) => (
              <tr key={question.id}>
                <td>{question.text}</td>
                <td>{question.accuracy}%</td>
                <td>
                  <span className={`difficulty ${question.difficulty}`}>
                    {question.difficulty}
                  </span>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default InstructorDashboard;