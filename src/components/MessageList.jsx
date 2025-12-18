/**
 * Message List Component
 *
 * Displays the chat history with user and agent messages
 */

import React from 'react';

const MessageList = ({ messages }) => {
  if (!messages || messages.length === 0) {
    return (
      <div className="message-list">
        <div className="welcome-message">
          <p>Welcome to the RAG Agent Chat!</p>
          <p>Ask me anything about the book content. You can also select text on the page to provide context for your questions.</p>
        </div>
      </div>
    );
  }

  return (
    <div className="message-list">
      {messages.map((message) => (
        <div
          key={message.id}
          className={`message-item ${message.sender === 'user' ? 'user-message' : 'agent-message'}`}
        >
          <div className="message-header">
            <span className="sender">{message.sender === 'user' ? 'You' : 'RAG Agent'}</span>
            <span className="timestamp">
              {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </span>
          </div>

          <div className="message-content">
            {message.content}

            {message.selectedText && (
              <div className="selected-text-context">
                <strong>Context:</strong> "{message.selectedText.substring(0, 100)}..."
              </div>
            )}

            {message.sources && message.sources.length > 0 && (
              <div className="sources">
                <strong>Sources:</strong>
                <ul>
                  {message.sources.map((source, index) => (
                    <li key={index}>
                      <a href={source.url} target="_blank" rel="noopener noreferrer">
                        {source.page || source.title || `Source ${index + 1}`}
                      </a>
                    </li>
                  ))}
                </ul>
              </div>
            )}

            {message.confidence !== undefined && (
              <div className="confidence-score">
                <small>Confidence: {(message.confidence * 100).toFixed(1)}%</small>
              </div>
            )}
          </div>
        </div>
      ))}

      <style jsx>{`
        .message-list {
          flex: 1;
          overflow-y: auto;
          padding: 16px;
          display: flex;
          flex-direction: column;
          gap: 16px;
        }

        .welcome-message {
          text-align: center;
          padding: 40px 20px;
          color: #666;
          font-style: italic;
        }

        .message-item {
          display: flex;
          flex-direction: column;
          max-width: 85%;
          padding: 12px 16px;
          border-radius: 8px;
          margin-bottom: 8px;
          position: relative;
          animation: fadeIn 0.3s ease-out;
        }

        @keyframes fadeIn {
          from { opacity: 0; transform: translateY(10px); }
          to { opacity: 1; transform: translateY(0); }
        }

        .user-message {
          align-self: flex-end;
          background-color: #dcf8c6;
          border: 1px solid #d0d0d0;
        }

        .agent-message {
          align-self: flex-start;
          background-color: #ffffff;
          border: 1px solid #e0e0e0;
          box-shadow: 0 1px 2px rgba(0,0,0,0.1);
        }

        .message-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          margin-bottom: 8px;
          font-size: 0.9em;
          color: #666;
        }

        .sender {
          font-weight: bold;
          color: #333;
        }

        .message-content {
          word-wrap: break-word;
          line-height: 1.5;
        }

        .selected-text-context {
          margin-top: 8px;
          padding: 8px;
          background-color: #f0f8ff;
          border-left: 3px solid #007bff;
          font-size: 0.9em;
          color: #555;
        }

        .sources {
          margin-top: 8px;
          padding-top: 8px;
          border-top: 1px dashed #ddd;
        }

        .sources ul {
          margin: 4px 0;
          padding-left: 20px;
        }

        .sources li {
          margin-bottom: 4px;
        }

        .sources a {
          color: #007bff;
          text-decoration: none;
        }

        .sources a:hover {
          text-decoration: underline;
        }

        .confidence-score {
          margin-top: 8px;
          text-align: right;
          font-size: 0.8em;
          color: #888;
        }
      `}</style>
    </div>
  );
};

export default MessageList;