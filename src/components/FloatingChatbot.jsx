/**
 * Floating Chatbot Component
 *
 * A floating chat widget that appears as a side-mounted chat interface
 * accessible from all pages in the Docusaurus application.
 */

import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import apiClient from '../utils/apiClient';
import textSelectionUtility from '../utils/textSelection';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [contextMetadata, setContextMetadata] = useState({});

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Initialize text selection utility
  useEffect(() => {
    if (isOpen) {
      // Attach text selection listeners to the document when chat is open
      textSelectionUtility.attachSelectionListeners(document.body);

      const handleTextSelection = (event) => {
        const selection = event.detail;
        setSelectedText(selection.text);

        // Update context metadata based on selection
        setContextMetadata({
          page_url: selection.page_url,
          page_title: selection.page_title,
          module: selection.page_title, // Simplified mapping
        });

        console.log('Text selected for context:', selection.text.substring(0, 50) + '...');
      };

      document.addEventListener('textSelected', handleTextSelection);

      // Cleanup event listener
      return () => {
        document.removeEventListener('textSelected', handleTextSelection);
      };
    }
  }, [isOpen]);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      setTimeout(() => {
        inputRef.current.focus();
      }, 100);
    }
  }, [isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      // Add a welcome message when opening
      if (messages.length === 0) {
        setMessages([
          {
            id: 'welcome',
            content: 'Hello! I\'m your RAG Agent. Ask me anything about the book content. You can also select text on the page to provide context for your questions.',
            sender: 'agent',
            timestamp: new Date().toISOString()
          }
        ]);
      }
    }
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();

    if (!inputValue.trim()) {
      setError('Message cannot be empty');
      return;
    }

    try {
      // Add user message to chat
      const userMessage = {
        id: `msg-${Date.now()}`,
        content: inputValue,
        sender: 'user',
        timestamp: new Date().toISOString(),
        selectedText: selectedText || null
      };

      setMessages(prev => [...prev, userMessage]);
      setInputValue('');
      setIsLoading(true);
      setError(null);

      // Prepare context metadata
      const messageContext = {
        query: inputValue,
        selected_text: selectedText,
        context_metadata: contextMetadata,
        user_preferences: {
          temperature: 0.7,
          max_tokens: 500
        }
      };

      // Send query to backend
      const response = await apiClient.sendQuery(messageContext);

      if (response.success) {
        // Add agent response to chat
        const agentMessage = {
          id: `msg-${Date.now()}-agent`,
          content: response.message,
          sender: 'agent',
          timestamp: new Date().toISOString(),
          sources: response.sources || [],
          confidence: response.confidence || 0.0
        };

        setMessages(prev => [...prev, agentMessage]);
      } else {
        setError(response.error?.message || 'Failed to get response from agent');
      }
    } catch (err) {
      console.error('Error sending message:', err);
      setError(err.message || 'An error occurred while sending the message');
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after sending
    }
  };

  const handleClearChat = () => {
    setMessages([]);
    setError(null);
  };

  const handleInputChange = (e) => {
    setInputValue(e.target.value);
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage(e);
    }
  };

  return (
    <>
      {/* Floating Chat Icon */}
      <button
        className="floating-chat-icon"
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
          className="chat-icon-svg"
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
      </button>

      {/* Floating Chat Window - Only render when open */}
      {isOpen && (
        <div className="floating-chat-window">
          <div className="chat-header">
            <h3>RAG Agent Chat</h3>
            <div className="header-actions">
              <button onClick={handleClearChat} className="clear-chat-btn" title="Clear chat">
                Clear
              </button>
              <button onClick={toggleChat} className="close-chat-btn" title="Close chat">
                ×
              </button>
            </div>
          </div>

          <div className="chat-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.sender}`}
                style={{
                  alignSelf: message.sender === 'user' ? 'flex-end' : 'flex-start',
                  backgroundColor: message.sender === 'user' ? '#e3f2fd' : '#f5f5f5',
                  borderRadius: '12px',
                  padding: '8px 12px',
                  maxWidth: '80%',
                  margin: '4px 0',
                }}
              >
                <div className="message-content">{message.content}</div>
                {message.sources && message.sources.length > 0 && (
                  <div className="message-sources">
                    Sources: {message.sources.map((source, idx) => (
                      <span key={idx} className="source-tag">
                        {source.title || `Source ${idx + 1}`}
                      </span>
                    ))}
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div
                className="message agent"
                style={{
                  alignSelf: 'flex-start',
                  backgroundColor: '#f5f5f5',
                  borderRadius: '12px',
                  padding: '8px 12px',
                  maxWidth: '80%',
                  margin: '4px 0',
                }}
              >
                <div className="loading-indicator">Thinking...</div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {error && (
            <div className="chat-error">
              <span className="error-text">{error}</span>
            </div>
          )}

          <form onSubmit={handleSendMessage} className="chat-input-form">
            {selectedText && (
              <div className="selected-text-preview">
                <small>Context: "{selectedText.substring(0, 50)}..."</small>
                <button
                  type="button"
                  onClick={() => setSelectedText('')}
                  className="clear-selection-btn"
                >
                  ×
                </button>
              </div>
            )}
            <div className="input-container">
              <textarea
                ref={inputRef}
                value={inputValue}
                onChange={handleInputChange}
                onKeyDown={handleKeyDown}
                placeholder="Type your question here... (Press Enter to send, Shift+Enter for new line)"
                className="chat-input"
                rows={1}
                disabled={isLoading}
              />
              <button
                type="submit"
                disabled={!inputValue.trim() || isLoading}
                className="send-button"
              >
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  width="16"
                  height="16"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  className="send-icon"
                >
                  <line x1="22" y1="2" x2="11" y2="13" />
                  <polygon points="22 2 15 22 11 13 2 9 22 2" />
                </svg>
              </button>
            </div>
          </form>
        </div>
      )}

      <style jsx>{`
        .floating-chat-icon {
          position: fixed;
          bottom: 20px;
          right: 20px;
          width: 60px;
          height: 60px;
          border-radius: 50%;
          background-color: #1a73e8;
          color: white;
          border: none;
          cursor: pointer;
          display: flex;
          align-items: center;
          justify-content: center;
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
          z-index: 1000;
          transition: all 0.3s ease;
        }

        .floating-chat-icon:hover {
          background-color: #0d5cb6;
          transform: scale(1.05);
          box-shadow: 0 6px 16px rgba(0, 0, 0, 0.2);
        }

        .chat-icon-svg {
          width: 28px;
          height: 28px;
        }

        .floating-chat-window {
          position: fixed;
          bottom: 90px;
          right: 20px;
          width: 380px;
          height: 500px;
          max-height: 80vh;
          background-color: white;
          border-radius: 12px;
          box-shadow: 0 8px 32px rgba(0, 0, 0, 0.2);
          display: flex;
          flex-direction: column;
          z-index: 1000;
          overflow: hidden;
          border: 1px solid #e0e0e0;
        }

        .chat-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 16px;
          background-color: #f8f9fa;
          border-bottom: 1px solid #e0e0e0;
        }

        .chat-header h3 {
          margin: 0;
          font-size: 16px;
          font-weight: 600;
          color: #333;
        }

        .header-actions {
          display: flex;
          gap: 8px;
        }

        .clear-chat-btn, .close-chat-btn {
          background: none;
          border: 1px solid #ddd;
          border-radius: 4px;
          padding: 4px 8px;
          cursor: pointer;
          font-size: 12px;
        }

        .close-chat-btn {
          width: 28px;
          height: 28px;
          display: flex;
          align-items: center;
          justify-content: center;
          padding: 0;
        }

        .chat-messages {
          flex: 1;
          overflow-y: auto;
          padding: 16px;
          display: flex;
          flex-direction: column;
          background-color: #fafafa;
        }

        .message {
          margin-bottom: 8px;
          word-wrap: break-word;
          overflow-wrap: break-word;
        }

        .message-content {
          font-size: 14px;
          line-height: 1.4;
        }

        .message-sources {
          margin-top: 4px;
          font-size: 11px;
          color: #666;
        }

        .source-tag {
          display: inline-block;
          background-color: #e3f2fd;
          color: #1a73e8;
          padding: 2px 6px;
          border-radius: 10px;
          margin-right: 4px;
          font-size: 10px;
        }

        .loading-indicator {
          font-style: italic;
          color: #666;
        }

        .chat-error {
          background-color: #ffebee;
          color: #c62828;
          padding: 8px 16px;
          font-size: 12px;
          border-top: 1px solid #ffcdd2;
        }

        .error-text {
          word-break: break-word;
        }

        .chat-input-form {
          padding: 12px;
          background-color: white;
          border-top: 1px solid #e0e0e0;
        }

        .selected-text-preview {
          display: flex;
          align-items: center;
          background-color: #e3f2fd;
          border: 1px solid #bbdefb;
          border-radius: 6px;
          padding: 6px 8px;
          margin-bottom: 8px;
          font-size: 12px;
        }

        .clear-selection-btn {
          background: none;
          border: none;
          color: #666;
          cursor: pointer;
          margin-left: 8px;
          font-size: 14px;
          padding: 0;
          width: 18px;
          height: 18px;
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .input-container {
          display: flex;
          gap: 8px;
          align-items: flex-end;
        }

        .chat-input {
          flex: 1;
          border: 1px solid #ddd;
          border-radius: 8px;
          padding: 10px 12px;
          resize: none;
          font-family: inherit;
          font-size: 14px;
          max-height: 100px;
          min-height: 40px;
          outline: none;
        }

        .chat-input:focus {
          border-color: #1a73e8;
          box-shadow: 0 0 0 2px rgba(26, 115, 232, 0.2);
        }

        .send-button {
          width: 36px;
          height: 36px;
          border: 1px solid #ddd;
          border-radius: 8px;
          background-color: #f5f5f5;
          cursor: pointer;
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .send-button:disabled {
          opacity: 0.5;
          cursor: not-allowed;
        }

        .send-icon {
          width: 16px;
          height: 16px;
        }

        /* Responsive styles */
        @media (max-width: 768px) {
          .floating-chat-window {
            width: calc(100vw - 40px);
            height: 50vh;
            bottom: 90px;
            right: 20px;
            left: 20px;
          }

          .floating-chat-icon {
            width: 50px;
            height: 50px;
            bottom: 15px;
            right: 15px;
          }
        }

        @media (max-width: 480px) {
          .floating-chat-window {
            width: calc(100vw - 30px);
            height: 45vh;
            bottom: 85px;
            right: 15px;
            left: 15px;
          }

          .floating-chat-icon {
            width: 48px;
            height: 48px;
            bottom: 15px;
            right: 15px;
          }
        }
      `}</style>
    </>
  );
};

export default FloatingChatbot;