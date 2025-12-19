/**
 * Floating Chatbot Component
 *
 * A floating chat widget that appears as a side-mounted chat interface
 * accessible from all pages in the Docusaurus application.
 */

import React, { useState, useEffect, useRef } from 'react';
import apiClient from '../utils/apiClient';

const FloatingChatbot = ({ currentTheme = 'light' }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [contextMetadata, setContextMetadata] = useState({});

  const isDarkTheme = currentTheme === 'dark';

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Initialize text selection utility
  useEffect(() => {
    let textSelectionHandler = null;

    if (isOpen) {
      // Function to handle text selection
      const handleTextSelection = () => {
        const selection = window.getSelection();
        if (selection.toString().trim() !== '') {
          const selectedTextContent = selection.toString();

          // Get the selected text boundaries
          const range = selection.getRangeAt(0);
          const startOffset = range.startOffset;
          const endOffset = range.endOffset;

          // Store the selected text with metadata
          setSelectedText(selectedTextContent);

          // Update context metadata based on selection
          setContextMetadata({
            page_url: window.location.href,
            page_title: document.title,
            module: document.title, // Simplified mapping
            position_start: startOffset,
            position_end: endOffset
          });

          console.log('Text selected for context:', selectedTextContent.substring(0, 50) + '...');
        }
      };

      // Add event listeners for text selection
      document.addEventListener('mouseup', handleTextSelection);
      textSelectionHandler = handleTextSelection;

      // Cleanup event listener
      return () => {
        document.removeEventListener('mouseup', handleTextSelection);
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

  // Theme-aware styles
  const getThemeStyles = () => {
    if (isDarkTheme) {
      return {
        // Chat icon styles
        chatIcon: {
          backgroundColor: '#2563eb',
          color: 'white',
          boxShadow: '0 4px 12px rgba(0, 0, 0, 0.3)',
        },
        chatIconHover: {
          backgroundColor: '#1d4ed8',
        },
        // Chat window styles
        chatWindow: {
          backgroundColor: '#1f2937',
          border: '1px solid #374151',
          boxShadow: '0 8px 32px rgba(0, 0, 0, 0.4)',
        },
        // Header styles
        header: {
          backgroundColor: '#374151',
          borderBottom: '1px solid #4b5563',
          color: 'white',
        },
        // Messages container
        messagesContainer: {
          backgroundColor: '#111827',
          color: 'white',
        },
        // User message
        userMessage: {
          backgroundColor: '#3b82f6',
          color: 'white',
        },
        // Agent message
        agentMessage: {
          backgroundColor: '#374151',
          color: 'white',
        },
        // Input form
        inputForm: {
          backgroundColor: '#1f2937',
          borderTop: '1px solid #374151',
        },
        // Input field
        inputField: {
          backgroundColor: '#374151',
          borderColor: '#4b5563',
          color: 'white',
        },
        // Error message
        errorMessage: {
          backgroundColor: '#dc2626',
          color: 'white',
        },
        // Selected text preview
        selectedTextPreview: {
          backgroundColor: '#1e40af',
          borderColor: '#3b82f6',
        },
        // Source tags
        sourceTag: {
          backgroundColor: '#1e40af',
          color: '#93c5fd',
        },
        // Buttons
        button: {
          backgroundColor: '#374151',
          borderColor: '#4b5563',
          color: 'white',
        },
        clearButton: {
          backgroundColor: 'transparent',
          borderColor: '#4b5563',
          color: 'white',
        },
        // Robot topper styles
        robotTopper: {
          backgroundColor: '#10b981',
          color: 'white',
        }
      };
    } else {
      // Light theme styles (default)
      return {
        // Chat icon styles
        chatIcon: {
          backgroundColor: '#1a73e8',
          color: 'white',
          boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
        },
        chatIconHover: {
          backgroundColor: '#0d5cb6',
        },
        // Chat window styles
        chatWindow: {
          backgroundColor: 'white',
          border: '1px solid #e0e0e0',
          boxShadow: '0 8px 32px rgba(0, 0, 0, 0.2)',
        },
        // Header styles
        header: {
          backgroundColor: '#f8f9fa',
          borderBottom: '1px solid #e0e0e0',
          color: '#333',
        },
        // Messages container
        messagesContainer: {
          backgroundColor: '#fafafa',
          color: '#333',
        },
        // User message
        userMessage: {
          backgroundColor: '#e3f2fd',
          color: '#333',
        },
        // Agent message
        agentMessage: {
          backgroundColor: '#f5f5f5',
          color: '#333',
        },
        // Input form
        inputForm: {
          backgroundColor: 'white',
          borderTop: '1px solid #e0e0e0',
        },
        // Input field
        inputField: {
          backgroundColor: 'white',
          borderColor: '#ddd',
          color: '#333',
        },
        // Error message
        errorMessage: {
          backgroundColor: '#ffebee',
          color: '#c62828',
        },
        // Selected text preview
        selectedTextPreview: {
          backgroundColor: '#e3f2fd',
          borderColor: '#bbdefb',
        },
        // Source tags
        sourceTag: {
          backgroundColor: '#e3f2fd',
          color: '#1a73e8',
        },
        // Buttons
        button: {
          backgroundColor: '#f5f5f5',
          borderColor: '#ddd',
          color: '#333',
        },
        clearButton: {
          backgroundColor: 'transparent',
          borderColor: '#ddd',
          color: '#333',
        },
        // Robot topper styles
        robotTopper: {
          backgroundColor: '#059669',
          color: 'white',
        }
      };
    }
  };

  const themeStyles = getThemeStyles();

  return (
    <>
      {/* Floating Chat Icon with Robot Topper */}
      <button
        className="floating-chat-icon"
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          border: 'none',
          cursor: 'pointer',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          zIndex: '1000',
          transition: 'all 0.3s ease',
          ...themeStyles.chatIcon,
        }}
        onMouseEnter={(e) => {
          Object.assign(e.target.style, {
            ...themeStyles.chatIconHover,
            transform: 'scale(1.05)',
            boxShadow: isDarkTheme
              ? '0 6px 16px rgba(0, 0, 0, 0.4)'
              : '0 6px 16px rgba(0, 0, 0, 0.2)',
          });
        }}
        onMouseLeave={(e) => {
          Object.assign(e.target.style, {
            ...themeStyles.chatIcon,
            transform: 'scale(1)',
            boxShadow: isDarkTheme
              ? '0 4px 12px rgba(0, 0, 0, 0.3)'
              : '0 4px 12px rgba(0, 0, 0, 0.15)',
          });
        }}
      >
        {/* Main chat icon (background) */}
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="28"
          height="28"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          {/* Chat bubble shape */}
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>

        {/* Robot topper (small badge on top) */}
        <div
          style={{
            position: 'absolute',
            top: '-8px',
            right: '-8px',
            width: '24px',
            height: '24px',
            borderRadius: '50%',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            fontSize: '12px',
            fontWeight: 'bold',
            zIndex: '1001',
            ...themeStyles.robotTopper,
            boxShadow: isDarkTheme
              ? '0 2px 6px rgba(0, 0, 0, 0.3)'
              : '0 2px 6px rgba(0, 0, 0, 0.15)',
          }}
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            width="14"
            height="14"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            {/* Robot icon path */}
            <path d="M12 8V4H8m8 4V4h-4m-6 4a2 2 0 1 1 4 0m-4 0a2 2 0 1 0 4 0m-4 0h8m-6 0v2a2 2 0 1 0 4 0V12m-4 0v6a2 2 0 0 0 2 2h2a2 2 0 0 0 2-2v-6" />
            <circle cx="12" cy="10" r="2" />
          </svg>
        </div>
      </button>

      {/* Floating Chat Window - Only render when open */}
      {isOpen && (
        <div
          className="floating-chat-window"
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '380px',
            height: '500px',
            maxHeight: '80vh',
            borderRadius: '12px',
            display: 'flex',
            flexDirection: 'column',
            zIndex: '1000',
            overflow: 'hidden',
            ...themeStyles.chatWindow,
          }}
        >
          <div
            className="chat-header"
            style={{
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              padding: '16px',
              ...themeStyles.header,
            }}
          >
            <h3 style={{ margin: 0, fontSize: '16px', fontWeight: '600', color: themeStyles.header.color }}>
              RAG Agent Chat
            </h3>
            <div className="header-actions" style={{ display: 'flex', gap: '8px' }}>
              <button
                onClick={handleClearChat}
                className="clear-chat-btn"
                title="Clear chat"
                style={{
                  border: '1px solid',
                  borderRadius: '4px',
                  padding: '4px 8px',
                  cursor: 'pointer',
                  fontSize: '12px',
                  ...themeStyles.clearButton,
                }}
              >
                Clear
              </button>
              <button
                onClick={toggleChat}
                className="close-chat-btn"
                title="Close chat"
                style={{
                  width: '28px',
                  height: '28px',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  padding: '0',
                  border: '1px solid',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  ...themeStyles.clearButton,
                }}
              >
                ×
              </button>
            </div>
          </div>

          <div
            className="chat-messages"
            style={{
              flex: 1,
              overflowY: 'auto',
              padding: '16px',
              display: 'flex',
              flexDirection: 'column',
              ...themeStyles.messagesContainer,
            }}
          >
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.sender}`}
                style={{
                  alignSelf: message.sender === 'user' ? 'flex-end' : 'flex-start',
                  borderRadius: '12px',
                  padding: '8px 12px',
                  maxWidth: '80%',
                  margin: '4px 0',
                  ...(message.sender === 'user'
                    ? themeStyles.userMessage
                    : themeStyles.agentMessage),
                }}
              >
                <div className="message-content" style={{ fontSize: '14px', lineHeight: '1.4', color: message.sender === 'user' ? themeStyles.userMessage.color : themeStyles.agentMessage.color }}>
                  {message.content}
                </div>
                {message.sources && message.sources.length > 0 && (
                  <div className="message-sources" style={{ marginTop: '4px', fontSize: '11px', color: themeStyles.agentMessage.color }}>
                    Sources: {message.sources.map((source, idx) => (
                      <span
                        key={idx}
                        className="source-tag"
                        style={{
                          display: 'inline-block',
                          padding: '2px 6px',
                          borderRadius: '10px',
                          margin: '0 2px',
                          fontSize: '10px',
                          ...themeStyles.sourceTag,
                        }}
                      >
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
                  borderRadius: '12px',
                  padding: '8px 12px',
                  maxWidth: '80%',
                  margin: '4px 0',
                  ...themeStyles.agentMessage,
                }}
              >
                <div className="loading-indicator" style={{ fontStyle: 'italic', color: themeStyles.agentMessage.color }}>
                  Thinking...
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {error && (
            <div
              className="chat-error"
              style={{
                padding: '8px 16px',
                fontSize: '12px',
                borderTop: isDarkTheme ? '1px solid #7f1d1d' : '1px solid #ffcdd2',
                ...themeStyles.errorMessage,
              }}
            >
              <span className="error-text" style={{ wordBreak: 'break-word', color: themeStyles.errorMessage.color }}>
                {error}
              </span>
            </div>
          )}

          <form
            onSubmit={handleSendMessage}
            className="chat-input-form"
            style={{
              padding: '12px',
              ...themeStyles.inputForm,
            }}
          >
            {selectedText && (
              <div
                className="selected-text-preview"
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  border: '1px solid',
                  borderRadius: '6px',
                  padding: '6px 8px',
                  margin: '0 0 8px 0',
                  fontSize: '12px',
                  ...themeStyles.selectedTextPreview,
                }}
              >
                <small style={{ flex: 1, color: isDarkTheme ? 'white' : 'inherit' }}>
                  Context: "{selectedText.substring(0, 50)}..."
                </small>
                <button
                  type="button"
                  onClick={() => setSelectedText('')}
                  className="clear-selection-btn"
                  style={{
                    background: 'none',
                    border: 'none',
                    cursor: 'pointer',
                    margin: '0 0 0 8px',
                    fontSize: '14px',
                    padding: '0',
                    width: '18px',
                    height: '18px',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    color: isDarkTheme ? 'white' : '#666',
                  }}
                >
                  ×
                </button>
              </div>
            )}
            <div className="input-container" style={{ display: 'flex', gap: '8px', alignItems: 'flex-end' }}>
              <textarea
                ref={inputRef}
                value={inputValue}
                onChange={handleInputChange}
                onKeyDown={handleKeyDown}
                placeholder="Type your question here... (Press Enter to send, Shift+Enter for new line)"
                className="chat-input"
                rows={1}
                disabled={isLoading}
                style={{
                  flex: 1,
                  border: '1px solid',
                  borderRadius: '8px',
                  padding: '10px 12px',
                  resize: 'none',
                  fontFamily: 'inherit',
                  fontSize: '14px',
                  maxHeight: '100px',
                  minHeight: '40px',
                  outline: 'none',
                  ...themeStyles.inputField,
                }}
                onInput={(e) => {
                  e.target.style.height = 'auto';
                  e.target.style.height = e.target.scrollHeight + 'px';
                }}
              />
              <button
                type="submit"
                disabled={!inputValue.trim() || isLoading}
                className="send-button"
                style={{
                  width: '36px',
                  height: '36px',
                  border: '1px solid',
                  borderRadius: '8px',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  ...themeStyles.button,
                }}
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
                >
                  <line x1="22" y1="2" x2="11" y2="13" />
                  <polygon points="22 2 15 22 11 13 2 9 22 2" />
                </svg>
              </button>
            </div>
          </form>
        </div>
      )}
    </>
  );
};

export default FloatingChatbot;