/**
 * Modern AI Assistant Chatbot Component
 *
 * A floating chat widget with modern AI assistant interface
 * inspired by ChatGPT, Claude, and advanced robotics research tools.
 */

import React, { useState, useEffect, useRef } from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import apiClient from '../utils/apiClient';
import styles from './chatStyles.module.css';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [contextMetadata, setContextMetadata] = useState({});

  const { colorMode } = useColorMode();
  const isDarkTheme = colorMode === 'dark';

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
        let content = response.message;
        let senderType = 'agent';

        // Check if the response starts with the book prefix
        if (response.message.startsWith('--- According to this book: ---')) {
          senderType = 'system';
          content = response.message.replace('--- According to this book: ---', '').trim();
        } else if (response.message.startsWith('--- Based on general robotics knowledge (not from the book): ---')) {
          senderType = 'agent';
          content = response.message.replace('--- Based on general robotics knowledge (not from the book): ---', '').trim();
        }

        const agentMessage = {
          id: `msg-${Date.now()}-agent`,
          content: content,
          sender: senderType,
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
        className={styles.floatingChatIcon}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
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
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
      </button>

      {/* Floating Chat Window - Only render when open */}
      {isOpen && (
        <div className={styles.chatContainer}>
          <div className={styles.chatHeader}>
            <h3 className={styles.headerTitle}>
              <span className={styles.agentStatus}>
                <span className={styles.statusIndicator}></span>
                Agent is Online
              </span>
            </h3>
            <div className={styles.headerActions}>
              <button
                onClick={handleClearChat}
                className={styles.chatButton}
                title="Clear chat"
              >
                Clear
              </button>
              <button
                onClick={toggleChat}
                className={`${styles.chatButton} ${styles.closeButton}`}
                title="Close chat"
              >
                ×
              </button>
            </div>
          </div>

          <div className={styles.messagesContainer}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${
                  message.sender === 'user' ? styles.userMessage :
                  message.sender === 'system' ? styles.systemMessage :
                  styles.agentMessage
                }`}
              >
                <div className={styles.messageContent}>
                  {message.content}
                </div>

                {/* Sources section */}
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sourcesSection}>
                    <span className={styles.sourcesLabel}>Sources:</span>
                    <div className={styles.sourcesList}>
                      {message.sources.map((source, idx) => (
                        <span
                          key={idx}
                          className={styles.sourceTag}
                        >
                          {source.title || `Source ${idx + 1}`}
                        </span>
                      ))}
                    </div>

                    {/* Confidence indicator */}
                    {message.confidence !== undefined && (
                      <div className={styles.confidenceIndicator}>
                        <span>Confidence: {(message.confidence * 100).toFixed(0)}%</span>
                        <div className={styles.confidenceBar}>
                          <div
                            className={styles.confidenceFill}
                            style={{ width: `${message.confidence * 100}%` }}
                          />
                        </div>
                      </div>
                    )}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={styles.typingIndicator}>
                <span>Thinking...</span>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {error && (
            <div className={styles.errorMessage}>
              {error}
            </div>
          )}

          <form
            onSubmit={handleSendMessage}
            className={styles.inputForm}
          >
            {selectedText && (
              <div className={styles.selectedTextPreview}>
                <small>Context: "{selectedText.substring(0, 50)}..."</small>
                <button
                  type="button"
                  onClick={() => setSelectedText('')}
                  className={styles.clearSelectionBtn}
                >
                  ×
                </button>
              </div>
            )}
            <div className={styles.inputContainer}>
              <textarea
                ref={inputRef}
                value={inputValue}
                onChange={handleInputChange}
                onKeyDown={handleKeyDown}
                placeholder="Type your question here... (Press Enter to send, Shift+Enter for new line)"
                className={styles.chatInput}
                rows={1}
                disabled={isLoading}
                onInput={(e) => {
                  e.target.style.height = 'auto';
                  e.target.style.height = Math.min(e.target.scrollHeight, 120) + 'px';
                }}
              />
              <button
                type="submit"
                disabled={!inputValue.trim() || isLoading}
                className={styles.sendButton}
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