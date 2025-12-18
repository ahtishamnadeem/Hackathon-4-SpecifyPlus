/**
 * Chat Interface Component
 *
 * Reusable chat UI component for the Docusaurus frontend that provides an interface
 * for users to interact with the RAG agent.
 */

import React, { useState, useEffect, useRef } from 'react';
import MessageList from './MessageList';
import InputArea from './InputArea';
import LoadingSpinner from './LoadingSpinner';
import ErrorDisplay from './ErrorDisplay';
import apiClient from '../utils/apiClient';
import textSelectionUtility from '../utils/textSelection';

const ChatInterface = () => {
  // State management
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [contextMetadata, setContextMetadata] = useState({});
  const messagesEndRef = useRef(null);

  // Initialize text selection utility
  useEffect(() => {
    // Attach text selection listeners to the document
    textSelectionUtility.attachSelectionListeners(document.body);

    // Listen for text selection events
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
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Handle sending a message
  const handleSendMessage = async (query) => {
    if (!query.trim()) {
      setError('Query cannot be empty');
      return;
    }

    try {
      // Add user message to chat
      const userMessage = {
        id: `msg-${Date.now()}`,
        content: query,
        sender: 'user',
        timestamp: new Date().toISOString(),
        selectedText: selectedText || null
      };

      setMessages(prev => [...prev, userMessage]);
      setIsLoading(true);
      setError(null);

      // Prepare context metadata
      const messageContext = {
        query: query,
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

  // Clear chat history
  const handleClearChat = () => {
    setMessages([]);
    setError(null);
  };

  return (
    <div className="chat-interface">
      <div className="chat-header">
        <h3>RAG Agent Chat</h3>
        <button onClick={handleClearChat} className="clear-chat-btn">
          Clear Chat
        </button>
      </div>

      <div className="chat-container">
        <MessageList messages={messages} />

        {error && <ErrorDisplay message={error} />}

        {isLoading && <LoadingSpinner />}

        <InputArea
          onSendMessage={handleSendMessage}
          isLoading={isLoading}
          selectedText={selectedText}
          onClearSelection={() => setSelectedText('')}
        />

        <div ref={messagesEndRef} />
      </div>

      <style jsx>{`
        .chat-interface {
          display: flex;
          flex-direction: column;
          height: 600px;
          border: 1px solid #ddd;
          border-radius: 8px;
          overflow: hidden;
          font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
        }

        .chat-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 16px;
          background-color: #f5f5f5;
          border-bottom: 1px solid #ddd;
        }

        .chat-header h3 {
          margin: 0;
          color: #333;
        }

        .clear-chat-btn {
          background-color: #e74c3c;
          color: white;
          border: none;
          padding: 6px 12px;
          border-radius: 4px;
          cursor: pointer;
          font-size: 14px;
        }

        .clear-chat-btn:hover {
          background-color: #c0392b;
        }

        .chat-container {
          flex: 1;
          display: flex;
          flex-direction: column;
          overflow: hidden;
        }

        .chat-messages {
          flex: 1;
          overflow-y: auto;
          padding: 16px;
        }
      `}</style>
    </div>
  );
};

export default ChatInterface;