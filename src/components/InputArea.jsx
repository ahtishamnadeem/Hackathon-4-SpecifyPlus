/**
 * Input Area Component
 *
 * Provides the input area for users to type and submit their queries
 */

import React, { useState, useRef, useEffect } from 'react';

const InputArea = ({ onSendMessage, isLoading, selectedText, onClearSelection }) => {
  const [inputValue, setInputValue] = useState('');
  const textareaRef = useRef(null);

  // Auto-resize textarea based on content
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = Math.min(textareaRef.current.scrollHeight, 150) + 'px';
    }
  }, [inputValue]);

  const handleSubmit = (e) => {
    e.preventDefault();

    if (inputValue.trim() || selectedText) {
      const query = inputValue.trim() || `Regarding: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? '...' : ''}"`;
      onSendMessage(query);
      setInputValue('');
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  const handleClearSelection = () => {
    onClearSelection();
  };

  return (
    <div className="input-area">
      {selectedText && (
        <div className="selected-text-preview">
          <div className="selected-text-content">
            <strong>Selected Text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
          </div>
          <button
            onClick={handleClearSelection}
            className="remove-selection-btn"
            title="Remove selected text context"
          >
            ×
          </button>
        </div>
      )}

      <form onSubmit={handleSubmit} className="input-form">
        <div className="input-wrapper">
          <textarea
            ref={textareaRef}
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder={selectedText ? "Ask a question about the selected text..." : "Type your question here... (Press Enter to send, Shift+Enter for new line)"}
            disabled={isLoading}
            rows={1}
            className="chat-input"
          />
          <button
            type="submit"
            disabled={isLoading || (!inputValue.trim() && !selectedText)}
            className="send-button"
          >
            {isLoading ? 'Sending...' : 'Send'}
          </button>
        </div>
      </form>

      <style jsx>{`
        .input-area {
          padding: 16px;
          border-top: 1px solid #ddd;
          background-color: #fafafa;
        }

        .selected-text-preview {
          display: flex;
          align-items: center;
          justify-content: space-between;
          background-color: #e3f2fd;
          border: 1px solid #bbdefb;
          border-radius: 4px;
          padding: 8px 12px;
          margin-bottom: 12px;
          font-size: 0.9em;
        }

        .selected-text-content {
          flex: 1;
          overflow: hidden;
          text-overflow: ellipsis;
          white-space: nowrap;
        }

        .remove-selection-btn {
          background: none;
          border: none;
          font-size: 1.2em;
          cursor: pointer;
          color: #666;
          padding: 0 4px;
          margin-left: 8px;
          width: 24px;
          height: 24px;
          display: flex;
          align-items: center;
          justify-content: center;
          border-radius: 50%;
        }

        .remove-selection-btn:hover {
          background-color: #ffcccb;
          color: #d32f2f;
        }

        .input-form {
          margin: 0;
        }

        .input-wrapper {
          display: flex;
          align-items: flex-end;
          gap: 8px;
        }

        .chat-input {
          flex: 1;
          min-height: 40px;
          max-height: 150px;
          padding: 10px 12px;
          border: 1px solid #ddd;
          border-radius: 8px;
          resize: none;
          font-family: inherit;
          font-size: 14px;
          line-height: 1.4;
          outline: none;
          background-color: white;
        }

        .chat-input:focus {
          border-color: #007bff;
          box-shadow: 0 0 0 2px rgba(0, 123, 255, 0.25);
        }

        .chat-input:disabled {
          background-color: #f5f5f5;
          cursor: not-allowed;
        }

        .send-button {
          padding: 10px 16px;
          background-color: #007bff;
          color: white;
          border: none;
          border-radius: 8px;
          cursor: pointer;
          font-weight: 500;
          transition: background-color 0.2s;
        }

        .send-button:hover:not(:disabled) {
          background-color: #0056b3;
        }

        .send-button:disabled {
          background-color: #ccc;
          cursor: not-allowed;
        }
      `}</style>
    </div>
  );
};

export default InputArea;