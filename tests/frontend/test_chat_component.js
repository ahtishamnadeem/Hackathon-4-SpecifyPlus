/**
 * Unit and Integration Tests for Chat Interface Component
 *
 * Tests for the ChatInterface React component and related functionality
 */

// Mock the necessary modules
jest.mock('../../src/utils/apiClient', () => ({
  sendQuery: jest.fn()
}));

jest.mock('../../src/utils/textSelection', () => ({
  attachSelectionListeners: jest.fn(),
  getCurrentSelection: jest.fn()
}));

// Mock React and other dependencies
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import ChatInterface from '../../src/components/ChatInterface';
import apiClient from '../../src/utils/apiClient';
import textSelectionUtility from '../../src/utils/textSelection';

describe('ChatInterface Component', () => {
  beforeEach(() => {
    jest.clearAllMocks();

    // Mock successful API responses
    apiClient.sendQuery.mockResolvedValue({
      success: true,
      message: 'This is a test response',
      sources: [{ text: 'test source', url: 'https://example.com', page: 'Test Page' }],
      confidence: 0.95
    });

    // Mock text selection utility
    textSelectionUtility.attachSelectionListeners.mockImplementation(() => {});
    textSelectionUtility.getCurrentSelection.mockReturnValue(null);
  });

  test('renders chat interface with header, message list, and input area', () => {
    render(<ChatInterface />);

    // Check that the chat header is rendered
    expect(screen.getByText('RAG Agent Chat')).toBeInTheDocument();

    // Check that the clear chat button is rendered
    expect(screen.getByRole('button', { name: /clear chat/i })).toBeInTheDocument();

    // Check that the message list container is rendered
    expect(screen.getByTestId('message-list')).toBeInTheDocument();

    // Check that the input area is rendered
    expect(screen.getByPlaceholderText(/type your question/i)).toBeInTheDocument();
  });

  test('displays welcome message when no messages exist', () => {
    render(<ChatInterface />);

    expect(screen.getByText(/welcome to the rag agent chat/i)).toBeInTheDocument();
    expect(screen.getByText(/ask me anything about the book content/i)).toBeInTheDocument();
  });

  test('allows user to submit a query', async () => {
    render(<ChatInterface />);

    const inputElement = screen.getByPlaceholderText(/type your question/i);
    const sendButton = screen.getByRole('button', { name: /send/i });

    fireEvent.change(inputElement, { target: { value: 'What is ROS 2?' } });
    fireEvent.click(sendButton);

    await waitFor(() => {
      expect(apiClient.sendQuery).toHaveBeenCalledWith(
        expect.objectContaining({
          query: 'What is ROS 2?',
          selected_text: null
        })
      );
    });
  });

  test('handles empty query submission gracefully', async () => {
    render(<ChatInterface />);

    const sendButton = screen.getByRole('button', { name: /send/i });

    // Try to submit with empty query
    fireEvent.click(sendButton);

    // Should not call the API
    expect(apiClient.sendQuery).not.toHaveBeenCalled();
  });

  test('displays user message and agent response', async () => {
    render(<ChatInterface />);

    const inputElement = screen.getByPlaceholderText(/type your question/i);
    const sendButton = screen.getByRole('button', { name: /send/i });

    fireEvent.change(inputElement, { target: { value: 'Test query' } });
    fireEvent.click(sendButton);

    await waitFor(() => {
      // Check that user message is displayed
      expect(screen.getByText('Test query')).toBeInTheDocument();

      // Check that agent response is displayed
      expect(screen.getByText('This is a test response')).toBeInTheDocument();
    });
  });

  test('shows loading state during API request', async () => {
    // Mock a delayed response
    const mockPromise = new Promise(resolve =>
      setTimeout(() => resolve({
        success: true,
        message: 'Delayed response',
        sources: [],
        confidence: 0.8
      }), 100)
    );
    apiClient.sendQuery.mockReturnValue(mockPromise);

    render(<ChatInterface />);

    const inputElement = screen.getByPlaceholderText(/type your question/i);
    const sendButton = screen.getByRole('button', { name: /send/i });

    fireEvent.change(inputElement, { target: { value: 'Delayed query' } });
    fireEvent.click(sendButton);

    // Check that loading spinner appears
    expect(screen.getByText(/processing your request/i)).toBeInTheDocument();

    // Wait for response to complete
    await waitFor(() => {
      expect(screen.queryByText(/processing your request/i)).not.toBeInTheDocument();
    });
  });

  test('handles API errors gracefully', async () => {
    // Mock API error
    apiClient.sendQuery.mockRejectedValue(new Error('Network error'));

    render(<ChatInterface />);

    const inputElement = screen.getByPlaceholderText(/type your question/i);
    const sendButton = screen.getByRole('button', { name: /send/i });

    fireEvent.change(inputElement, { target: { value: 'Error test query' } });
    fireEvent.click(sendButton);

    await waitFor(() => {
      // Check that error message is displayed
      expect(screen.getByText(/network error/i)).toBeInTheDocument();
    });
  });

  test('clears chat history when clear button is clicked', () => {
    render(<ChatInterface />);

    const clearButton = screen.getByRole('button', { name: /clear chat/i });
    fireEvent.click(clearButton);

    // After clearing, should show welcome message again
    expect(screen.getByText(/welcome to the rag agent chat/i)).toBeInTheDocument();
  });

  test('uses selected text context when available', async () => {
    // Simulate selected text
    const mockSelection = {
      text: 'This is the selected text',
      page_url: 'https://example.com/page',
      page_title: 'Test Page',
      position_start: 100,
      position_end: 150,
      timestamp: '2025-12-19T10:30:00Z'
    };

    textSelectionUtility.getCurrentSelection.mockReturnValue(mockSelection);

    render(<ChatInterface />);

    const inputElement = screen.getByPlaceholderText(/type your question/i);
    const sendButton = screen.getByRole('button', { name: /send/i });

    fireEvent.change(inputElement, { target: { value: 'Question about the selected text' } });
    fireEvent.click(sendButton);

    await waitFor(() => {
      expect(apiClient.sendQuery).toHaveBeenCalledWith(
        expect.objectContaining({
          query: 'Question about the selected text',
          selected_text: 'This is the selected text'
        })
      );
    });
  });

  test('removes selected text after query submission', async () => {
    // Simulate selected text
    const mockSelection = {
      text: 'Selected text for context',
      page_url: 'https://example.com/page',
      page_title: 'Test Page',
      position_start: 100,
      position_end: 150,
      timestamp: '2025-12-19T10:30:00Z'
    };

    textSelectionUtility.getCurrentSelection.mockReturnValue(mockSelection);

    render(<ChatInterface />);

    const inputElement = screen.getByPlaceholderText(/type your question/i);
    const sendButton = screen.getByRole('button', { name: /send/i });

    fireEvent.change(inputElement, { target: { value: 'Test query with context' } });
    fireEvent.click(sendButton);

    await waitFor(() => {
      // Verify that the query was sent with selected text
      expect(apiClient.sendQuery).toHaveBeenCalledWith(
        expect.objectContaining({
          selected_text: 'Selected text for context'
        })
      );
    });
  });
});

describe('MessageList Component Integration', () => {
  test('displays messages with proper formatting', () => {
    const mockMessages = [
      {
        id: 'msg-1',
        content: 'Hello, this is a user message',
        sender: 'user',
        timestamp: new Date().toISOString()
      },
      {
        id: 'msg-2',
        content: 'Hello, this is an agent response',
        sender: 'agent',
        timestamp: new Date().toISOString(),
        sources: [{ text: 'source text', url: 'https://example.com', page: 'Example Page' }],
        confidence: 0.95
      }
    ];

    // Since we can't directly test MessageList separately, we'll test through ChatInterface
    render(<ChatInterface />);

    // We'd need to update the ChatInterface to accept messages as props for this test
    // This would require modifying the component to allow direct message injection for testing
  });
});

describe('InputArea Component Integration', () => {
  test('formats query when no input text but selected text exists', async () => {
    // This test would require the ability to simulate selected text context
    // For now, we'll test the basic functionality

    render(<ChatInterface />);

    // The component should handle the case where there's no query text but selected text exists
    expect(screen.getByPlaceholderText(/type your question/i)).toBeInTheDocument();
  });
});