/**
 * API Client for Frontend Communication
 *
 * Handles communication between the Docusaurus frontend chat component and the FastAPI backend
 */

import axios from 'axios';

class ApiClient {
  constructor(baseURL = this.getBackendURL()) {
    this.client = axios.create({
      baseURL,
      timeout: 30000, // 30 seconds timeout
      headers: {
        'Content-Type': 'application/json',
      }
    });

    // Add request interceptor for logging and error handling
    this.client.interceptors.request.use(
      (config) => {
        console.log('API Request:', config.method?.toUpperCase(), config.url);
        return config;
      },
      (error) => {
        console.error('API Request Error:', error);
        return Promise.reject(error);
      }
    );

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => {
        console.log('API Response:', response.status, response.config.url);
        return response;
      },
      (error) => {
        console.error('API Response Error:', error.response?.status, error.message);
        return Promise.reject(error);
      }
    );
  }

  /**
   * Get the backend URL, checking for environment variables or using default
   * @returns {string} The backend URL
   */
  getBackendURL() {
    // Check if we're in a Node.js environment first
    if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL) {
      return process.env.REACT_APP_BACKEND_URL;
    }
    // In browser environment, check for window-based configuration or use default
    return 'http://localhost:8000'; // Default backend URL
  }

  /**
   * Send a user query to the backend
   * @param {Object} queryData - The query data to send
   * @param {string} queryData.query - The user's query text
   * @param {string} [queryData.selected_text] - Text selected from book pages for context
   * @param {Object} [queryData.context_metadata] - Additional context information
   * @param {Object} [queryData.user_preferences] - User preferences for the response
   * @returns {Promise<Object>} The API response
   */
  async sendQuery(queryData) {
    try {
      const response = await this.client.post('/chat/send', queryData);
      return response.data;
    } catch (error) {
      console.error('Error sending query:', error);

      // Return a structured error response
      return {
        success: false,
        error: {
          type: 'RequestError',
          message: error.message || 'Failed to send query',
          code: error.code || 'QUERY_SEND_ERROR'
        }
      };
    }
  }

  /**
   * Send text selection data to the backend
   * @param {Object} selectionData - The text selection data to send
   * @param {string} selectionData.selected_text - The selected text content
   * @param {string} selectionData.page_url - URL of the page where text was selected
   * @param {string} selectionData.page_title - Title of the page where text was selected
   * @param {number} selectionData.position_start - Starting character position of selection
   * @param {number} selectionData.position_end - Ending character position of selection
   * @param {string} selectionData.timestamp - ISO 8601 timestamp of selection
   * @returns {Promise<Object>} The API response
   */
  async sendTextSelection(selectionData) {
    try {
      const response = await this.client.post('/chat/text-selection', selectionData);
      return response.data;
    } catch (error) {
      console.error('Error sending text selection:', error);

      // Return a structured error response
      return {
        success: false,
        error: {
          type: 'RequestError',
          message: error.message || 'Failed to send text selection',
          code: error.code || 'TEXT_SELECTION_ERROR'
        }
      };
    }
  }

  /**
   * Check the health of the backend service
   * @returns {Promise<Object>} The health check response
   */
  async checkHealth() {
    try {
      const response = await this.client.get('/chat/health');
      return response.data;
    } catch (error) {
      console.error('Error checking health:', error);

      // Return a structured error response
      return {
        status: 'unavailable',
        timestamp: new Date().toISOString(),
        services: {
          frontend: 'connected',
          backend: 'disconnected',
          rag_agent: 'disconnected'
        },
        response_time_ms: 0,
        error: {
          type: 'HealthCheckError',
          message: error.message || 'Health check failed',
          code: error.code || 'HEALTH_CHECK_ERROR'
        }
      };
    }
  }
}

// Export a singleton instance
const apiClient = new ApiClient();
export default apiClient;

// Export the class for testing purposes
export { ApiClient };