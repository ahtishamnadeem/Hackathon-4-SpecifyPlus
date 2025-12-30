/**
 * API Client for Docusaurus frontend
 * Handles communication with FastAPI RAG backend
 */

import axios from 'axios';

class ApiClient {
  constructor(baseURL = this.getBackendURL()) {
    this.client = axios.create({
      baseURL,
      timeout: 120000, // 2 minutes to prevent timeout
      headers: {'Content-Type': 'application/json'}
    });

    // Request interceptor
    this.client.interceptors.request.use(
      (config) => {
        console.log('API Request:', config.method?.toUpperCase(), config.url);
        return config;
      },
      (error) => Promise.reject(error)
    );

    // Response interceptor
    this.client.interceptors.response.use(
      (response) => response,
      (error) => {
        console.error('API Response Error:', error.response?.status, error.message);
        return Promise.reject(error);
      }
    );
  }

  getBackendURL() {
    if (typeof process !== 'undefined' && process.env.REACT_APP_BACKEND_URL) {
      return process.env.REACT_APP_BACKEND_URL;
    }
    return 'https://ahtisham2006-rag-chatbot-backend.hf.space';
  }

  // Send chat query
  async sendQuery(queryData) {
    try {
      // Prepare the payload with the new structure expected by the backend
      const payload = {
        query: queryData.query || queryData.message, // Handle both field names
        selected_text: queryData.selected_text || queryData.selectedText || null,
        context_metadata: queryData.context_metadata || queryData.contextMetadata || {},
        max_tokens: queryData.user_preferences?.max_tokens || 500,
        temperature: queryData.user_preferences?.temperature || 0.7,
        include_sources: true
      };

      const response = await this.client.post('/chat/send', payload);
      return {
        success: true,
        message: response.data.reply,
        sources: response.data.sources,
        confidence: response.data.confidence,
        session_id: response.data.session_id
      };
    } catch (error) {
      console.error('Error sending query:', error);
      return {
        success: false,
        error: {
          message: error.response?.data?.detail || error.message || 'Failed to send query'
        }
      };
    }
  }

  // Check health
  async checkHealth() {
    try {
      const response = await this.client.get('/health');
      return response.data;
    } catch (error) {
      console.error('Health check failed:', error);
      return { status: 'unavailable', error: { message: error.message } };
    }
  }
}

const apiClient = new ApiClient();
export default apiClient;
export { ApiClient };
