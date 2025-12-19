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
    return 'http://localhost:8000';
  }

  // Send chat query
  async sendQuery(queryData) {
    try {
      const response = await this.client.post('/chat/send', queryData);
      return response.data;
    } catch (error) {
      console.error('Error sending query:', error);
      return { success: false, error: { message: error.message } };
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
