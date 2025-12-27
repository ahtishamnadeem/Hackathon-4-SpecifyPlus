/**
 * Translation API Client for Urdu Translation Feature
 * Uses the existing backend infrastructure
 */

import axios from 'axios';

class TranslationApi {
  constructor() {
    // Safely handle environment variables in both Node.js and browser environments
    let backendUrl = 'http://localhost:8000';

    // Check if process exists and has env property (Node.js environment)
    if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL) {
      backendUrl = process.env.REACT_APP_BACKEND_URL;
    }

    this.client = axios.create({
      baseURL: backendUrl,
      timeout: 60000, // 60 seconds timeout for translation
      headers: {
        'Content-Type': 'application/json',
      },
    });
  }

  /**
   * Translate English content to Urdu using the existing LLM infrastructure
   * @param {string} content - The content to translate
   * @param {string} context - Context for better translation (e.g., "technical", "educational")
   * @returns {Promise<string>} - Translated content
   */
  async translateToUrdu(content, context = 'educational') {
    try {
      // Use the existing chat endpoint to perform translation
      // We'll send a specific prompt to the LLM asking for Urdu translation
      const prompt = `Translate the following text to Urdu. Maintain technical terms in English where appropriate (like ROS 2, DDS, QoS, etc.). The context is ${context}. Here is the text to translate:\n\n${content}`;

      const response = await this.client.post('/chat/send', {
        query: prompt,
        selected_text: null,
        context_metadata: {
          translation_request: true,
          target_language: 'urdu',
          source_language: 'english'
        },
        max_tokens: 2000, // Allow for longer translations
        temperature: 0.3 // Lower temperature for more consistent translations
      });

      if (response.data && response.data.reply) {
        return response.data.reply;
      } else {
        throw new Error('Invalid response from translation service');
      }
    } catch (error) {
      console.error('Translation API error:', error);
      // Fallback to a simpler translation request if the first one fails
      try {
        const fallbackPrompt = `Translate to Urdu: ${content.substring(0, 1000)}`;
        const fallbackResponse = await this.client.post('/chat/send', {
          query: fallbackPrompt,
          max_tokens: 1000,
          temperature: 0.3
        });

        return fallbackResponse.data?.reply || content;
      } catch (fallbackError) {
        console.error('Fallback translation also failed:', fallbackError);
        throw new Error(`Translation failed: ${error.response?.data?.detail || error.message}`);
      }
    }
  }

  /**
   * Batch translate content chunks using the existing infrastructure
   * @param {Array<string>} chunks - Array of content chunks to translate
   * @param {string} context - Context for translation
   * @returns {Promise<Array<string>>} - Array of translated chunks
   */
  async batchTranslateToUrdu(chunks, context = 'educational') {
    try {
      const translations = [];

      for (const chunk of chunks) {
        const translated = await this.translateToUrdu(chunk, context);
        translations.push(translated);
      }

      return translations;
    } catch (error) {
      console.error('Batch translation API error:', error);
      throw new Error(`Batch translation failed: ${error.message}`);
    }
  }

  /**
   * Check if translation service is available (using existing health check)
   * @returns {Promise<boolean>}
   */
  async healthCheck() {
    try {
      const response = await this.client.get('/chat/health');
      return response.data.status === 'healthy';
    } catch (error) {
      console.error('Translation service health check failed:', error);
      return false;
    }
  }
}

const translationApi = new TranslationApi();
export default translationApi;