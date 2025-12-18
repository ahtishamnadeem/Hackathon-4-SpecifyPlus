/**
 * Text Selection Utility
 *
 * Handles capturing text selections from book pages and preparing them for query submission
 */

import apiClient from './apiClient';

class TextSelectionUtility {
  constructor() {
    this.selectionHistory = [];
    this.currentSelection = null;
  }

  /**
   * Get the current text selection from the document
   * @returns {Object|null} Selection object with text, positions, and metadata
   */
  getCurrentSelection() {
    const selection = window.getSelection ? window.getSelection() : document.selection;

    if (!selection || selection.toString().trim() === '') {
      return null;
    }

    const selectedText = selection.toString().trim();

    if (!selectedText) {
      return null;
    }

    // Get the anchor and focus nodes to determine selection positions
    const anchorNode = selection.anchorNode;
    const focusNode = selection.focusNode;
    const anchorOffset = selection.anchorOffset;
    const focusOffset = selection.focusOffset;

    // Determine start and end positions based on selection direction
    let startPosition = 0;
    let endPosition = 0;

    if (anchorNode === focusNode) {
      // Simple case: selection within same text node
      startPosition = Math.min(anchorOffset, focusOffset);
      endPosition = Math.max(anchorOffset, focusOffset);
    } else {
      // Complex case: selection spans multiple nodes
      // For now, we'll approximate by using the length of the selected text
      // In a more sophisticated implementation, we'd traverse the DOM to calculate positions
      startPosition = anchorOffset;
      endPosition = anchorOffset + selectedText.length;
    }

    // Get page information
    const pageUrl = window.location.href;
    const pageTitle = document.title || '';

    // Create the selection object
    const selectionObj = {
      id: this.generateSelectionId(),
      text: selectedText,
      page_url: pageUrl,
      page_title: pageTitle,
      position_start: startPosition,
      position_end: endPosition,
      timestamp: new Date().toISOString()
    };

    this.currentSelection = selectionObj;
    this.selectionHistory.push(selectionObj);

    // Limit history to prevent memory issues
    if (this.selectionHistory.length > 100) {
      this.selectionHistory = this.selectionHistory.slice(-50); // Keep last 50 selections
    }

    return selectionObj;
  }

  /**
   * Attach event listeners for text selection on book pages
   * @param {HTMLElement} [container=document.body] - The container element to attach listeners to
   */
  attachSelectionListeners(container = document.body) {
    // Listen for mouseup event which indicates end of text selection
    container.addEventListener('mouseup', (event) => {
      // Small delay to ensure selection is finalized
      setTimeout(() => {
        const selection = this.getCurrentSelection();
        if (selection) {
          this.onTextSelected(selection);
        }
      }, 10);
    });

    // Also listen for keyup events (e.g., Shift+Arrow selections)
    container.addEventListener('keyup', (event) => {
      if (event.key === 'Shift') {
        setTimeout(() => {
          const selection = this.getCurrentSelection();
          if (selection) {
            this.onTextSelected(selection);
          }
        }, 10);
      }
    });
  }

  /**
   * Handler for when text is selected
   * @param {Object} selection - The selection object
   */
  onTextSelected(selection) {
    console.log('Text selected:', selection);

    // Dispatch a custom event that other parts of the application can listen to
    const textSelectedEvent = new CustomEvent('textSelected', {
      detail: selection,
      bubbles: true,
      cancelable: true
    });

    document.dispatchEvent(textSelectedEvent);
  }

  /**
   * Get the most recent selection
   * @returns {Object|null} The most recent selection or null if none
   */
  getMostRecentSelection() {
    if (this.selectionHistory.length === 0) {
      return null;
    }
    return this.selectionHistory[this.selectionHistory.length - 1];
  }

  /**
   * Clear the selection history
   */
  clearSelectionHistory() {
    this.selectionHistory = [];
    this.currentSelection = null;
  }

  /**
   * Generate a unique ID for a selection
   * @returns {string} A unique selection ID
   */
  generateSelectionId() {
    return `sel-${Date.now()}-${Math.floor(Math.random() * 1000000)}`;
  }

  /**
   * Validate a selection object
   * @param {Object} selection - The selection object to validate
   * @returns {Array<string>} Array of validation errors (empty if valid)
   */
  validateSelection(selection) {
    const errors = [];

    if (!selection) {
      errors.push('Selection object is required');
      return errors;
    }

    if (!selection.text || typeof selection.text !== 'string' || selection.text.trim() === '') {
      errors.push('Selection text is required and must be a non-empty string');
    }

    if (!selection.page_url || typeof selection.page_url !== 'string') {
      errors.push('Page URL is required and must be a string');
    }

    if (!selection.page_title || typeof selection.page_title !== 'string') {
      errors.push('Page title is required and must be a string');
    }

    if (typeof selection.position_start !== 'number' || selection.position_start < 0) {
      errors.push('Position start must be a non-negative number');
    }

    if (typeof selection.position_end !== 'number' || selection.position_end < 0) {
      errors.push('Position end must be a non-negative number');
    }

    if (selection.position_start > selection.position_end) {
      errors.push('Position start must be less than or equal to position end');
    }

    if (!selection.timestamp || typeof selection.timestamp !== 'string') {
      errors.push('Timestamp is required and must be a string in ISO format');
    }

    return errors;
  }

  /**
   * Format a selection for API submission
   * @param {Object} selection - The selection object to format
   * @returns {Object} Formatted selection object for API
   */
  formatSelectionForApi(selection) {
    const validationErrors = this.validateSelection(selection);

    if (validationErrors.length > 0) {
      throw new Error(`Invalid selection: ${validationErrors.join(', ')}`);
    }

    return {
      selected_text: selection.text,
      page_url: selection.page_url,
      page_title: selection.page_title,
      position_start: selection.position_start,
      position_end: selection.position_end,
      timestamp: selection.timestamp
    };
  }

  /**
   * Send the selected text to the backend for context capture
   * @param {Object} selection - The selection object to send
   * @returns {Promise<Object>} The API response
   */
  async sendSelectionToBackend(selection) {
    try {
      const formattedSelection = this.formatSelectionForApi(selection);

      const response = await apiClient.sendTextSelection(formattedSelection);

      return response;
    } catch (error) {
      console.error('Error sending text selection to backend:', error);
      throw error;
    }
  }
}

// Export a singleton instance
const textSelectionUtility = new TextSelectionUtility();
export default textSelectionUtility;

// Export the class for testing purposes
export { TextSelectionUtility };