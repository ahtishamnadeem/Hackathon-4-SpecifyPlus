"""
Validation Module

Contains validation functions and error handling classes for the frontend-backend integration.
"""

import logging
from typing import Any, Dict, Optional
from pydantic import BaseModel, ValidationError as PydanticValidationError


logger = logging.getLogger(__name__)


# Error classes
class ValidationError(Exception):
    """Raised when request validation fails"""
    pass


class ConfigurationError(Exception):
    """Raised when configuration parameters are invalid"""
    pass


class ConnectionError(Exception):
    """Raised when a connection error occurs"""
    pass


class AuthenticationError(Exception):
    """Raised when authentication fails"""
    pass


class NotFoundError(Exception):
    """Raised when a requested resource is not found"""
    pass


class RateLimitError(Exception):
    """Raised when API rate limits are exceeded"""
    pass


class ServiceUnavailableError(Exception):
    """Raised when a dependent service is unavailable"""
    pass


class BadRequestError(Exception):
    """Raised when a request is malformed or invalid"""
    pass


class InternalError(Exception):
    """Raised when an internal server error occurs"""
    pass


def validate_request(request_data: Dict[str, Any], schema: BaseModel) -> Dict[str, Any]:
    """
    Validate a request against a Pydantic schema

    Args:
        request_data: The request data to validate
        schema: The Pydantic schema to validate against

    Returns:
        Validated request data

    Raises:
        ValidationError: If the request data doesn't match the schema
    """
    try:
        # Validate the request data against the schema
        validated_data = schema(**request_data)

        # Convert back to dictionary
        return validated_data.model_dump()

    except PydanticValidationError as e:
        # Extract error details from Pydantic validation error
        error_messages = []
        for error in e.errors():
            field = ".".join(str(loc) for loc in error['loc']) if error['loc'] else 'unknown'
            message = f"{field}: {error['msg']}"
            error_messages.append(message)

        error_msg = "; ".join(error_messages)
        logger.error(f"Request validation failed: {error_msg}")
        raise ValidationError(f"Request validation failed: {error_msg}")


def validate_chat_query_request(query: str, selected_text: Optional[str] = None,
                               context_metadata: Optional[Dict[str, Any]] = None,
                               user_preferences: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Validate chat query request parameters

    Args:
        query: The user's query text
        selected_text: Optional selected text from book pages
        context_metadata: Optional additional context information
        user_preferences: Optional user preferences for the response

    Returns:
        Validated request parameters

    Raises:
        ValidationError: If any parameter is invalid
    """
    errors = []

    # Validate query
    if not query or not query.strip():
        errors.append("Query text is required and cannot be empty")
    elif len(query.strip()) < 1:
        errors.append("Query text must be at least 1 character long")
    elif len(query) > 5000:  # Arbitrary limit to prevent extremely large queries
        errors.append("Query text must be less than 5000 characters")

    # Validate selected_text if provided
    if selected_text is not None:
        if not isinstance(selected_text, str):
            errors.append("Selected text must be a string")
        elif len(selected_text) > 10000:  # Arbitrary limit
            errors.append("Selected text must be less than 10000 characters")

    # Validate context_metadata if provided
    if context_metadata is not None:
        if not isinstance(context_metadata, dict):
            errors.append("Context metadata must be a dictionary")
        else:
            # Validate specific expected fields in context_metadata
            if 'page_url' in context_metadata:
                page_url = context_metadata['page_url']
                if not isinstance(page_url, str) or not page_url.strip():
                    errors.append("Page URL in context metadata must be a non-empty string")

            if 'page_title' in context_metadata:
                page_title = context_metadata['page_title']
                if not isinstance(page_title, str) or not page_title.strip():
                    errors.append("Page title in context metadata must be a non-empty string")

    # Validate user_preferences if provided
    if user_preferences is not None:
        if not isinstance(user_preferences, dict):
            errors.append("User preferences must be a dictionary")
        else:
            # Validate specific expected fields in user_preferences
            if 'temperature' in user_preferences:
                temp = user_preferences['temperature']
                if not isinstance(temp, (int, float)) or temp < 0.0 or temp > 1.0:
                    errors.append("Temperature in user preferences must be a number between 0.0 and 1.0")

            if 'max_tokens' in user_preferences:
                max_tokens = user_preferences['max_tokens']
                if not isinstance(max_tokens, int) or max_tokens <= 0:
                    errors.append("Max tokens in user preferences must be a positive integer")

    if errors:
        error_msg = "; ".join(errors)
        logger.error(f"Chat query request validation failed: {error_msg}")
        raise ValidationError(f"Chat query request validation failed: {error_msg}")

    # Return validated parameters
    return {
        'query': query.strip(),
        'selected_text': selected_text.strip() if selected_text else None,
        'context_metadata': context_metadata or {},
        'user_preferences': user_preferences or {}
    }


def validate_text_selection_request(selected_text: str, page_url: str, page_title: str,
                                  position_start: int, position_end: int, timestamp: str) -> Dict[str, Any]:
    """
    Validate text selection request parameters

    Args:
        selected_text: The selected text content
        page_url: URL of the page where text was selected
        page_title: Title of the page where text was selected
        position_start: Starting character position of selection
        position_end: Ending character position of selection
        timestamp: ISO 8601 timestamp of selection

    Returns:
        Validated request parameters

    Raises:
        ValidationError: If any parameter is invalid
    """
    errors = []

    # Validate selected_text
    if not selected_text or not selected_text.strip():
        errors.append("Selected text is required and cannot be empty")
    elif len(selected_text) > 10000:  # Arbitrary limit
        errors.append("Selected text must be less than 10000 characters")

    # Validate page_url
    if not page_url or not page_url.strip():
        errors.append("Page URL is required and cannot be empty")
    elif not page_url.startswith(('http://', 'https://')):
        errors.append("Page URL must be a valid URL starting with http:// or https://")

    # Validate page_title
    if not page_title or not page_title.strip():
        errors.append("Page title is required and cannot be empty")
    elif len(page_title) > 500:  # Arbitrary limit
        errors.append("Page title must be less than 500 characters")

    # Validate positions
    if not isinstance(position_start, int) or position_start < 0:
        errors.append("Position start must be a non-negative integer")

    if not isinstance(position_end, int) or position_end < 0:
        errors.append("Position end must be a non-negative integer")

    if position_start > position_end:
        errors.append("Position start cannot be greater than position end")

    # Validate timestamp
    if not timestamp or not timestamp.strip():
        errors.append("Timestamp is required and cannot be empty")
    # In a real implementation, we would validate that it's in ISO 8601 format

    if errors:
        error_msg = "; ".join(errors)
        logger.error(f"Text selection request validation failed: {error_msg}")
        raise ValidationError(f"Text selection request validation failed: {error_msg}")

    # Return validated parameters
    return {
        'selected_text': selected_text.strip(),
        'page_url': page_url.strip(),
        'page_title': page_title.strip(),
        'position_start': position_start,
        'position_end': position_end,
        'timestamp': timestamp.strip()
    }


def validate_api_response(response_data: Dict[str, Any], required_fields: list = None) -> bool:
    """
    Validate an API response structure

    Args:
        response_data: The response data to validate
        required_fields: List of required field names in the response

    Returns:
        True if validation passes, False otherwise

    Raises:
        ValidationError: If the response structure is invalid
    """
    if not isinstance(response_data, dict):
        raise ValidationError("API response must be a dictionary")

    if required_fields:
        missing_fields = [field for field in required_fields if field not in response_data]
        if missing_fields:
            raise ValidationError(f"Missing required fields in API response: {', '.join(missing_fields)}")

    return True


# Example usage and testing
if __name__ == "__main__":
    print("Validation module loaded successfully!")

    # Test validation functions
    try:
        # Test chat query validation
        valid_params = validate_chat_query_request(
            query="What is ROS 2?",
            selected_text="Robot Operating System 2 is a flexible framework",
            context_metadata={
                "page_url": "https://book.example.com/ros2-intro",
                "page_title": "Introduction to ROS 2"
            }
        )
        print(f"Chat query validation passed: {valid_params}")

        # Test text selection validation
        valid_selection = validate_text_selection_request(
            selected_text="This is the selected text",
            page_url="https://book.example.com/chapter-2",
            page_title="Advanced Concepts",
            position_start=150,
            position_end=190,
            timestamp="2025-12-19T10:30:00Z"
        )
        print(f"Text selection validation passed: {valid_selection}")

        print("All validation tests passed!")

    except ValidationError as e:
        print(f"Validation error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")