"""
Database Utilities for Neon Postgres Integration

Provides helper functions for database operations related to chat sessions and memory.
"""
from sqlalchemy.orm import Session
from typing import Optional, List, Dict, Any
from datetime import datetime
import json
from db_models import ChatSession, ChatMessage, ConversationMemory, get_db_session


def create_session(user_id: Optional[str] = None, title: Optional[str] = None) -> ChatSession:
    """
    Create a new chat session

    Args:
        user_id: Optional user identifier
        title: Optional session title

    Returns:
        Created ChatSession object
    """
    session = get_db_session()
    try:
        chat_session = ChatSession(
            user_id=user_id,
            title=title
        )
        session.add(chat_session)
        session.commit()
        session.refresh(chat_session)
        return chat_session
    finally:
        session.close()


def get_session(session_id: str) -> Optional[ChatSession]:
    """
    Retrieve a chat session by ID

    Args:
        session_id: The session ID

    Returns:
        ChatSession object or None if not found
    """
    session = get_db_session()
    try:
        chat_session = session.query(ChatSession).filter(ChatSession.id == session_id).first()
        return chat_session
    finally:
        session.close()


def add_message_to_session(
    session_id: str,
    role: str,
    content: str,
    selected_text: Optional[str] = None,
    context_metadata: Optional[Dict[str, Any]] = None
) -> ChatMessage:
    """
    Add a message to a chat session

    Args:
        session_id: The session ID
        role: Message role ('user', 'assistant', 'system')
        content: Message content
        selected_text: Optional selected text for context
        context_metadata: Optional context metadata

    Returns:
        Created ChatMessage object
    """
    db_session = get_db_session()
    try:
        message = ChatMessage(
            session_id=session_id,
            role=role,
            content=content,
            selected_text=selected_text,
            context_metadata=context_metadata
        )
        db_session.add(message)
        db_session.commit()
        db_session.refresh(message)
        return message
    finally:
        db_session.close()


def get_session_messages(session_id: str, limit: int = 50) -> List[ChatMessage]:
    """
    Get messages from a specific session

    Args:
        session_id: The session ID
        limit: Maximum number of messages to retrieve

    Returns:
        List of ChatMessage objects
    """
    db_session = get_db_session()
    try:
        messages = db_session.query(ChatMessage)\
            .filter(ChatMessage.session_id == session_id)\
            .order_by(ChatMessage.timestamp.asc())\
            .limit(limit)\
            .all()
        return messages
    finally:
        db_session.close()


def get_recent_sessions(user_id: Optional[str] = None, limit: int = 10) -> List[ChatSession]:
    """
    Get recent chat sessions

    Args:
        user_id: Optional user identifier to filter by user
        limit: Maximum number of sessions to retrieve

    Returns:
        List of ChatSession objects
    """
    db_session = get_db_session()
    try:
        query = db_session.query(ChatSession).order_by(ChatSession.updated_at.desc()).limit(limit)

        if user_id:
            query = query.filter(ChatSession.user_id == user_id)

        sessions = query.all()
        return sessions
    finally:
        db_session.close()


def save_conversation_memory(
    user_id: Optional[str],
    session_id: Optional[str],
    memory_type: str,
    key: str,
    value: str,
    importance: int = 5,
    expires_at: Optional[datetime] = None
) -> ConversationMemory:
    """
    Save a memory item to the conversation memory table

    Args:
        user_id: Optional user identifier
        session_id: Optional session ID
        memory_type: Type of memory ('fact', 'preference', 'context', etc.)
        key: Memory key
        value: Memory value
        importance: Importance level (1-10)
        expires_at: Optional expiration datetime

    Returns:
        Created ConversationMemory object
    """
    db_session = get_db_session()
    try:
        # Check if a memory with this key already exists for the user
        existing_memory = db_session.query(ConversationMemory)\
            .filter(ConversationMemory.user_id == user_id)\
            .filter(ConversationMemory.key == key)\
            .first()

        if existing_memory:
            # Update existing memory
            existing_memory.value = value
            existing_memory.memory_type = memory_type
            existing_memory.importance = importance
            existing_memory.updated_at = datetime.utcnow()
            existing_memory.expires_at = expires_at
            db_session.commit()
            db_session.refresh(existing_memory)
            return existing_memory
        else:
            # Create new memory
            memory = ConversationMemory(
                user_id=user_id,
                session_id=session_id,
                memory_type=memory_type,
                key=key,
                value=value,
                importance=importance,
                expires_at=expires_at
            )
            db_session.add(memory)
            db_session.commit()
            db_session.refresh(memory)
            return memory
    finally:
        db_session.close()


def get_conversation_memory(user_id: Optional[str], key: str) -> Optional[ConversationMemory]:
    """
    Retrieve a specific memory item

    Args:
        user_id: User identifier
        key: Memory key

    Returns:
        ConversationMemory object or None if not found
    """
    db_session = get_db_session()
    try:
        memory = db_session.query(ConversationMemory)\
            .filter(ConversationMemory.user_id == user_id)\
            .filter(ConversationMemory.key == key)\
            .first()
        return memory
    finally:
        db_session.close()


def get_memories_by_type(user_id: Optional[str], memory_type: str) -> List[ConversationMemory]:
    """
    Retrieve all memories of a specific type for a user

    Args:
        user_id: User identifier
        memory_type: Type of memory to retrieve

    Returns:
        List of ConversationMemory objects
    """
    db_session = get_db_session()
    try:
        memories = db_session.query(ConversationMemory)\
            .filter(ConversationMemory.user_id == user_id)\
            .filter(ConversationMemory.memory_type == memory_type)\
            .all()
        return memories
    finally:
        db_session.close()


def update_session_title(session_id: str, title: str) -> bool:
    """
    Update the title of a chat session

    Args:
        session_id: The session ID
        title: New title for the session

    Returns:
        True if successful, False otherwise
    """
    db_session = get_db_session()
    try:
        chat_session = db_session.query(ChatSession).filter(ChatSession.id == session_id).first()
        if chat_session:
            chat_session.title = title
            chat_session.updated_at = datetime.utcnow()
            db_session.commit()
            return True
        return False
    finally:
        db_session.close()


def deactivate_session(session_id: str) -> bool:
    """
    Deactivate a chat session (soft delete)

    Args:
        session_id: The session ID

    Returns:
        True if successful, False otherwise
    """
    db_session = get_db_session()
    try:
        chat_session = db_session.query(ChatSession).filter(ChatSession.id == session_id).first()
        if chat_session:
            chat_session.is_active = False
            chat_session.updated_at = datetime.utcnow()
            db_session.commit()
            return True
        return False
    finally:
        db_session.close()


def get_user_session_count(user_id: str) -> int:
    """
    Get the number of sessions for a user

    Args:
        user_id: User identifier

    Returns:
        Number of sessions for the user
    """
    db_session = get_db_session()
    try:
        count = db_session.query(ChatSession)\
            .filter(ChatSession.user_id == user_id)\
            .count()
        return count
    finally:
        db_session.close()


def get_recent_messages(session_id: str, limit: int = 10) -> List[ChatMessage]:
    """
    Get recent messages from a specific session

    Args:
        session_id: The session ID
        limit: Maximum number of messages to retrieve

    Returns:
        List of ChatMessage objects
    """
    db_session = get_db_session()
    try:
        messages = db_session.query(ChatMessage)\
            .filter(ChatMessage.session_id == session_id)\
            .order_by(ChatMessage.timestamp.desc())\
            .limit(limit)\
            .all()
        # Reverse the order to get chronological order (oldest first)
        return messages[::-1]
    finally:
        db_session.close()


def save_memory(
    user_id: Optional[str],
    session_id: Optional[str],
    memory_type: str,
    key: str,
    value: str,
    importance: int = 5,
    expires_at: Optional[datetime] = None
) -> ConversationMemory:
    """
    Save a memory item to the conversation memory table (alias for save_conversation_memory)

    Args:
        user_id: Optional user identifier
        session_id: Optional session ID
        memory_type: Type of memory ('fact', 'preference', 'context', etc.)
        key: Memory key
        value: Memory value
        importance: Importance level (1-10)
        expires_at: Optional expiration datetime

    Returns:
        Created ConversationMemory object
    """
    return save_conversation_memory(
        user_id=user_id,
        session_id=session_id,
        memory_type=memory_type,
        key=key,
        value=value,
        importance=importance,
        expires_at=expires_at
    )


def get_memory(user_id: Optional[str], key: str) -> Optional[ConversationMemory]:
    """
    Retrieve a specific memory item (alias for get_conversation_memory)

    Args:
        user_id: User identifier
        key: Memory key

    Returns:
        ConversationMemory object or None if not found
    """
    return get_conversation_memory(user_id=user_id, key=key)


def clear_expired_memories() -> int:
    """
    Remove expired memory entries

    Returns:
        Number of deleted entries
    """
    db_session = get_db_session()
    try:
        from sqlalchemy import and_
        current_time = datetime.utcnow()
        deleted_count = db_session.query(ConversationMemory)\
            .filter(and_(
                ConversationMemory.expires_at.isnot(None),
                ConversationMemory.expires_at < current_time
            ))\
            .delete()
        db_session.commit()
        return deleted_count
    finally:
        db_session.close()