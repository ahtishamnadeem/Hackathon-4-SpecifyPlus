"""
Database Models for Neon Postgres Integration

Defines SQLAlchemy models for chat sessions, messages, and related entities.
"""
from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, ForeignKey, Boolean, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship, sessionmaker
from sqlalchemy.sql import func
import uuid
from datetime import datetime
import os

Base = declarative_base()


class ChatSession(Base):
    """
    Represents a chat session with persistent memory
    """
    __tablename__ = 'chat_sessions'

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, nullable=True)  # Optional user identifier
    title = Column(String, nullable=True)  # Session title derived from first message
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    is_active = Column(Boolean, default=True)

    # Relationship to messages
    messages = relationship("ChatMessage", back_populates="session", cascade="all, delete-orphan")


class ChatMessage(Base):
    """
    Represents individual chat messages within a session
    """
    __tablename__ = 'chat_messages'

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    session_id = Column(String, ForeignKey('chat_sessions.id'), nullable=False)
    role = Column(String, nullable=False)  # 'user', 'assistant', 'system'
    content = Column(Text, nullable=False)
    selected_text = Column(Text, nullable=True)  # Text selected by user for context
    context_metadata = Column(JSON, nullable=True)  # Additional context information
    timestamp = Column(DateTime, default=datetime.utcnow)

    # Relationships
    session = relationship("ChatSession", back_populates="messages")


class ConversationMemory(Base):
    """
    Stores persistent memory across conversations
    """
    __tablename__ = 'conversation_memory'

    id = Column(Integer, primary_key=True, autoincrement=True)
    user_id = Column(String, nullable=True)  # Optional user identifier
    session_id = Column(String, ForeignKey('chat_sessions.id'), nullable=True)
    memory_type = Column(String, nullable=False)  # 'fact', 'preference', 'context', etc.
    key = Column(String, nullable=False)  # Memory key
    value = Column(Text, nullable=False)  # Memory value
    importance = Column(Integer, default=5)  # 1-10 scale of importance
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    expires_at = Column(DateTime, nullable=True)  # Optional expiration

    # Relationships
    session = relationship("ChatSession")


def get_database_url():
    """
    Construct database URL from environment variables
    """
    db_url = os.getenv('NEON_DATABASE_URL')
    if not db_url or db_url == 'postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require':
        # Use SQLite for development/testing
        return 'sqlite:///./rag_agent.db'
    return db_url


def get_db_engine():
    """
    Create and return a database engine
    """
    db_url = get_database_url()
    engine = create_engine(db_url, pool_pre_ping=True)
    return engine


def initialize_database():
    """
    Initialize the database tables
    """
    engine = get_db_engine()
    Base.metadata.create_all(engine)
    return engine


def get_db_session():
    """
    Get a database session
    """
    engine = get_db_engine()
    Session = sessionmaker(bind=engine)
    return Session()