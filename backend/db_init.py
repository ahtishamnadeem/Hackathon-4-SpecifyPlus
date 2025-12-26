"""
Database Initialization Script

This script initializes the Neon Postgres database with the required tables
for the RAG chatbot application.
"""
import os
import sys
import logging
from sqlalchemy import create_engine, text
from sqlalchemy.exc import SQLAlchemyError

# Add the current directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from db_models import Base, get_database_url

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)


def initialize_database():
    """
    Initialize the database by creating all required tables
    """
    try:
        # Get the database URL
        db_url = get_database_url()
        logger.info(f"Connecting to database: {db_url}")

        # Create the engine
        engine = create_engine(db_url, pool_pre_ping=True)

        # Test the connection
        with engine.connect() as connection:
            connection.execute(text("SELECT 1"))
        logger.info("Database connection successful")

        # Create all tables defined in our models
        logger.info("Creating database tables...")
        Base.metadata.create_all(engine)
        logger.info("Database tables created successfully")

        # Verify tables were created
        tables = Base.metadata.tables.keys()
        logger.info(f"Tables created: {list(tables)}")

        return True

    except SQLAlchemyError as e:
        logger.error(f"SQLAlchemy error during database initialization: {str(e)}")
        return False
    except Exception as e:
        logger.error(f"Error during database initialization: {str(e)}")
        return False


def test_database_connection():
    """
    Test the database connection without creating tables
    """
    try:
        db_url = get_database_url()
        logger.info(f"Testing database connection: {db_url}")

        engine = create_engine(db_url, pool_pre_ping=True)

        with engine.connect() as connection:
            # For SQLite, there's no version() function, so we'll use a simple query
            if db_url.startswith('sqlite'):
                result = connection.execute(text("SELECT sqlite_version()"))
                version = result.fetchone()[0]
                logger.info(f"SQLite connection successful. Version: {version}")
            else:
                result = connection.execute(text("SELECT version()"))
                version = result.fetchone()[0]
                logger.info(f"PostgreSQL connection successful. Version: {version}")

        return True

    except SQLAlchemyError as e:
        logger.error(f"SQLAlchemy error during connection test: {str(e)}")
        return False
    except Exception as e:
        logger.error(f"Error during database connection test: {str(e)}")
        return False


def main():
    """
    Main function to run the database initialization
    """
    print("=== RAG Chatbot Database Initialization ===\n")

    # Get database URL from the models (with fallback to SQLite)
    try:
        db_url = get_database_url()
    except ValueError:
        # If the default URL is used, set a fallback SQLite URL
        db_url = 'sqlite:///./rag_agent.db'
        print("Using SQLite for development (NEON_DATABASE_URL not configured)")

    print(f"Database URL: {db_url[:50]}...")  # Show beginning of URL for verification
    print()

    # Test the connection first
    print("1. Testing database connection...")
    if not test_database_connection():
        print("   FAILED: Could not connect to database")
        return 1
    print("   SUCCESS: Database connection established\n")

    # Initialize the database
    print("2. Initializing database tables...")
    if not initialize_database():
        print("   FAILED: Could not initialize database tables")
        return 1
    print("   SUCCESS: Database initialized successfully\n")

    print("=== Database initialization completed successfully! ===")
    print("\nNext steps:")
    print("- Ensure your .env file has the correct NEON_DATABASE_URL for production")
    print("- Run the application to start using the RAG chatbot")
    print("- Monitor logs for any database-related issues")

    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)