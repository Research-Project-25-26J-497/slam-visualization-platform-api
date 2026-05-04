"""SQLAlchemy database connection and session management.

This module reads the database URL from the environment and exposes the engine,
base metadata, and a session generator for dependency injection.
"""

import os
from dotenv import load_dotenv
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base

# Load the variables from .env
load_dotenv()

# Fetch the URL from environment variables
DATABASE_URL = os.getenv("DATABASE_URL")

# Safety check
if not DATABASE_URL:
    raise ValueError("DATABASE_URL not found in environment. Check your .env file!")

engine = create_engine(DATABASE_URL, pool_pre_ping=True)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

def get_db():
    """Provide a database session for FastAPI dependency injection."""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()