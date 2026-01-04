# Backend/database/db.py
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base

# ---------------------------------------------------------
# ⚠️ ACTION REQUIRED: PASTE YOUR NEON CONNECTION STRING BELOW
# It should look like: postgres://user:pass@ep-xyz.neon.tech/neondb?sslmode=require
# ---------------------------------------------------------
DATABASE_URL = "postgresql://neondb_owner:npg_1cgr3jzWOsVC@ep-green-flower-adk6knnd-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"

# Create the engine (pool_pre_ping keeps the cloud connection alive)
engine = create_engine(DATABASE_URL, pool_pre_ping=True)

# Create the Session Factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for models
Base = declarative_base()

# Utility to get a DB session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()