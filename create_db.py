"""Database initialization script.

Creates or resets the SQLAlchemy schema for this project.
Use this script to drop existing tables and recreate the schema from models.
"""

from database.db import engine, Base
# ✅ FIXED: Only import Annotation (MapPoint is gone!)
from database.models import Annotation 

def reset_database():
    """Reset the database schema by dropping and recreating all tables."""
    print("⏳ Connecting to Database...")
    
    # 1. DELETE EVERYTHING (The Clean Slate)
    Base.metadata.drop_all(bind=engine)
    print("🗑️ Old data wiped successfully.")

    # 2. CREATE FRESH TABLES
    Base.metadata.create_all(bind=engine)
    print("✅ New tables created.")
    print("🚀 DATABASE RESET COMPLETE!")

if __name__ == "__main__":
    reset_database()