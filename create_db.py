from database.db import engine, Base
# Import models so SQLAlchemy knows what tables to create
from database.models import Annotation, MapPoint 

def reset_database():
    print("⏳ Connecting to Database...")
    
    # 1. DELETE EVERYTHING (The Clean Slate)
    # This drops all tables defined in your models
    Base.metadata.drop_all(bind=engine)
    print("🗑️ Old data wiped successfully.")

    # 2. CREATE FRESH TABLES
    Base.metadata.create_all(bind=engine)
    print("✅ New tables created.")
    print("🚀 DATABASE RESET COMPLETE!")

if __name__ == "__main__":
    reset_database()