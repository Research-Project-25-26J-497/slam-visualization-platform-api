from database.db import engine, Base
# ✅ FIXED: Only import Annotation (MapPoint is gone!)
from database.models import Annotation 

def reset_database():
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