# Backend/create_db.py
from database.db import engine, Base
from database.models import Annotation

print("🔌 Connecting to Neon Cloud...")
# Looks at your 'Annotation' model and creates the table in the DB
Base.metadata.create_all(bind=engine)
print("✅ SUCCESS: Tables created in Neon!")