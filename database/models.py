# Backend/database/models.py
from sqlalchemy import Column, Integer, String, Float, DateTime
from sqlalchemy.sql import func
from .db import Base

# --- OPTIMIZED: ONLY ANNOTATIONS TABLE ---
# MapPoint table removed - map data stored in memory for performance

class Annotation(Base):
    """
    Persistent storage for human and auto-detected annotations.
    This is the only table that needs database storage.
    """
    __tablename__ = "annotations"

    id = Column(Integer, primary_key=True, index=True)
    label = Column(String, index=True)
    x = Column(Float, nullable=False)
    y = Column(Float, default=0.0)
    z = Column(Float, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __repr__(self):
        return f"<Annotation(id={self.id}, label='{self.label}', x={self.x:.1f}, z={self.z:.1f})>"

# PERFORMANCE NOTES:
# - MapPoint table removed to eliminate 10,000+ row inserts per session
# - Map data now stored in-memory with deque (rolling window)
# - Database only stores persistent annotations (typically < 100 rows)
# - This reduces disk I/O by ~99% and improves response times significantly