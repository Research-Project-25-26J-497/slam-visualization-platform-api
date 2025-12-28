# Backend/database/models.py
from sqlalchemy import Column, Integer, String, Float, DateTime
from sqlalchemy.sql import func
from .db import Base

# --- CLASS 1: ANNOTATIONS ---
class Annotation(Base):
    __tablename__ = "annotations"

    id = Column(Integer, primary_key=True, index=True)
    label = Column(String, index=True)
    x = Column(Float, nullable=False)
    y = Column(Float, default=0.0)
    z = Column(Float, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __repr__(self):
        return f"<Annotation(id={self.id}, label='{self.label}')>"

# --- CLASS 2: MAP POINTS (Must be separate!) ---
class MapPoint(Base):
    __tablename__ = "map_points"
    
    id = Column(Integer, primary_key=True, index=True)
    x = Column(Float, nullable=False)
    y = Column(Float, default=0.0)
    z = Column(Float, nullable=False)
    confidence = Column(Float, default=1.0)