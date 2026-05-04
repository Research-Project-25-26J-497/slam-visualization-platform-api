"""ORM models for the SLAM visualization platform.

Defines the persistent database schema for semantic annotations.
"""

from sqlalchemy import Column, Integer, String, Float, DateTime
from sqlalchemy.sql import func
from .db import Base

class Annotation(Base):
    """
    Persistent storage for semantic map annotations.
    Database only stores persistent annotations (typically < 100 rows).
    """
    __tablename__ = "annotations"

    id = Column(Integer, primary_key=True, index=True)
    label = Column(String, index=True)
    
    type = Column(String, default="OBJECT") 
    
    x = Column(Float, nullable=False)
    y = Column(Float, default=0.0)
    z = Column(Float, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __repr__(self):
        return f"<Annotation(id={self.id}, label='{self.label}', type='{self.type}')>"