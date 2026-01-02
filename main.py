from fastapi import FastAPI, Depends, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import List, Optional
import json
import time
from datetime import datetime

# Database imports (No change)
from database.db import get_db
from database.models import Annotation, MapPoint

app = FastAPI(title="M.A.N.T.I.S Gateway API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- GLOBAL STATE ---
# 🟢 CHANGE 1: Removed 'sim_process'. Added 'last_robot_heartbeat'
current_target = {"x": 0.0, "z": 0.0} 
last_robot_heartbeat: Optional[datetime] = None 

# --- MODELS ---
class WaypointRequest(BaseModel):
    robot_id: str
    x: float
    y: float = 0.0
    z: float

class AnnotationCreate(BaseModel):
    label: str
    x: float
    y: float = 0.0
    z: float

class MapPointCreate(BaseModel):
    x: float
    z: float
    y: float = 0.0
    confidence: float = 1.0 

class RobotTelemetry(BaseModel):
    robot_id: str
    x: float
    z: float
    angle: float

# --- WEBSOCKET MANAGER (No Change) ---
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        for connection in self.active_connections:
            try:
                await connection.send_text(json.dumps(message))
            except:
                self.disconnect(connection)

manager = ConnectionManager()

# --- ENDPOINTS ---

@app.get("/")
def read_root():
    return {"status": "ONLINE", "system": "M.A.N.T.I.S Backend"}

# 🟢 CHANGE 2: New Status Endpoint (Replaces Start/Stop)
@app.get("/api/system/status")
def get_system_status():
    global last_robot_heartbeat
    is_online = False
    
    # Logic: If we heard from the robot in the last 5 seconds, it's CONNECTED
    if last_robot_heartbeat and (datetime.now() - last_robot_heartbeat).total_seconds() < 5:
        is_online = True
            
    return {
        "backend": "ONLINE",
        "simulator_status": "CONNECTED" if is_online else "DISCONNECTED",
        "last_heartbeat": str(last_robot_heartbeat) if last_robot_heartbeat else "Never"
    }

@app.post("/api/command/waypoint")
async def set_waypoint(cmd: WaypointRequest):
    global current_target
    current_target = {"x": cmd.x, "z": cmd.z}
    await manager.broadcast({"type": "COMMAND_WAYPOINT", "robot_id": cmd.robot_id, "target": current_target})
    return {"status": "Sent"}

@app.post("/api/command/emergency_stop")
async def emergency_stop():
    await manager.broadcast({"type": "EMERGENCY_STOP"})
    return {"status": "HALTED"}

@app.get("/api/robot/target")
def get_target():
    return current_target

@app.post("/api/robot/telemetry")
async def update_robot_pose(data: RobotTelemetry):
    # 🟢 CHANGE 3: Update Heartbeat on Telemetry
    global last_robot_heartbeat
    last_robot_heartbeat = datetime.now()

    await manager.broadcast({
        "type": "ROBOT_POSE",
        "robot_id": data.robot_id,
        "x": data.x, "z": data.z, "angle": data.angle
    })
    return {"status": "ok"}

# ✅ MAP & HAZARD LOGIC (Option 1 Core)
@app.post("/api/map/batch")
async def receive_map_chunk(points: List[MapPointCreate], db: Session = Depends(get_db)):
    # 🟢 CHANGE 4: Update Heartbeat on Map Data
    global last_robot_heartbeat
    last_robot_heartbeat = datetime.now()

    # 1. Save Points
    db_points = [MapPoint(x=p.x, y=p.y, z=p.z, confidence=p.confidence) for p in points]
    db.bulk_save_objects(db_points)
    
    # 2. 🧠 ANALYZE FOR HAZARDS (Low Intensity + Low Height = Water)
    detected_hazards = []
    for p in points:
        # ⚠️ This matches your simulate_slam.py puddle generation!
        if p.y < 0.1 and p.confidence < 0.2:
            existing = db.query(Annotation).filter(
                Annotation.x > p.x - 1.0, Annotation.x < p.x + 1.0,
                Annotation.z > p.z - 1.0, Annotation.z < p.z + 1.0
            ).first()
            
            if not existing:
                print(f"⚠️ HAZARD DETECTED at {p.x:.1f}, {p.z:.1f}")
                new_ann = Annotation(label="💧 Water Leak (Auto)", x=p.x, y=p.y, z=p.z)
                db.add(new_ann)
                detected_hazards.append(new_ann)

    db.commit()
    
    # 3. Broadcast updates
    await manager.broadcast({"type": "MAP_UPDATE"})
    if detected_hazards:
        # This triggers the Frontend Alert!
        await manager.broadcast({"type": "HAZARD_ALERT"})
        
    return {"status": "Chunk Received"}

@app.get("/api/map")
def get_map(db: Session = Depends(get_db)):
    return db.query(MapPoint).limit(10000).all()

@app.post("/api/annotations")
def create_annotation(annotation: AnnotationCreate, db: Session = Depends(get_db)):
    new_ann = Annotation(label=annotation.label, x=annotation.x, y=annotation.y, z=annotation.z)
    db.add(new_ann)
    db.commit()
    db.refresh(new_ann)
    return new_ann

@app.get("/api/annotations")
def read_annotations(db: Session = Depends(get_db)):
    return db.query(Annotation).all()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True: await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)