from fastapi import FastAPI, Depends, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import List
import json
import time

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

# ✅ NEW: Telemetry Model
class RobotTelemetry(BaseModel):
    robot_id: str
    x: float
    z: float
    angle: float

# --- WEBSOCKET MANAGER ---
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
    return {"status": "ONLINE"}

@app.post("/api/command/waypoint")
async def set_waypoint(cmd: WaypointRequest):
    print(f"[COMMAND] Routing Robot {cmd.robot_id} to {cmd.x}, {cmd.z}")
    await manager.broadcast({
        "type": "COMMAND_WAYPOINT",
        "robot_id": cmd.robot_id,
        "target": {"x": cmd.x, "y": cmd.y, "z": cmd.z}
    })
    return {"status": "Sent"}

@app.post("/api/command/emergency_stop")
async def emergency_stop():
    print("[ALERT] EMERGENCY STOP")
    await manager.broadcast({"type": "EMERGENCY_STOP"})
    return {"status": "HALTED"}

# ✅ NEW: Telemetry Endpoint (Robot Position)
@app.post("/api/robot/telemetry")
async def update_robot_pose(data: RobotTelemetry):
    # Send position to frontend instantly
    await manager.broadcast({
        "type": "ROBOT_POSE",
        "robot_id": data.robot_id,
        "x": data.x,
        "z": data.z,
        "angle": data.angle
    })
    return {"status": "ok"}

# Map Batch Upload
@app.post("/api/map/batch")
async def receive_map_chunk(points: List[MapPointCreate], db: Session = Depends(get_db)):
    db_points = [MapPoint(x=p.x, y=p.y, z=p.z, confidence=p.confidence) for p in points]
    db.bulk_save_objects(db_points)
    db.commit()
    # Notify Frontend
    await manager.broadcast({"type": "MAP_UPDATE"})
    return {"status": "Chunk Received"}

# Map Download
@app.get("/api/map")
def get_map(db: Session = Depends(get_db)):
    return db.query(MapPoint).limit(8000).all()

# Websocket
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)