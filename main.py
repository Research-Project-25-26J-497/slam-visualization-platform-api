from fastapi import FastAPI, Depends, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import List, Optional
import json
import random
from datetime import datetime
import os 

from database.db import get_db
from database.models import Annotation

app = FastAPI(title="M.A.N.T.I.S Gateway API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

SNAPSHOT_FILE = "map_snapshot.json"
global_map = {} 
map_version = 0 
current_target = {"x": 0.0, "z": 0.0}
last_robot_heartbeat: Optional[datetime] = None
hazard_grid = {} 
HAZARD_GRID_SIZE = 2.0
HAZARD_COOLDOWN = 15.0 

class WaypointRequest(BaseModel):
    robot_id: str
    x: float
    y: float = 0.0
    z: float

class AnnotationCreate(BaseModel):
    label: str
    type: str
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
    status: Optional[str] = "IDLE"

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
        disconnected = []
        for connection in self.active_connections:
            try: await connection.send_text(json.dumps(message))
            except: disconnected.append(connection)
        for conn in disconnected: self.disconnect(conn)

manager = ConnectionManager()

@app.get("/")
def read_root(): return {"status": "ONLINE"}

@app.get("/api/system/status")
def get_system_status():
    global last_robot_heartbeat
    is_online = False
    if last_robot_heartbeat and (datetime.now() - last_robot_heartbeat).total_seconds() < 5:
        is_online = True
    return {
        "backend": "ONLINE",
        "simulator_status": "CONNECTED" if is_online else "DISCONNECTED",
        "last_heartbeat": str(last_robot_heartbeat) if last_robot_heartbeat else "Never",
        "map_points": len(global_map)
    }

@app.get("/api/analytics/collisions")
def get_collision_heatmap():
    data = []
    for _ in range(25):
        data.append({
            "x": 3.0 + random.uniform(-0.8, 0.8),
            "z": -3.0 + random.uniform(-0.8, 0.8),
            "intensity": random.uniform(0.5, 1.0)
        })
    for _ in range(15):
        data.append({
            "x": -5.0 + random.uniform(-1.0, 1.0),
            "z": 0.0 + random.uniform(-1.0, 1.0),
            "intensity": random.uniform(0.3, 0.8)
        })
    return data

@app.post("/api/map/snapshot")
def save_map_snapshot():
    global global_map
    try:
        data = list(global_map.values())
        with open(SNAPSHOT_FILE, "w") as f: json.dump(data, f)
        return {"status": "Saved", "count": len(data)}
    except Exception as e: return {"status": "Error", "detail": str(e)}

@app.post("/api/map/load")
def load_map_snapshot():
    global global_map, map_version
    try:
        if os.path.exists(SNAPSHOT_FILE):
            with open(SNAPSHOT_FILE, "r") as f: data = json.load(f)
            global_map.clear()
            for p in data:
                key = f"{round(p['x'], 1)},{round(p['y'], 1)},{round(p['z'], 1)}"
                global_map[key] = p
            map_version += 1 
            return {"status": "Loaded", "count": len(global_map)}
        return {"status": "No snapshot found"}
    except Exception as e: return {"status": "Error", "detail": str(e)}

@app.delete("/api/map/clear")
def clear_map():
    global global_map, map_version
    global_map.clear()
    map_version += 1
    return {"status": "Map cleared"}

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
def get_target(): return current_target

@app.post("/api/robot/telemetry")
async def update_robot_pose(data: RobotTelemetry):
    global last_robot_heartbeat
    last_robot_heartbeat = datetime.now()
    await manager.broadcast({
        "type": "ROBOT_POSE", 
        "robot_id": data.robot_id, 
        "x": data.x, 
        "z": data.z, 
        "angle": data.angle,
        "status": data.status 
    })
    return {"status": "ok"}

@app.post("/api/map/batch")
async def receive_map_chunk(points: List[MapPointCreate], db: Session = Depends(get_db)):
    global last_robot_heartbeat, map_version, global_map, hazard_grid
    last_robot_heartbeat = datetime.now()
    current_time = datetime.now().timestamp()
    
    for p in points:
        key = f"{round(p.x, 1)},{round(p.y, 1)},{round(p.z, 1)}"
        global_map[key] = {"x": p.x, "y": p.y, "z": p.z, "confidence": p.confidence}

    map_version += 1
    
    for p in points:
        if p.y < 0.1 and p.confidence < 0.2:
            grid_key = (int(p.x / HAZARD_GRID_SIZE), int(p.z / HAZARD_GRID_SIZE))
            if grid_key not in hazard_grid or (current_time - hazard_grid[grid_key]) > HAZARD_COOLDOWN:
                await manager.broadcast({"type": "HAZARD_ALERT", "message": "⚠️ Liquid Hazard Detected!"})
                hazard_grid[grid_key] = current_time

    if len(points) > 0: await manager.broadcast({"type": "MAP_UPDATE"})
    return {"status": "Chunk Received"}

@app.get("/api/map")
def get_map(version: Optional[int] = None):
    if version is not None and version == map_version:
        return {"status": "not_modified", "version": map_version}
    return {"status": "ok", "version": map_version, "points": list(global_map.values())}

@app.get("/api/annotations")
def read_annotations(db: Session = Depends(get_db)): return db.query(Annotation).all()

@app.post("/api/annotations")
def create_annotation(annotation: AnnotationCreate, db: Session = Depends(get_db)):
    """
    Fulfills User-Interactive Annotation System objective[cite: 211, 214].
    Ensures semantic labels (HAZARD/OBJECT) are persisted in the Neon DB.
    """
    new_ann = Annotation(
        label=annotation.label, 
        type=annotation.type,  # ✅ This MUST be here to save the hazard status
        x=annotation.x, 
        y=annotation.y, 
        z=annotation.z
    )
    db.add(new_ann)
    db.commit()
    db.refresh(new_ann) # Refresh to get the ID and confirm the type
    return new_ann

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True: await websocket.receive_text()
    except WebSocketDisconnect: manager.disconnect(websocket)