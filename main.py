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

# ── Multi-robot state ───────────────────────────────────────────────────────────
robot_poses: dict = {}          # {robot_id: {id, color, x, z, angle, status}}
position_history: list = []     # [{x, z}] – capped, used for heatmap density
MAX_POSITION_HISTORY = 5000

ROBOT_COLORS: dict = {
    "robot1": "orange",
    "robot2": "#22d3ee",
    "robot3": "#4ade80",
    "robot4": "#f472b6",
}

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
    color: Optional[str] = None  # forwarded by ros2_web_bridge; falls back to ROBOT_COLORS

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
    """Returns a density heatmap derived from actual robot position history."""
    global position_history
    if not position_history:
        return []

    CELL_SIZE = 1.0
    cell_counts: dict = {}
    for pos in position_history:
        cx = round(pos["x"] / CELL_SIZE) * CELL_SIZE
        cz = round(pos["z"] / CELL_SIZE) * CELL_SIZE
        key = (cx, cz)
        cell_counts[key] = cell_counts.get(key, 0) + 1

    if not cell_counts:
        return []

    max_count = max(cell_counts.values())
    return [
        {"x": k[0], "z": k[1], "intensity": round(v / max_count, 3)}
        for k, v in cell_counts.items()
        if v / max_count > 0.05  # filter near-zero cells
    ]

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
    global global_map, map_version, robot_poses, position_history, hazard_grid, last_robot_heartbeat
    global_map.clear()
    robot_poses.clear()
    position_history.clear()
    hazard_grid.clear()
    last_robot_heartbeat = None
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
    global last_robot_heartbeat, robot_poses, position_history
    last_robot_heartbeat = datetime.now()

    # Resolve color: use explicit value from bridge, or fall back to known palette
    color = data.color or ROBOT_COLORS.get(data.robot_id, "orange")
    robot_poses[data.robot_id] = {
        "id":     data.robot_id,
        "color":  color,
        "x":      data.x,
        "z":      data.z,
        "angle":  data.angle,
        "status": data.status,
    }

    # Track position for real-time heatmap density
    position_history.append({"x": data.x, "z": data.z})
    if len(position_history) > MAX_POSITION_HISTORY:
        position_history.pop(0)

    # Legacy single-robot message (backwards-compatible with simulate_slam.py)
    await manager.broadcast({
        "type":     "ROBOT_POSE",
        "robot_id": data.robot_id,
        "x":        data.x,
        "z":        data.z,
        "angle":    data.angle,
        "status":   data.status,
    })
    # Multi-robot message consumed by MapCanvas.tsx
    await manager.broadcast({
        "type":   "MULTI_ROBOT_POSES",
        "robots": list(robot_poses.values()),
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