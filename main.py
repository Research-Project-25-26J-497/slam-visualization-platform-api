from fastapi import FastAPI, Depends, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import List, Optional
import json
from datetime import datetime
import os 

# Database imports
from database.db import get_db
from database.models import Annotation

app = FastAPI(title="M.A.N.T.I.S Gateway API - Voxel Grid")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- CONFIGURATION ---
SNAPSHOT_FILE = "map_snapshot.json"
VOXEL_SIZE = 0.1  # 10cm Resolution (Professional Standard)

# --- GLOBAL STATE ---
# 🚀 CHANGED: Using a Dictionary for Voxel Grid instead of a List
# Key: "x,y,z" string | Value: The point object
global_map = {} 
map_version = 0 
current_target = {"x": 0.0, "z": 0.0}
last_robot_heartbeat: Optional[datetime] = None

# Change Detection Reference
reference_map = set()

# Hazard Grid for Spatial Hashing
hazard_grid = {} 
HAZARD_GRID_SIZE = 2.0
HAZARD_COOLDOWN = 10.0 

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
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(json.dumps(message))
            except:
                disconnected.append(connection)
        for conn in disconnected:
            self.disconnect(conn)

manager = ConnectionManager()

# --- ENDPOINTS ---

@app.get("/")
def read_root():
    return {"status": "ONLINE", "system": "M.A.N.T.I.S Backend"}

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
        "map_points": len(global_map) # Shows distinct voxels
    }

# --- SNAPSHOT FEATURES ---
@app.post("/api/map/snapshot")
def save_map_snapshot():
    global global_map
    try:
        # Convert dictionary back to list for saving
        data = list(global_map.values())
        with open(SNAPSHOT_FILE, "w") as f:
            json.dump(data, f)
        return {"status": "Saved", "count": len(data)}
    except Exception as e:
        return {"status": "Error", "detail": str(e)}

@app.post("/api/map/load")
def load_map_snapshot():
    global global_map, map_version, reference_map
    try:
        if os.path.exists(SNAPSHOT_FILE):
            with open(SNAPSHOT_FILE, "r") as f:
                data = json.load(f)
            
            # Rebuild Voxel Grid from file
            global_map.clear()
            reference_map.clear()
            
            for p in data:
                # 1. Add to Map
                key = f"{round(p['x'], 1)},{round(p['y'], 1)},{round(p['z'], 1)}"
                global_map[key] = p
                
                # 2. Add to Change Detection Reference (Comparison Layer)
                ref_key = f"{round(p['x'], 1)},{round(p['z'], 1)}"
                reference_map.add(ref_key)
            
            map_version += 1 
            return {"status": "Loaded", "count": len(global_map)}
        return {"status": "No snapshot found"}
    except Exception as e:
        return {"status": "Error", "detail": str(e)}

@app.delete("/api/map/clear")
def clear_map():
    global global_map, map_version, reference_map
    global_map.clear()
    reference_map.clear()
    map_version += 1
    return {"status": "Map cleared"}

# --- ROBOT CONTROL ---
@app.post("/api/command/waypoint")
async def set_waypoint(cmd: WaypointRequest):
    global current_target
    current_target = {"x": cmd.x, "z": cmd.z}
    await manager.broadcast({
        "type": "COMMAND_WAYPOINT", "robot_id": cmd.robot_id, "target": current_target
    })
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
    global last_robot_heartbeat
    last_robot_heartbeat = datetime.now()
    await manager.broadcast({
        "type": "ROBOT_POSE", "robot_id": data.robot_id, "x": data.x, "z": data.z, "angle": data.angle
    })
    return {"status": "ok"}

# --- MAPPING & HAZARDS ---
def get_hazard_grid_key(x: float, z: float) -> tuple:
    return (int(x / HAZARD_GRID_SIZE), int(z / HAZARD_GRID_SIZE))

@app.post("/api/map/batch")
async def receive_map_chunk(points: List[MapPointCreate], db: Session = Depends(get_db)):
    global last_robot_heartbeat, map_version, global_map, hazard_grid
    
    last_robot_heartbeat = datetime.now()
    current_time = datetime.now().timestamp()
    
    # 1. Voxel Grid Update (The "Memory" Fix)
    for p in points:
        # Create a unique key for this 10cm cube
        # format: "5.1,0.2,-3.4"
        key = f"{round(p.x, 1)},{round(p.y, 1)},{round(p.z, 1)}"
        
        # Insert/Overwrite (Deduplication happens automatically here)
        global_map[key] = {"x": p.x, "y": p.y, "z": p.z, "confidence": p.confidence}

    map_version += 1
    
    # 2. Detect Hazards & Changes
    alert_triggered = False
    
    for p in points:
        # A. WATER PUDDLE DETECTION
        if p.y < 0.1 and p.confidence < 0.2:
            grid_key = get_hazard_grid_key(p.x, p.z)
            if grid_key not in hazard_grid or (current_time - hazard_grid[grid_key]) > HAZARD_COOLDOWN:
                print(f"⚠️ HAZARD DETECTED at {p.x:.1f}, {p.z:.1f}")
                hazard_grid[grid_key] = current_time
                alert_triggered = True

        # B. CHANGE DETECTION (New Object vs Reference)
        if len(reference_map) > 0 and p.y > 0.2:
             # Check if this object exists in our "Saved Snapshot"
             # We use 2D comparison (x,z) for simpler object detection
             ref_key = f"{round(p.x, 1)},{round(p.z, 1)}"
             
             if ref_key not in reference_map:
                 grid_key = get_hazard_grid_key(p.x, p.z)
                 if grid_key not in hazard_grid or (current_time - hazard_grid[grid_key]) > HAZARD_COOLDOWN:
                    print(f"🚨 CHANGE DETECTED: Unknown Object at {p.x:.1f}, {p.z:.1f}")
                    # Trigger a different alert if needed
                    await manager.broadcast({"type": "HAZARD_ALERT", "message": "New Object Detected!"})
                    hazard_grid[grid_key] = current_time

    # 3. Broadcast
    # Note: We send the incremental update to WebSocket, but the GET /api/map will return the FULL map
    if len(points) > 0:
        await manager.broadcast({"type": "MAP_UPDATE"})
    if alert_triggered:
        await manager.broadcast({"type": "HAZARD_ALERT"})
        
    return {"status": "Chunk Received"}

@app.get("/api/map")
def get_map(version: Optional[int] = None):
    # Only return map if version changed
    if version is not None and version == map_version:
        return {"status": "not_modified", "version": map_version}
    
    # Return ALL values from the Voxel Grid
    return {"status": "ok", "version": map_version, "points": list(global_map.values())}

@app.get("/api/annotations")
def read_annotations(db: Session = Depends(get_db)):
    return db.query(Annotation).all()

@app.post("/api/annotations")
def create_annotation(annotation: AnnotationCreate, db: Session = Depends(get_db)):
    new_ann = Annotation(label=annotation.label, x=annotation.x, y=annotation.y, z=annotation.z)
    db.add(new_ann)
    db.commit()
    return new_ann

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True: await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)