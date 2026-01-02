import time
import math
import random
import requests
import datetime
import sys # 👈 Needed for self-termination

API_URL = "http://127.0.0.1:8000"
ROBOT_ID = "R1"

# CONFIGURATION
MOVE_SPEED = 0.1      
MAP_UPLOAD_INTERVAL = 1.0  

# Global State
robot_x = 0
robot_z = 0
robot_angle = 0
last_target = {"x": 0, "z": 0} 
last_map_upload = datetime.datetime.now()

def check_backend_connection():
    """WATCHDOG: Kills simulation if Backend is offline."""
    try:
        requests.get(f"{API_URL}/", timeout=0.5)
    except:
        print("❌ Backend Offline! Stopping Simulation to prevent Zombie process.")
        sys.exit(0) # Suicide command

def get_target_from_backend():
    global last_target
    try:
        resp = requests.get(f"{API_URL}/api/robot/target", timeout=0.2)
        if resp.status_code == 200:
            last_target = resp.json()
    except: pass
    return last_target

def upload_telemetry(x, z, angle):
    try:
        requests.post(f"{API_URL}/api/robot/telemetry", json={
            "robot_id": ROBOT_ID, "x": x, "z": z, "angle": angle
        }, timeout=0.1)
    except: pass

def upload_map_chunk(points):
    if not points: return
    try:
        requests.post(f"{API_URL}/api/map/batch", json=points, timeout=0.5)
    except: pass

def generate_complex_room(rx, rz):
    points = []
    
    def add_lidar_points(x, y, z, count=20, spread=0.15, intensity=0.8):
        for _ in range(count):
            points.append({
                "x": x + random.uniform(-spread, spread),
                "y": y + random.uniform(-0.01, 0.01),
                "z": z + random.uniform(-spread, spread),
                "confidence": intensity 
            })

    # 1. PUDDLE (Hazard) - Low Intensity
    if abs(rx - 5) < 3 and abs(rz - 5) < 3:
        add_lidar_points(5, 0.02, 5, count=15, intensity=0.1) 

    # 2. TABLE & LEGS
    dist = math.sqrt(rx**2 + rz**2)
    if dist < 4.5: 
        if random.random() > 0.1: 
             add_lidar_points(random.uniform(-1.0, 1.0), 1.0, random.uniform(-1.0, 1.0), count=8, intensity=0.9)
        legs = [(0.9, 0.9), (-0.9, 0.9), (0.9, -0.9), (-0.9, -0.9)] 
        for lx, lz in legs:
            if abs(rx - lx) < 2 and abs(rz - lz) < 2:
                add_lidar_points(lx, random.uniform(0, 1), lz, count=8, spread=0.08, intensity=0.8)

    # 3. WALLS
    if abs(rz - 10) < 4: add_lidar_points(rx, random.uniform(0, 2), 10, count=25)
    if abs(rz - (-10)) < 4: add_lidar_points(rx, random.uniform(0, 2), -10, count=25)
    if abs(rx - 10) < 4: add_lidar_points(10, random.uniform(0, 2), rz, count=25)
    if abs(rx - (-10)) < 4: add_lidar_points(-10, random.uniform(0, 2), rz, count=25)
    
    return points

# --- MAIN LOOP ---
print("🤖 ROBOT ONLINE: Watchdog Active...")

while True:
    # 0. WATCHDOG CHECK (Safety First)
    check_backend_connection()

    # 1. Ask Backend
    target = get_target_from_backend()
    target_x = target["x"]
    target_z = target["z"]

    # 2. Move Logic
    dx = target_x - robot_x
    dz = target_z - robot_z
    dist = math.sqrt(dx**2 + dz**2)

    if dist > 0.5:
        target_angle = -math.atan2(dz, dx)
        robot_angle = target_angle 
        step_size = 0.5 
        robot_x += step_size * math.cos(-robot_angle)
        robot_z += step_size * math.sin(-robot_angle)
        upload_telemetry(robot_x, robot_z, robot_angle)

    # 3. Map Upload (Throttled)
    now = datetime.datetime.now()
    if (now - last_map_upload).total_seconds() > MAP_UPLOAD_INTERVAL:
        print(f"🔄 Scanning... (Pos: {robot_x:.1f}, {robot_z:.1f})")
        new_points = generate_complex_room(robot_x, robot_z)
        upload_map_chunk(new_points)
        last_map_upload = now

    time.sleep(MOVE_SPEED)