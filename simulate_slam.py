import time
import math
import random
import requests

API_URL = "http://127.0.0.1:8000"
ROBOT_ID = "R1"
SPEED = 0.05 

def upload_telemetry(x, z, angle):
    try:
        requests.post(f"{API_URL}/api/robot/telemetry", json={
            "robot_id": ROBOT_ID, "x": x, "z": z, "angle": angle
        })
    except: pass

def upload_map_chunk(points):
    if not points: return
    try:
        requests.post(f"{API_URL}/api/map/batch", json=points)
        print(f"✨ Uploaded {len(points)} voxels")
    except: pass

def generate_complex_room(rx, rz):
    points = []
    
    def add_voxel_cluster(x, y, z):
        for _ in range(3):
            points.append({
                "x": x + random.uniform(-0.05, 0.05),
                "y": y + random.uniform(-0.05, 0.05),
                "z": z + random.uniform(-0.05, 0.05),
                "confidence": 0.95
            })

    # 1. TABLE (Center: 0,0)
    dist = math.sqrt(rx**2 + rz**2)
    if dist < 4:
        if random.random() > 0.3: # Top
            add_voxel_cluster(random.uniform(-1.0, 1.0), 1.0, random.uniform(-1.0, 1.0))
        if random.random() > 0.6: # Legs
            add_voxel_cluster(0.9, random.uniform(0, 1), 0.9)
            add_voxel_cluster(-0.9, random.uniform(0, 1), 0.9)
            add_voxel_cluster(0.9, random.uniform(0, 1), -0.9)
            add_voxel_cluster(-0.9, random.uniform(0, 1), -0.9)

    # 2. SOFA (x = -8, z = 0)
    if abs(rx - (-8)) < 4 and abs(rz) < 3:
        add_voxel_cluster(-8 + random.uniform(-0.5, 1.0), 0.5, random.uniform(-2, 2))
        add_voxel_cluster(-8.5, random.uniform(0.5, 1.2), random.uniform(-2, 2))

    # 3. PUDDLE (Hazard at x=5, z=5)
    if abs(rx - 5) < 3 and abs(rz - 5) < 3:
        add_voxel_cluster(5 + random.uniform(-1.5, 1.5), 0.02, 5 + random.uniform(-1.5, 1.5))

    # 4. WALLS
    if abs(rz - 10) < 4: add_voxel_cluster(rx + random.uniform(-1,1), random.uniform(0, 2), 10)
    if abs(rz - (-10)) < 4: add_voxel_cluster(rx + random.uniform(-1,1), random.uniform(0, 2), -10)
    if abs(rx - 10) < 4: add_voxel_cluster(10, random.uniform(0, 2), rz + random.uniform(-1,1))
    if abs(rx - (-10)) < 4: add_voxel_cluster(-10, random.uniform(0, 2), rz + random.uniform(-1,1))

    return points

# --- MAIN LOOP ---
print("🤖 STARTING SIMULATION...")
t = 0
while True:
    t += 0.05
    x = 6 * math.cos(t)
    z = 3 * math.sin(2 * t)
    
    dx = -6 * math.sin(t)
    dz = 6 * math.cos(2 * t)
    angle = -math.atan2(dz, dx)

    upload_telemetry(x, z, angle)
    new_points = generate_complex_room(x, z)
    upload_map_chunk(new_points)
    
    time.sleep(SPEED)