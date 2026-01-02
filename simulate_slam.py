import time
import math
import random
import requests
import datetime
import sys
import heapq

API_URL = "http://127.0.0.1:8000"
ROBOT_ID = "R1"

# --- CONFIGURATION ---
MOVE_SPEED = 0.03        
MAP_UPLOAD_INTERVAL = 0.1 
GRID_SIZE = 0.5         
LIDAR_RANGE = 5.0  # Increased range slightly for bigger rooms

# --- GLOBAL STATE ---
# Start in "Room A" (Top Left)
robot_x = -6.0
robot_z = -6.0
robot_angle = 0
last_target = {"x": -6.0, "z": -6.0}
current_path = [] 
last_map_upload = datetime.datetime.now()

# --- 1. DYNAMIC GRID ---
class DynamicGrid:
    def __init__(self):
        self.obstacles = set()
    
    def mark_obstacle(self, x, z):
        self.obstacles.add((round(x/GRID_SIZE), round(z/GRID_SIZE)))
        # Add padding (Safety Buffer)
        for dx in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                 self.obstacles.add((round(x/GRID_SIZE)+dx, round(z/GRID_SIZE)+dz))

    def is_blocked(self, x, z):
        return (round(x/GRID_SIZE), round(z/GRID_SIZE)) in self.obstacles

    def is_path_blocked(self, path):
        for (px, pz) in path:
            if self.is_blocked(px, pz): return True
        return False

robot_map = DynamicGrid()

# --- 2. A* ALGORITHM ---
def heuristic(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def astar(start, goal):
    start_node = (round(start[0]/GRID_SIZE), round(start[1]/GRID_SIZE))
    goal_node = (round(goal[0]/GRID_SIZE), round(goal[1]/GRID_SIZE))
    if start_node == goal_node: return []

    frontier = []
    heapq.heappush(frontier, (0, start_node))
    came_from = {start_node: None}
    cost_so_far = {start_node: 0}
    iterations = 0

    while frontier:
        iterations += 1
        if iterations > 8000: return [] # Increased limit for complex maze
        _, current = heapq.heappop(frontier)
        if current == goal_node: break

        for dx, dz in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            next_node = (current[0]+dx, current[1]+dz)
            if next_node in robot_map.obstacles: continue
            
            new_cost = cost_so_far[current] + 1
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(goal_node, next_node)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
                
    if goal_node not in came_from: return []
    path = []
    curr = goal_node
    while curr != start_node:
        path.append((curr[0]*GRID_SIZE, curr[1]*GRID_SIZE))
        curr = came_from[curr]
    path.reverse()
    return path

# --- 3. COMPLEX MAP DEFINITION ---
def check_map_collision(x, z):
    """
    Returns TRUE if (x,z) hits a wall or object.
    This defines the 'Physics' of our world.
    """
    # 1. Outer Boundary Walls (20m x 20m)
    if x > 10 or x < -10 or z > 10 or z < -10: return True

    # 2. Central Corridor Walls
    # Wall separating Left Rooms from Right Rooms
    # Gap (Doorway) at z = [-2, 2]
    if abs(x) < 0.5 and (z > 2 or z < -2): return True

    # 3. Room B (Top Right) - The "Meeting Room"
    # Table is at (6, -6)
    dist_table = math.sqrt((x - 6)**2 + (z - (-6))**2)
    if dist_table < 2.5: return True # Table Top (Solid)

    # 4. Room C (Bottom Left) - "Storage Room"
    # Random Boxes
    if -8 < x < -4 and 4 < z < 8: return True # Big Crate

    # 5. Structural Pillars (Round)
    pillars = [(5, 5), (-5, 5)]
    for px, pz in pillars:
        if math.sqrt((x-px)**2 + (z-pz)**2) < 0.8: return True

    # 6. THE HAZARD (Puddle) - In the main corridor!
    # Located at (0, 0) right in the center doorway
    # Robot sees this as "Water", but physics sees it only if we want collision.
    # We DON'T return True here because robot *can* physically walk on water, 
    # but we want it to choose not to. (Handled by A* later if marked)
    
    return False

# --- 4. SENSOR SIMULATION ---
def scan_environment_truth(rx, rz):
    detected = []
    
    # Raycast in 360 degrees to find walls
    # This mimics a real Lidar much better than hardcoded boxes
    for angle in range(0, 360, 5): # Every 5 degrees
        rad = math.radians(angle)
        
        # Ray trace up to LIDAR_RANGE
        for dist in range(1, int(LIDAR_RANGE * 10)):
            d = dist / 10.0
            tx = rx + d * math.cos(rad)
            tz = rz + d * math.sin(rad)
            
            if check_map_collision(tx, tz):
                detected.append((tx, tz))
                break # Laser hit something, stop this ray

    # Special Check for Puddle (Semantic Object)
    # Robot "sees" it on the floor
    dist_puddle = math.sqrt(rx**2 + rz**2)
    if dist_puddle < LIDAR_RANGE:
        # Puddle geometry at (0,0) radius 1.5
        for px in [0, 0.5, -0.5, 1.0, -1.0]:
            for pz in [0, 0.5, -0.5, 1.0, -1.0]:
                detected.append((px, pz))

    return detected

def generate_lidar_cloud(rx, rz):
    points = []
    
    # 1. Puddle (Blue) - Center Corridor
    if math.sqrt(rx**2 + rz**2) < LIDAR_RANGE + 1:
        points.append({"x": random.uniform(-1.5, 1.5), "y": 0.02, "z": random.uniform(-1.5, 1.5), "confidence": 0.1})

    # 2. Raycast to generate Walls/Objects Visuals
    # We reuse the logic to spawn dots where walls are
    for angle in range(0, 360, 2): # Higher resolution for visuals
        rad = math.radians(angle)
        for dist in range(1, int(LIDAR_RANGE * 10)):
            d = dist / 10.0
            tx = rx + d * math.cos(rad)
            tz = rz + d * math.sin(rad)
            
            if check_map_collision(tx, tz):
                # Wall/Object Point
                points.append({
                    "x": tx + random.uniform(-0.1, 0.1),
                    "y": random.uniform(0.1, 2.0), # Wall height
                    "z": tz + random.uniform(-0.1, 0.1),
                    "confidence": 0.8
                })
                break

    # 3. Add Ceiling Lights (Just for visual flair)
    if random.random() > 0.9:
         points.append({"x": rx, "y": 3.0, "z": rz, "confidence": 0.5})

    return points

# --- MAIN LOOP ---
def check_backend():
    try: requests.get(f"{API_URL}/", timeout=20.0)
    except: sys.exit(0)

print("🤖 ROBOT ONLINE: Complex Office Map Active...")
loop_count = 0
while True:
    if loop_count % 30 == 0: check_backend()
    loop_count += 1
    
    # 1. Update Target
    try:
        resp = requests.get(f"{API_URL}/api/robot/target", timeout=0.1)
        if resp.status_code == 200:
            t = resp.json()
            if t["x"] != last_target["x"] or t["z"] != last_target["z"]:
                print(f"📍 New Target: {t['x']}, {t['z']}")
                last_target = t
    except: pass

    # 2. Scan & Re-plan
    visible = scan_environment_truth(robot_x, robot_z)
    map_updated = False
    for (ox, oz) in visible:
        if not robot_map.is_blocked(ox, oz):
            robot_map.mark_obstacle(ox, oz)
            map_updated = True
            
    if (map_updated and robot_map.is_path_blocked(current_path)) or not current_path:
        if map_updated: print("🚧 Environment Changed! Re-calculating...")
        current_path = astar((robot_x, robot_z), (last_target["x"], last_target["z"]))

    # 3. Move
    if current_path:
        wp_x, wp_z = current_path[0]
        dx = wp_x - robot_x
        dz = wp_z - robot_z
        if math.sqrt(dx**2+dz**2) < 0.3:
            current_path.pop(0)
        else:
            angle = -math.atan2(dz, dx)
            robot_x += 0.2 * math.cos(-angle)
            robot_z += 0.2 * math.sin(-angle)
            robot_angle = angle
            try: requests.post(f"{API_URL}/api/robot/telemetry", json={"robot_id": ROBOT_ID, "x": robot_x, "z": robot_z, "angle": angle}, timeout=0.1)
            except: pass

    # 4. Upload Visuals
    if (datetime.datetime.now() - last_map_upload).total_seconds() > MAP_UPLOAD_INTERVAL:
        try: requests.post(f"{API_URL}/api/map/batch", json=generate_lidar_cloud(robot_x, robot_z), timeout=0.1)
        except: pass
        last_map_upload = datetime.datetime.now()
        
    time.sleep(MOVE_SPEED)