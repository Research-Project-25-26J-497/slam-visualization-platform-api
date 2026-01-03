import time
import math
import random
import requests
import datetime
import sys
import heapq

# --- CONFIGURATION ---
API_URL = "http://127.0.0.1:8000"
ROBOT_ID = "R1"
MOVE_SPEED = 0.03         # Lower = Faster simulation loop
MAP_UPLOAD_INTERVAL = 0.1 # How often to send "LiDAR" points
GRID_SIZE = 0.5           # A* Pathfinding resolution
LIDAR_RANGE = 5.0 

# --- GLOBAL STATE ---
# Start safe in top-left corner
robot_x = -8.5
robot_z = 3.0
robot_angle = 0
last_target = {"x": -8.5, "z": 3.0}
current_path = [] 
last_map_upload = datetime.datetime.now()

# --- 1. DYNAMIC GRID (Mental Model of the Robot) ---
class DynamicGrid:
    def __init__(self):
        self.obstacles = set()
    
    def mark_obstacle(self, x, z):
        # Mark the grid cell as blocked
        self.obstacles.add((round(x/GRID_SIZE), round(z/GRID_SIZE)))
        # Mark neighbors to create a "safety buffer"
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

# --- 2. A* ALGORITHM (Pathfinding) ---
def heuristic(a, b): return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

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
        if iterations > 8000: return [] # Safety break for infinite loops
        
        _, current = heapq.heappop(frontier)
        
        if current == goal_node: break
        
        # 8-Directional movement
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
    
    # Reconstruct Path
    path = []
    curr = goal_node
    while curr != start_node:
        path.append((curr[0]*GRID_SIZE, curr[1]*GRID_SIZE))
        curr = came_from[curr]
    path.reverse()
    return path

# --- 3. MAP PHYSICS (The "Real" World) ---
def check_map_collision(x, z):
    # Walls
    if x > 10 or x < -10 or z > 6 or z < -6: return True
    if abs(x) < 0.3 and (z > 1.5 or z < -1.5): return True # Central divider
    
    # Furniture (Invisible to robot until scanned)
    if math.sqrt((x - (-5))**2 + (z - 0)**2) < 1.5: return True # Round Table
    if 4 < x < 8 and 4 < z < 5.8: return True # Sofa
    if 5 < x < 7 and 2 < z < 3: return True # Coffee Table
    if 5 < x < 7 and -5.8 < z < -4.5: return True # TV Unit
    
    # 💧 HAZARD PHYSICS: Puddle at (3.0, -3.0)
    # The robot cannot physically pass through this zone
    if math.sqrt((x - 3.0)**2 + (z - (-3.0))**2) < 1.2: return True 

    return False

# --- 4. SENSORS (Simulating LiDAR) ---
def scan_environment_truth(rx, rz):
    """Raycasts to find obstacles for navigation logic"""
    detected = []
    
    # 1. Standard Wall/Furniture Scan
    for angle in range(0, 360, 5): 
        rad = math.radians(angle)
        for dist in range(1, int(LIDAR_RANGE * 10)):
            d = dist / 10.0
            tx = rx + d * math.cos(rad)
            tz = rz + d * math.sin(rad)
            if check_map_collision(tx, tz):
                detected.append((tx, tz))
                break 

    # 2. 💧 HAZARD SENSOR (Simulating Floor Camera)
    # If the robot gets close to the puddle, it detects the perimeter as obstacles
    puddle_x, puddle_z = 3.0, -3.0
    dist_to_puddle = math.sqrt((rx - puddle_x)**2 + (rz - puddle_z)**2)
    
    if dist_to_puddle < LIDAR_RANGE:
        # Detect the edge of the puddle as an obstacle
        for theta in range(0, 360, 15):
            rad = math.radians(theta)
            px = puddle_x + 1.2 * math.cos(rad)
            pz = puddle_z + 1.2 * math.sin(rad)
            detected.append((px, pz))

    return detected

def generate_lidar_cloud(rx, rz):
    """Generates visual points for the frontend map"""
    points = []
    
    # 💧 HAZARD VISUALIZATION
    puddle_x, puddle_z = 3.0, -3.0
    if math.sqrt((rx - puddle_x)**2 + (rz - puddle_z)**2) < LIDAR_RANGE:
         for _ in range(30): # Denser points for visibility
             points.append({
                 "x": puddle_x + random.uniform(-0.8, 0.8), 
                 "y": 0.02,        # Low to ground (Liquid)
                 "z": puddle_z + random.uniform(-0.8, 0.8), 
                 "confidence": 0.1 # Low Confidence triggers "Blue" in frontend
             })

    # 🧱 WALL VISUALIZATION
    for angle in range(0, 360, 2):
        rad = math.radians(angle)
        for dist in range(1, int(LIDAR_RANGE * 10)):
            d = dist / 10.0
            tx = rx + d * math.cos(rad)
            tz = rz + d * math.sin(rad)
            if check_map_collision(tx, tz):
                # Assign height based on object type for 3D effect
                h = 2.0 
                if 4 < tx < 8 and 4 < tz < 5.8: h = 0.8
                elif 5 < tx < 7 and 2 < tz < 3: h = 0.5
                elif math.sqrt((tx - (-5))**2 + (tz - 0)**2) < 1.5: h = 1.0
                # Puddle "collision" should not generate tall walls visually
                if math.sqrt((tx - 3.0)**2 + (tz - (-3.0))**2) < 1.5: continue

                points.append({
                    "x": tx + random.uniform(-0.05, 0.05), 
                    "y": random.uniform(0.1, h), 
                    "z": tz + random.uniform(-0.05, 0.05), 
                    "confidence": 0.9 
                })
                break
    return points

# --- MAIN LOOP ---
def check_backend():
    try: requests.get(f"{API_URL}/", timeout=5.0) 
    except: pass 

print("🤖 ROBOT ONLINE | MODE: Hazard Avoidance Active")
print(f"🎯 Target: {last_target}")

loop_count = 0
while True:
    # 1. Heartbeat check
    if loop_count % 30 == 0: check_backend()
    loop_count += 1

    # 2. Get New Target from API
    try:
        resp = requests.get(f"{API_URL}/api/robot/target", timeout=0.1)
        if resp.status_code == 200:
            t = resp.json()
            if t["x"] != last_target["x"] or t["z"] != last_target["z"]:
                print(f"📍 New Target Received: {t['x']}, {t['z']}")
                last_target = t
                current_path = [] # Force replan
    except: pass

    # 3. Update Internal Map (SLAM)
    visible = scan_environment_truth(robot_x, robot_z)
    map_updated = False
    for (ox, oz) in visible:
        if not robot_map.is_blocked(ox, oz):
            robot_map.mark_obstacle(ox, oz)
            map_updated = True

    # 4. Pathfinding (Replan if map changed or no path exists)
    if (map_updated and robot_map.is_path_blocked(current_path)) or not current_path:
        current_path = astar((robot_x, robot_z), (last_target["x"], last_target["z"]))

    # 5. Movement Execution
    if current_path:
        wp_x, wp_z = current_path[0]
        dx = wp_x - robot_x
        dz = wp_z - robot_z
        
        # If close to waypoint, move to next
        if math.sqrt(dx**2+dz**2) < 0.3:
            current_path.pop(0)
        else:
            # Move towards waypoint
            angle = -math.atan2(dz, dx)
            robot_x += 0.2 * math.cos(-angle)
            robot_z += 0.2 * math.sin(-angle)
            robot_angle = angle
            
            # Send Position
            try: 
                requests.post(f"{API_URL}/api/robot/telemetry", 
                    json={"robot_id": ROBOT_ID, "x": robot_x, "z": robot_z, "angle": angle}, 
                    timeout=0.1
                )
            except: pass

    # 6. Upload Map Data (LiDAR Cloud)
    if (datetime.datetime.now() - last_map_upload).total_seconds() > MAP_UPLOAD_INTERVAL:
        try: 
            cloud = generate_lidar_cloud(robot_x, robot_z)
            if cloud:
                requests.post(f"{API_URL}/api/map/batch", json=cloud, timeout=0.1)
        except: pass
        last_map_upload = datetime.datetime.now()

    time.sleep(MOVE_SPEED)