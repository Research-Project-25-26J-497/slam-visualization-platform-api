import time
import math
import random
import requests
import datetime
import heapq

# --- CONFIGURATION ---
API_URL = "http://127.0.0.1:8000"
ROBOT_ID = "R1"
MOVE_SPEED = 0.1         
MAP_UPLOAD_INTERVAL = 0.1 
GRID_SIZE = 0.5           
LIDAR_RANGE = 4

# --- GLOBAL STATE ---
robot_x = -8.5
robot_z = 6.0
robot_angle = 0

# Set the initial target to a different location so it starts moving immediately!
# Let's tell it to drive out of the room and into the central corridor
last_target = {"x": -2.0, "z": 0.0} 
current_path = [] 
last_map_upload = datetime.datetime.now()

# --- PHYSICS & MAP ---
class DynamicGrid:
    def __init__(self):
        self.obstacles = set()
    def mark_obstacle(self, x, z):
        self.obstacles.add((round(x/GRID_SIZE), round(z/GRID_SIZE)))
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

def check_map_collision(x, z):
    # 1. Outer Perimeter (Square 20x20m based on visual scale)
    if abs(x) > 10 or abs(z) > 10: return True

    # 2. The "Cross" Dividers (Corridor Walls)
    # These walls create the 4-room split but stop before the center hub
    wall_thickness = 0.2
    if abs(x) < wall_thickness and abs(z) > 2: return True # Vertical walls
    if abs(z) < wall_thickness and abs(x) > 2: return True # Horizontal walls

    # 3. Objects by Room (Matching the Screenshot Colors/Shapes)
    
    # NW Room (Top-Left): Grey & Blue Boxes
    if (-8 < x < -6 and 6 < z < 8): return True # Grey Box
    if (-5 < x < -3 and 5 < z < 7): return True # Blue Box

    # NE Room (Top-Right): Green & Red Cylinders
    if math.sqrt((x - 3)**2 + (z - 7)**2) < 0.7: return True # Green Cylinder
    if math.sqrt((x - 6)**2 + (z - 7)**2) < 0.8: return True # Red Cylinder

    # SW Room (Bottom-Left): Two Brown Boxes
    if (-7 < x < -5 and -7 < z < -5): return True 
    if (-4 < x < -2 and -8 < z < -6): return True

    # SE Room (Bottom-Right): Two Brown Racks
    if (4 < x < 8 and -4 < z < -3): return True # Top Rack
    if (5 < x < 9 and -7 < z < -6): return True # Bottom Rack

    # Center Hub: Yellow Box
    if abs(x) < 0.6 and abs(z) < 0.6: return True

    return False

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
        if iterations > 8000: return []
        _, current = heapq.heappop(frontier)
        if current == goal_node: break
        for dx, dz in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            next_node = (current[0]+dx, current[1]+dz)
            if next_node in robot_map.obstacles: continue
            new_cost = cost_so_far[current] + 1
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + math.sqrt((goal_node[0]-next_node[0])**2 + (goal_node[1]-next_node[1])**2)
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

def scan_environment_truth(rx, rz):
    detected = []
    for angle in range(0, 360, 5): 
        rad = math.radians(angle)
        for dist in range(1, int(LIDAR_RANGE * 10)):
            d = dist / 10.0
            tx = rx + d * math.cos(rad)
            tz = rz + d * math.sin(rad)
            if check_map_collision(tx, tz):
                detected.append((tx, tz))
                break 
    return detected

def generate_lidar_cloud(rx, rz):
    points = []
    for angle in range(0, 360, 2): # Higher density for the new objects
        rad = math.radians(angle)
        for dist in range(1, int(LIDAR_RANGE * 10)):
            d = dist / 10.0
            tx = rx + d * math.cos(rad)
            tz = rz + d * math.sin(rad)
            
            if check_map_collision(tx, tz):
                # Height logic based on visual object sizes
                h = 2.0 # Walls
                if abs(tx) < 0.6 and abs(tz) < 0.6: h = 0.8 # Yellow box is shorter
                elif (3 < tx < 9 and -8 < tz < -3): h = 0.6 # Racks are low
                elif math.sqrt((tx-3)**2+(tz-7)**2) < 1: h = 1.2 # Cylinders
                
                points.append({
                    "x": tx + random.uniform(-0.01, 0.01), 
                    "y": random.uniform(0.1, h), 
                    "z": tz + random.uniform(-0.01, 0.01), 
                    "confidence": 0.98
                })
                break
    return points

# --- MAIN LOOP ---
print("🤖 ROBOT ONLINE | MODE: Standard Mapping (Multi-Room Warehouse)")

loop_count = 0
while True:
    loop_count += 1
    try:
        resp = requests.get(f"{API_URL}/api/robot/target", timeout=0.1)
        if resp.status_code == 200:
            t = resp.json()
            if t["x"] != last_target["x"] or t["z"] != last_target["z"]:
                print(f"📍 New Target: {t['x']}, {t['z']}")
                last_target = t
                current_path = []
    except: pass

    visible = scan_environment_truth(robot_x, robot_z)
    map_updated = False
    for (ox, oz) in visible:
        if not robot_map.is_blocked(ox, oz):
            robot_map.mark_obstacle(ox, oz)
            map_updated = True

    # --- STATUS CALCULATION BLOCK ---
    robot_status = "💤 IDLE"
    if current_path:
        robot_status = "🚀 Speed: 0.5 m/s"
    
    if map_updated and robot_map.is_path_blocked(current_path):
        robot_status = "⚠️ OBSTACLE! RECALCULATING..."
    # --------------------------------

    if (map_updated and robot_map.is_path_blocked(current_path)) or not current_path:
        current_path = astar((robot_x, robot_z), (last_target["x"], last_target["z"]))

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
            try: 
                requests.post(f"{API_URL}/api/robot/telemetry", 
                    json={
                        "robot_id": ROBOT_ID, 
                        "x": robot_x, 
                        "z": robot_z, 
                        "angle": angle,
                        "status": robot_status
                    }, timeout=0.1)
            except: pass

    if (datetime.datetime.now() - last_map_upload).total_seconds() > MAP_UPLOAD_INTERVAL:
        try: requests.post(f"{API_URL}/api/map/batch", json=generate_lidar_cloud(robot_x, robot_z), timeout=0.1)
        except: pass
        last_map_upload = datetime.datetime.now()
    time.sleep(MOVE_SPEED)