"""Robot simulation client for the slam visualization platform.

This script emulates a simple robot that scans the environment, uploads lidar points,
and posts telemetry to the FastAPI backend.
"""

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
robot_x = 0
robot_z = 0
robot_angle = 0
last_target = {"x": 0, "z": 0}
current_path = [] 
last_map_upload = datetime.datetime.now()

# --- PHYSICS & MAP ---
class DynamicGrid:
    """In-memory occupancy grid used to represent scanned obstacles."""

    def __init__(self):
        """Initialize an empty grid of obstacle cells."""
        self.obstacles = set()

    def mark_obstacle(self, x, z):
        """Mark a point and its surrounding neighbor cells as occupied."""
        self.obstacles.add((round(x/GRID_SIZE), round(z/GRID_SIZE)))
        for dx in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                self.obstacles.add((round(x/GRID_SIZE)+dx, round(z/GRID_SIZE)+dz))

    def is_blocked(self, x, z):
        """Return True if the given coordinate is blocked by a known obstacle."""
        return (round(x/GRID_SIZE), round(z/GRID_SIZE)) in self.obstacles

    def is_path_blocked(self, path):
        """Check whether any waypoint along a path intersects a blocked cell."""
        for (px, pz) in path:
            if self.is_blocked(px, pz):
                return True
        return False

robot_map = DynamicGrid()


def astar(start, goal):
    """Compute a low-resolution path from start to goal using the A* algorithm."""
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
    """Perform a simulated 360-degree scan and return the first detected obstacles."""
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
    # Hazard Sensor
    if math.sqrt((rx - 3.0)**2 + (rz - (-3.0))**2) < LIDAR_RANGE:
        for theta in range(0, 360, 20):
            rad = math.radians(theta)
            px = 3.0 + 1.2 * math.cos(rad)
            pz = -3.0 + 1.2 * math.sin(rad)
            detected.append((px, pz))
    return detected

def generate_lidar_cloud(rx, rz):
    """Generate a synthetic lidar point cloud for the robot's current pose."""
    points = []
    # Walls (room boundaries)
    for angle in range(0, 360, 2):
        rad = math.radians(angle)
        for dist in range(1, int(LIDAR_RANGE * 10)):
            d = dist / 10.0
            tx = rx + d * math.cos(rad)
            tz = rz + d * math.sin(rad)
            if check_map_collision(tx, tz):
                h = 2.0  # Default wall height
                
                # Box 1 (small cube) - moved left
                if -2 < tx < -1 and -1 < tz < 0: 
                    h = 0.8
                # Box 2 (small cube) - moved left  
                elif -7 < tx < -6 and 2 < tz < 3:
                    h = 0.8
                # Box 3 (small cube) - moved left
                elif 0 < tx < 1 and -3 < tz < -2:
                    h = 0.8
                # Table (larger surface with legs) - moved left
                elif -8 < tx < -5 and -2 < tz < 1:
                    h = 0.7
                
                points.append({
                    "x": tx + random.uniform(-0.05, 0.05), 
                    "y": random.uniform(0.1, h), 
                    "z": tz + random.uniform(-0.05, 0.05), 
                    "confidence": 0.9
                })
                break
    return points

def check_map_collision(x, z):
    """Check whether a point collides with any static scene geometry."""
    # Room boundaries (single room 20x12 units)
    if x > 10 or x < -10 or z > 6 or z < -6: 
        return True
    
    # Box 1 (at position: -1.5, -0.5) - moved left
    if -2 < x < -1 and -1 < z < 0:
        return True
    
    # Box 2 (at position: -6.5, 2.5) - moved left
    if -7 < x < -6 and 2 < z < 3:
        return True
    
    # Box 3 (at position: 0.5, -2.5) - moved left
    if 0 < x < 1 and -3 < z < -2:
        return True
    
    # Table (at position: -6.5, -0.5) - moved left
    # Table top
    if -8 < x < -5 and -2 < z < 1:
        return True
    # Table legs (optional - for more realistic collision)
    # Leg 1
    if -7.8 < x < -7.4 and -1.8 < z < -1.4:
        return True
    # Leg 2
    if -5.2 < x < -4.8 and -1.8 < z < -1.4:
        return True
    # Leg 3
    if -7.8 < x < -7.4 and 0.4 < z < 0.8:
        return True
    # Leg 4
    if -5.2 < x < -4.8 and 0.4 < z < 0.8:
        return True
    
    return False
# --- MAIN LOOP ---
print("🤖 ROBOT ONLINE | MODE: Standard Mapping")

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
                #  ADDED 'status' TO TELEMETRY DATA
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