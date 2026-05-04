"""
Map Manager Module
Handles map persistence, loading, and comparison logic.
Keeps map operations separate from API routes for cleaner architecture.
"""
import json
import os
from typing import Dict, List, Tuple
from datetime import datetime

SNAPSHOT_FILE = "map_snapshot.json"
LAST_SAVED_MAP_FILE = "map_snapshot_last_saved.json"


class MapManager:
    """Manages map operations including saving, loading, and comparison."""
    
    def __init__(self):
        """Initialize map manager state and load the most recently saved reference map."""
        self.current_map: Dict = {}  # Current generated map
        self.last_saved_map: Dict = {}  # Last saved reference map
        self.map_version: int = 0
        self.load_last_saved_on_startup()
    
    def load_last_saved_on_startup(self):
        """Load the last saved map from disk into memory on startup."""
        try:
            if os.path.exists(LAST_SAVED_MAP_FILE):
                with open(LAST_SAVED_MAP_FILE, "r") as f:
                    data = json.load(f)
                for p in data:
                    key = f"{round(p['x'], 1)},{round(p['y'], 1)},{round(p['z'], 1)}"
                    self.last_saved_map[key] = p
                print(f"✅ Loaded {len(self.last_saved_map)} points from last saved map")
        except Exception as e:
            print(f"⚠️ Could not load last saved map: {e}")
    
    def add_points(self, points: List[Dict]):
        """Add points to the current map."""
        for p in points:
            key = f"{round(p['x'], 1)},{round(p['y'], 1)},{round(p['z'], 1)}"
            self.current_map[key] = p
    
    def save_current_as_last_saved(self) -> Dict:
        """
        Save the current map as the 'last saved' reference map.
        This OVERWRITES the previous last saved map.
        """
        try:
            self.last_saved_map = dict(self.current_map)  # Deep copy
            data = list(self.last_saved_map.values())
            with open(LAST_SAVED_MAP_FILE, "w") as f:
                json.dump(data, f)
            print(f"💾 Map saved! Current map ({len(self.current_map)} points) is now the last saved reference.")
            return {"status": "Saved as Last", "count": len(self.last_saved_map), "timestamp": str(datetime.now())}
        except Exception as e:
            return {"status": "Error", "detail": str(e)}
    
    def get_last_saved(self) -> Dict:
        """Retrieve the last saved map."""
        try:
            if os.path.exists(LAST_SAVED_MAP_FILE):
                with open(LAST_SAVED_MAP_FILE, "r") as f:
                    data = json.load(f)
                return {"status": "ok", "points": data, "count": len(data)}
            return {"status": "No last saved map found", "points": [], "count": 0}
        except Exception as e:
            return {"status": "Error", "detail": str(e)}
    
    def get_differences(self) -> Dict:
        """
        Compare current map with last saved map.
        Identifies new, removed, and modified points.
        """
        try:
            current_keys = set(self.current_map.keys())
            saved_keys = set(self.last_saved_map.keys())
            
            # New points (in current but not in saved) - show in RED
            new_points = [self.current_map[k] for k in (current_keys - saved_keys)]
            
            # Removed points (in saved but not in current) - show as deleted
            removed_points = [self.last_saved_map[k] for k in (saved_keys - current_keys)]
            
            # Modified points (both exist but with changed confidence)
            modified_points = []
            common_keys = current_keys & saved_keys
            for k in common_keys:
                if self.current_map[k]["confidence"] != self.last_saved_map[k]["confidence"]:
                    modified_points.append({
                        "point": self.current_map[k],
                        "old_confidence": self.last_saved_map[k]["confidence"],
                        "new_confidence": self.current_map[k]["confidence"]
                    })
            
            return {
                "status": "ok",
                "new_points": new_points,           # RED highlight
                "removed_points": removed_points,   # Show as deleted
                "modified_points": modified_points, # Show as modified
                "stats": {
                    "new_count": len(new_points),
                    "removed_count": len(removed_points),
                    "modified_count": len(modified_points),
                    "current_total": len(self.current_map),
                    "saved_total": len(self.last_saved_map)
                }
            }
        except Exception as e:
            return {"status": "Error", "detail": str(e)}
    
    def save_snapshot(self) -> Dict:
        """Save current map to snapshot (for backup purposes)."""
        try:
            data = list(self.current_map.values())
            with open(SNAPSHOT_FILE, "w") as f:
                json.dump(data, f)
            return {"status": "Saved", "count": len(data)}
        except Exception as e:
            return {"status": "Error", "detail": str(e)}
    
    def load_snapshot(self) -> Dict:
        """Load map from snapshot."""
        try:
            if os.path.exists(SNAPSHOT_FILE):
                with open(SNAPSHOT_FILE, "r") as f:
                    data = json.load(f)
                self.current_map.clear()
                for p in data:
                    key = f"{round(p['x'], 1)},{round(p['y'], 1)},{round(p['z'], 1)}"
                    self.current_map[key] = p
                self.map_version += 1
                return {"status": "Loaded", "count": len(self.current_map)}
            return {"status": "No snapshot found"}
        except Exception as e:
            return {"status": "Error", "detail": str(e)}
    
    def clear_all(self):
        """Clear both current and last saved maps."""
        self.current_map.clear()
        self.last_saved_map.clear()
        self.map_version += 1
        print("🗑️ All maps cleared")
    
    def get_current_map(self) -> List[Dict]:
        """Get all points in the current map."""
        return list(self.current_map.values())
    
    def get_map_by_version(self, version: int) -> Tuple[bool, List[Dict]]:
        """Check if version matches current, return false if not modified."""
        if version == self.map_version:
            return False, []  # Not modified
        return True, self.get_current_map()
