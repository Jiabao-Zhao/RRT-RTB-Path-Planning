import numpy as np
from typing import List

class SphericalObstacle:
    """Create spherical obstacle in 3D space"""
    def __init__(self, center: np.ndarray, radius: float):
        self.center = np.asarray(center, dtype=float)
        self.radius = float(radius)

    def distance_to_point(self, point: np.ndarray) -> float:
        """Calculate distance from obstacle surface to a point (mm)"""
        return np.linalg.norm(point - self.center) - self.radius

class WorldModel:
    """World model containing obstacles and ground plane"""
    def __init__(self, ground_z: float):
        self.obstacles: List[SphericalObstacle] = []
        self.ground_z = ground_z

    def add_sphere(self, center: List[float], radius: float):
        """Add a spherical obstacle (center and radius in mm)"""

        obs = SphericalObstacle(np.array(center), radius)
        self.obstacles.append(obs)

    def clear_obstacles(self):
        """Remove all obstacles (keeps ground)"""
        self.obstacles.clear()
