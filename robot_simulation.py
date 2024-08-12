import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from collections import deque

class LaserScan2D:
    def __init__(self, parent_body, range_max=10, resolution=60):  # Reduced resolution
        self.parent_body = parent_body
        self.range_max = range_max
        self.resolution = resolution
        self.angle_step = 2 * math.pi / resolution
        self.recent_scans = deque(maxlen=5)
        self.ray_from = []
        self.ray_to = []
        self._precompute_rays()

    def _precompute_rays(self):
        for i in range(self.resolution):
            angle = i * self.angle_step
            self.ray_from.append([0, 0, 0])
            self.ray_to.append([
                self.range_max * math.cos(angle),
                self.range_max * math.sin(angle),
                0
            ])

    def scan(self):
        pos, orn = p.getBasePositionAndOrientation(self.parent_body)
        ray_from = [list(map(lambda x, y: x + y, pos, ray)) for ray in self.ray_from]
        ray_to = [list(map(lambda x, y: x + y, pos, ray)) for ray in self.ray_to]

        results = p.rayTestBatch(ray_from, ray_to)

        scan_data = [result[2] * self.range_max for result in results]
        self.recent_scans.append((pos, p.getEulerFromQuaternion(orn)[2], scan_data))
        return scan_data

    def get_recent_scans(self):
        return list(self.recent_scans)

def create_arena(size=4):
    """Create a square arena with walls."""
    halfExtents = [size/2, size/2, 0.2]

    ground_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents)
    ground_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=halfExtents, rgbaColor=[0.9, 0.9, 0.9, 1])
    ground = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=ground_shape, baseVisualShapeIndex=ground_visual, basePosition=[0, 0, -0.2])

    wall_height = 0.5
    wall_thickness = 0.1

    wall_positions = [
        [size/2, 0, wall_height/2],
        [-size/2, 0, wall_height/2],
        [0, size/2, wall_height/2],
        [0, -size/2, wall_height/2]
    ]

    wall_orientations = [
        p.getQuaternionFromEuler([0, 0, math.pi/2]),
        p.getQuaternionFromEuler([0, 0, math.pi/2]),
        p.getQuaternionFromEuler([0, 0, 0]),
        p.getQuaternionFromEuler([0, 0, 0])
    ]

    for pos, orn in zip(wall_positions, wall_orientations):
        wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size/2, wall_thickness/2, wall_height/2])
        wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.8, 0.8, 0.8, 1])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_shape, baseVisualShapeIndex=wall_visual, basePosition=pos, baseOrientation=orn)

def create_circular_robot(radius=0.2, height=0.1):
    """Create a circular robot."""
    robot_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    robot_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=[1, 0, 0, 1])
    robot = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=robot_shape, baseVisualShapeIndex=robot_visual, basePosition=[0, 0, height/2])
    return robot

def move_robot(robot, linear_velocity, angular_velocity):
    """Move the robot based on linear and angular velocity."""
    pos, orn = p.getBasePositionAndOrientation(robot)
    euler = p.getEulerFromQuaternion(orn)

    dt = 1/240  # Assuming 240 Hz simulation

    # Update position
    new_x = pos[0] + linear_velocity * math.cos(euler[2]) * dt
    new_y = pos[1] + linear_velocity * math.sin(euler[2]) * dt

    # Update orientation
    new_yaw = euler[2] + angular_velocity * dt
    new_orn = p.getQuaternionFromEuler([0, 0, new_yaw])

    # Set new position and orientation
    p.resetBasePositionAndOrientation(robot, [new_x, new_y, pos[2]], new_orn)

def visualize_laser_scan_data(laser_scanner):
    scans = laser_scanner.get_recent_scans()
    for scan_index, (robot_pos, robot_yaw, scan_data) in enumerate(scans):
        color = [(5 - scan_index) / 5, 0, scan_index / 5]  # Color gradient from red to blue
        for i, distance in enumerate(scan_data):
            if distance < 10:  # Only draw if there's a hit within range
                angle = i * laser_scanner.angle_step + robot_yaw
                end_pos = [
                    robot_pos[0] + distance * math.cos(angle),
                    robot_pos[1] + distance * math.sin(angle),
                    robot_pos[2]
                ]
                p.addUserDebugLine(robot_pos, end_pos, color, lifeTime=0.1)

def main():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    create_arena(size=4)
    robot = create_circular_robot()
    laser_scanner = LaserScan2D(robot, range_max=10, resolution=60)  # Reduced resolution

    p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0, 0, 0])

    linear_velocity_slider = p.addUserDebugParameter("Linear Velocity", -1, 1, 0)
    angular_velocity_slider = p.addUserDebugParameter("Angular Velocity", -math.pi, math.pi, 0)

    trail = []

    while True:
        start_time = time.time()

        linear_velocity = p.readUserDebugParameter(linear_velocity_slider)
        angular_velocity = p.readUserDebugParameter(angular_velocity_slider)

        move_robot(robot, linear_velocity, angular_velocity)

        # Update and visualize trail
        pos, _ = p.getBasePositionAndOrientation(robot)
        trail.append(pos)
        if len(trail) > 100:  # Keep only last 100 positions
            trail.pop(0)
        for i in range(1, len(trail)):
            p.addUserDebugLine(trail[i-1], trail[i], [0, 1, 0], lifeTime=0.1)

        start_laser_time = time.time()
        laser_scanner.scan()
        laser_scan_time = time.time()-start_laser_time
        finish_scan_time = time.time()
        print(f"laser scan time: {laser_scan_time}")
        visualize_laser_scan_data(laser_scanner)
        laser_viz_time = time.time()-finish_scan_time
        print(f"laser viz time: {laser_viz_time}")

        p.stepSimulation()

        # Limit the frame rate to 60 FPS
        elapsed = time.time() - start_time
        if elapsed < 1/60:
            time.sleep(1/60 - elapsed)

if __name__ == "__main__":
    main()
