import pybullet as p
import pybullet_data
import time
import math

def create_arena(size=4):
    """Create a square arena with walls."""
    halfExtents = [size/2, size/2, 0.2]

    # Create the ground plane
    ground_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents)
    ground_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=halfExtents, rgbaColor=[0.9, 0.9, 0.9, 1])
    ground = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=ground_shape, baseVisualShapeIndex=ground_visual, basePosition=[0, 0, -0.2])

    # Create walls
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

def apply_robot_motion(robot, linear_velocity, angular_velocity):
    """Apply linear and angular velocities to the robot."""
    # Get current position and orientation
    pos, orn = p.getBasePositionAndOrientation(robot)

    # Convert quaternion to Euler angles
    euler = p.getEulerFromQuaternion(orn)

    # Calculate new position based on linear velocity
    new_x = pos[0] + linear_velocity * math.cos(euler[2]) * (1/240)  # Assuming 240 Hz simulation
    new_y = pos[1] + linear_velocity * math.sin(euler[2]) * (1/240)

    # Calculate new orientation based on angular velocity
    new_yaw = euler[2] + angular_velocity * (1/240)

    # Set new position and orientation
    new_pos = [new_x, new_y, pos[2]]
    new_orn = p.getQuaternionFromEuler([euler[0], euler[1], new_yaw])
    p.resetBasePositionAndOrientation(robot, new_pos, new_orn)

    # Set velocities
    p.resetBaseVelocity(robot, linearVelocity=[linear_velocity * math.cos(euler[2]),
                                               linear_velocity * math.sin(euler[2]),
                                               0],
                        angularVelocity=[0, 0, angular_velocity])

def main():
    # Connect to the physics server
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Set up the simulation
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)  # Switch to non-real-time simulation for better control

    # Create the arena
    create_arena(size=4)

    # Create the robot
    robot = create_circular_robot()

    # Set up the camera
    p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0, 0, 0])

    # Add sliders for controlling velocities
    linear_velocity_slider = p.addUserDebugParameter("Linear Velocity", -1, 1, 0)
    angular_velocity_slider = p.addUserDebugParameter("Angular Velocity", -math.pi, math.pi, 0)

    while True:
        # Get velocities from sliders
        linear_velocity = p.readUserDebugParameter(linear_velocity_slider)
        angular_velocity = p.readUserDebugParameter(angular_velocity_slider)

        # Apply motion to the robot
        apply_robot_motion(robot, linear_velocity, angular_velocity)

        p.stepSimulation()
        time.sleep(1./240.)

if __name__ == "__main__":
    main()
