from enum import Enum

import numpy as np

from GlobalConstants import GlobalConstants
from utils import collision_with_plane, Vector3D


class IntruderTrajectoryTypes(Enum):
    SAFE = "SAFE"
    CHASE = "CHASE"
    DANGER = "DANGER"


class B:
    def __init__(self, position, velocity, danger_zone, chase_zone, free_motion, chase_motion, danger_motion):
        self.position = position
        self.velocity = velocity
        self.danger_zone = danger_zone
        self.chase_zone = chase_zone
        self.free_motion = free_motion
        self.chase_motion = chase_motion
        self.danger_motion = danger_motion

        # Objects Motion Params
        self.motion = None
        self.trajectory_type = None

    def reset_trajectory(self):
        # print("Trajectory Reset from {} to None".format(self.trajectory_type))
        self.motion = None
        self.trajectory_type = None

    def get_trajectory_type(self, defender_position):
        # Get Distance between A and B
        d = np.linalg.norm(self.position.vector - defender_position.vector)
        # Decide the trajectory type
        if d >= self.chase_zone:
            return IntruderTrajectoryTypes.SAFE
        elif self.danger_zone <= d < self.chase_zone:
            return IntruderTrajectoryTypes.CHASE
        else:
            return IntruderTrajectoryTypes.DANGER

    def move(self):
        """
        Moves the object forwards as per the current velocity
        """
        self.position.vector = self.position.vector + GlobalConstants.TIME_DELTA * self.velocity.vector

    def collided(self):
        """
        Finding out whether collision occurred or not
        If it is just on the boundary, then no collision is said to have happened
        """

        if not (0 <= self.position.x <= GlobalConstants.WORLD.x):
            return True
        if not (0 <= self.position.y <= GlobalConstants.WORLD.y):
            return True
        if not (0 <= self.position.z <= GlobalConstants.WORLD.z):
            return True
        return False

    def handle_collision(self):
        """
        Adjusts the position and velocity of the object when collided with walls
        """

        # Finding out whether collision occurred or not
        diff_x = self.position.x if self.position.x < 0 else (
            self.position.x - GlobalConstants.WORLD.x if self.position.x > GlobalConstants.WORLD.x else 0)
        diff_y = self.position.y if self.position.y < 0 else (
            self.position.y - GlobalConstants.WORLD.y if self.position.y > GlobalConstants.WORLD.y else 0)
        diff_z = self.position.z if self.position.z < 0 else (
            self.position.z - GlobalConstants.WORLD.z if self.position.z > GlobalConstants.WORLD.z else 0)

        max_diff = max(abs(diff_x), abs(diff_y), abs(diff_z))
        if max_diff == 0:
            # No collison
            return

        # print("Collided at: ", self)
        # Correct the position, so that the object is not hanging outside of space
        self.position.x = self.position.x if diff_x == 0 else (0 if diff_x < 0 else GlobalConstants.WORLD.x)
        self.position.y = self.position.y if diff_y == 0 else (0 if diff_y < 0 else GlobalConstants.WORLD.y)
        self.position.z = self.position.z if diff_z == 0 else (0 if diff_z < 0 else GlobalConstants.WORLD.z)

        # Also find which plane the collision occurred with and the new velocity after collision
        if abs(diff_x) == max_diff:
            if diff_x < 0:
                # Collision with the plane x = 0
                self.velocity.vector = collision_with_plane(self.velocity.vector, np.array([1, 0, 0, 0]))
            else:
                # Collision with the plane x = WORLD.x
                self.velocity.vector = collision_with_plane(self.velocity.vector, np.array([1, 0, 0, -GlobalConstants.WORLD.x]))
        elif abs(diff_y) == max_diff:
            if diff_y < 0:
                # Collision with the plane y = 0
                self.velocity.vector = collision_with_plane(self.velocity.vector, np.array([0, 1, 0, 0]))
            else:
                # Collision with the plane y = WORLD.y
                self.velocity.vector = collision_with_plane(self.velocity.vector, np.array([0, 1, 0, -GlobalConstants.WORLD.y]))
        else:
            if diff_z < 0:
                # Collision with the plane z = 0
                self.velocity.vector = collision_with_plane(self.velocity.vector, np.array([0, 0, 1, 0]))
            else:
                # Collision with the plane z = WORLD.z
                self.velocity.vector = collision_with_plane(self.velocity.vector, np.array([0, 0, 1, -GlobalConstants.WORLD.z]))

        # print('After position and velocity correction: ', self)
        # print()

    def update_trajectory_type(self, defender_position):
        # Check if trajectory type has changed
        new_trajectory_type = self.get_trajectory_type(defender_position)
        if new_trajectory_type == self.trajectory_type:
            return

        # print('Trajectory changed from: {} to {}'.format(self.trajectory_type, new_trajectory_type))

        # Set the new trajectory type
        other_position = None
        if new_trajectory_type == IntruderTrajectoryTypes.SAFE:
            # The intruder doesn't know the defenders position in here
            self.motion = self.free_motion
        elif new_trajectory_type == IntruderTrajectoryTypes.CHASE:
            # The intruder now knows the position of defender
            self.motion = self.chase_motion
            other_position = defender_position
        elif new_trajectory_type == IntruderTrajectoryTypes.DANGER:
            # The intruder now knows the position of defender
            # print("DANGER")
            # print(self.position)
            # print(defender.position)
            self.motion = self.danger_motion
            other_position = defender_position

        self.trajectory_type = new_trajectory_type
        self.motion.setup(position=self.position, velocity=self.velocity, other_position=other_position)

    def update_trajectory_velocity(self, defender_position):
        """
        Updates velocity for the next cycle
        """
        # Get State of the defender
        other_position = None
        if self.trajectory_type != IntruderTrajectoryTypes.SAFE:
            # The intruder now knows the position of defender
            # print("Defender found")
            other_position = defender_position

        self.velocity = self.motion.update_velocity(position=self.position, velocity=self.velocity, other_position=other_position)

    def __repr__(self):
        return "Intruder(position:{}, velocity:{})".format(self.position, self.velocity)