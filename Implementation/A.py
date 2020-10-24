from enum import Enum

import numpy as np
from numpy import random

from GlobalConstants import GlobalConstants
from utils import Vector3D, collision_with_plane_for_defender


class DefenderRegionTypes(Enum):
    STOCHASTIC_SEARCH = "STOCHASTIC SEARCH"
    LOFV = "Line of Field View"
    LOS = "Line of Sight"
    LOC = "Line of control"


class A:
    def __init__(self, position, velocity, line_of_control, line_of_sight, line_of_field_view , random_motion, field_view_motion,  chase_motion):  # ,  los, lops, loc):
        self.position = position
        self.velocity = velocity
        self.line_of_control = line_of_control
        self.line_of_sight = line_of_sight
        self.line_of_field_view = line_of_field_view
        self.random_motion = random_motion
        self.chase_motion = chase_motion
        self.field_view_motion = field_view_motion

        # Objects Motion Params
        self.motion = None
        self.trajectory_type = None

    def reset_trajectory(self):
        # print("Trajectory Reset from {} to None".format(self.trajectory_type))
        self.motion = None
        self.trajectory_type = None

    def get_trajectory_type(self, intruder_position):
        # Get Distance between A and B
        d = np.linalg.norm(self.position.vector - intruder_position.vector)
        # Decide the trajectory type
        if d >= self.line_of_field_view:
            return DefenderRegionTypes.STOCHASTIC_SEARCH
        elif self.line_of_sight <= d < self.line_of_field_view:
            return DefenderRegionTypes.LOFV
        elif self.line_of_control <= d < self.line_of_sight:
            return DefenderRegionTypes.LOS
        else:
            return DefenderRegionTypes.LOC

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
                self.velocity.vector = collision_with_plane_for_defender(self.velocity.vector, np.array([1, 0, 0, 0]))
            else:
                # Collision with the plane x = WORLD.x
                self.velocity.vector = collision_with_plane_for_defender(self.velocity.vector,
                                                            np.array([1, 0, 0, -GlobalConstants.WORLD.x]))
        elif abs(diff_y) == max_diff:
            if diff_y < 0:
                # Collision with the plane y = 0
                self.velocity.vector = collision_with_plane_for_defender(self.velocity.vector, np.array([0, 1, 0, 0]))
            else:
                # Collision with the plane y = WORLD.y
                self.velocity.vector = collision_with_plane_for_defender(self.velocity.vector,
                                                            np.array([0, 1, 0, -GlobalConstants.WORLD.y]))
        else:
            if diff_z < 0:
                # Collision with the plane z = 0
                self.velocity.vector = collision_with_plane_for_defender(self.velocity.vector, np.array([0, 0, 1, 0]))
            else:
                # Collision with the plane z = WORLD.z
                self.velocity.vector = collision_with_plane_for_defender(self.velocity.vector,
                                                            np.array([0, 0, 1, -GlobalConstants.WORLD.z]))

        # print('After position and velocity correction: ', self)
        # print()

    def update_trajectory_type(self, intruder_position):
        # Check if trajectory type has changed
        new_trajectory_type = self.get_trajectory_type(intruder_position)
        if new_trajectory_type == self.trajectory_type:
            return

        # print('Trajectory changed from: {} to {}'.format(self.trajectory_type, new_trajectory_type))

        # Set the new trajectory type
        other_position = None
        if new_trajectory_type == DefenderRegionTypes.STOCHASTIC_SEARCH:
            # The defender doesn't know the intruders position in here
            self.motion = self.random_motion
        elif new_trajectory_type == DefenderRegionTypes.LOFV:
            # The defender now knows the position of intruder
            self.motion = self.field_view_motion
            other_position = intruder_position
        elif new_trajectory_type == DefenderRegionTypes.LOS:
            # The defender now knows the position of intruder
            self.motion = self.chase_motion
            other_position = intruder_position
        elif new_trajectory_type == DefenderRegionTypes.LOC:
            self.trajectory_type = new_trajectory_type
            return

        self.trajectory_type = new_trajectory_type
        self.motion.setup(position=self.position, velocity=self.velocity, other_position=other_position)

    def update_trajectory_velocity(self, intruder_position):
        """
        Updates velocity for the next cycle
        """
        # Get State of the defender
        other_position = None
        if self.trajectory_type == DefenderRegionTypes.LOC:
            return

        if self.trajectory_type != DefenderRegionTypes.STOCHASTIC_SEARCH:
            # The intruder nocollision_with_planew knows the position of intruder
            # print("Defender found")
            other_position = intruder_position
        else:
            other_position = Vector3D(random.randint(1, 1000), random.randint(1, 1000), random.randint(1, 1000))

        self.velocity = self.motion.update_velocity(position=self.position, velocity=self.velocity,
                                                    other_position=other_position)

    def __repr__(self):
        return "Defender(position:{}, velocity:{})".format(self.position, self.velocity)