import random as rd
import numpy as np
import math

from numpy import random


def collision_with_plane(velocity_vector, plane):
    # print('With Plane: {}'.format(plane))

    # Get the vector perpendicular to the plane
    normal_vector = np.array([plane[0], plane[1], plane[2]])
    normal_vector_norm = normal_vector / np.linalg.norm(normal_vector)

    # Split the velocity in two components: 1 Parallel to normal and one perpendicular
    horizontal_projection_vector = normal_vector_norm * np.dot(velocity_vector, normal_vector_norm) / 1
    vertical_projection_vector = velocity_vector - horizontal_projection_vector

    # Collision flips the velocity projected along the normal
    horizontal_projection_vector = -1 * horizontal_projection_vector

    return horizontal_projection_vector + vertical_projection_vector

def collision_with_plane_for_defender(velocity_vector, plane):
    # print('With Plane: {}'.format(plane))

    # Get the vector perpendicular to the plane
    normal_vector = np.array([plane[0], plane[1], plane[2]])
    normal_vector_norm = normal_vector / np.linalg.norm(normal_vector)

    # Split the velocity in two components: 1 Parallel to normal and one perpendicular
    horizontal_projection_vector = normal_vector_norm * np.dot(velocity_vector, normal_vector_norm) / 1
    vertical_projection_vector = velocity_vector - horizontal_projection_vector

    # Collision flips the velocity projected along the normal
    horizontal_projection_vector = -1 * horizontal_projection_vector

    return horizontal_projection_vector + vertical_projection_vector

def get_random_point_on_plane(plane, constraints):
    """
    Returns a random point on the plane (obeying the constraints)
    plane = np.array() of size 4
    constraints = np.array() of size 3
    """
    # Get a random point in the plane
    knowns = dict()
    unknowns = {0, 1, 2}
    # Handle 0 coefficients in plane
    for i in range(3):
        if plane[i] == 0:
            knowns[i] = rd.uniform(0, constraints[i])
            unknowns.remove(i)
    # Set random values for all but one
    for i in range(3):
        if len(unknowns) <= 1:
            break
        if i not in knowns:
            knowns[i] = rd.uniform(0, constraints[i])
            unknowns.remove(i)

    last_unknown = list(unknowns)[0]
    val = 0
    for i in range(3):
        if i == last_unknown:
            continue
        val += (-1*plane[i]*knowns[i])
    val += (-1*plane[3])
    val /= plane[last_unknown]
    knowns[last_unknown] = val
    unknowns.remove(last_unknown)

    # print(plane)
    # print(np.array([knowns[0], knowns[1], knowns[2]]))
    # print()

    return np.array([knowns[0], knowns[1], knowns[2]])


def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


class Vector3D:
    """
     Makes use of numpy internally, so all numpy operations can be performed on it.
     The value for first 3 Dimensions can by accessed by x, y, z respectively.
    """
    def __init__(self, x=None, y=None, z=None, np_array=None):
        if np_array is not None:
            if np_array.size != 3:
                raise Exception("The np array passed must have size 3")
            else:
                self._vector = np.array([np_array[0], np_array[1], np_array[2]])
        else:
            if x is None or y is None or z is None:
                raise Exception("All 3 (x, y, z) values must be provided")
            else:
                self._vector = np.array([x, y, z])

    def __getattribute__(self, attr):
        if attr == 'x':
            return self._vector[0]
        elif attr == 'y':
            return self._vector[1]
        elif attr == 'z':
            return self._vector[2]
        elif attr == 'magnitude':
            return np.linalg.norm(self._vector)
        elif attr == 'vector':
            return np.copy(self._vector)
        else:
            return object.__getattribute__(self, attr)

    def __setattr__(self, attr, value):
        if attr == 'x':
            self.vector[0] = value
        elif attr == 'y':
            self.vector[1] = value
        elif attr == 'z':
            self.vector[2] = value
        elif attr == 'vector':
            self.__init__(np_array=value)
        else:
            object.__setattr__(self, attr, value)

    def __repr__(self):
        return "Vector3D(x:{}, y:{}, z:{})".format(self.x, self.y, self.z)


class UniformMotion:
    """
    Objects moved in the same direction as before but with a specified velocity magnitude
    """

    def __init__(self, velocity_magnitude, time_delta, constraints):
        self.velocity_magnitude = velocity_magnitude
        self.time_delta = time_delta
        self.constraints = constraints

    def setup(self, position, velocity, other_position=None, other_velocity=None):
        pass

    def update_velocity(self, position, velocity, other_position=None, other_velocity=None):
        return Vector3D(np_array=(velocity.vector / velocity.magnitude) * self.velocity_magnitude)


class RandomMotion:
    """
    Objects moved in the same direction as before but with a specified velocity magnitude
    """

    def __init__(self, velocity_magnitude, time_delta, constraints):
        self.velocity_magnitude = velocity_magnitude
        self.time_delta = time_delta
        self.constraints = constraints

    def setup(self, position, velocity, other_position=None, other_velocity=None):
        pass

    def update_velocity(self, position, velocity, other_position=None, other_velocity=None):
        random_permutation= np.random.permutation([velocity.x, velocity.y, velocity.z])
        return Vector3D(random_permutation[0], random_permutation[1], random_permutation[2])


class RandomMotionInsideCone:
    """
    Refer https://math.stackexchange.com/questions/56784/generate-a-random-direction-within-a-cone for explanation
    min_theta = min angle between new acceleration and current velocity vector (in degrees)
    max_theta = max angle between new acceleration and current velocity vector (in degrees)
    Can't go beyond a maximum velocity

    """
    def __init__(self, acceleration_magnitude, max_velocity, min_theta,  max_theta, time_delta, constraints):
        self.acceleration_magnitude = acceleration_magnitude
        self.max_velocity = max_velocity
        self.max_theta = max_theta
        self.min_theta = min_theta
        self.time_delta = time_delta
        self.constraints = constraints

    def setup(self, position, velocity, other_position=None, other_velocity=None):
        pass

    def update_velocity(self, position, velocity, other_position=None, other_velocity=None):
        # Get a random theta
        theta = rd.uniform(self.min_theta, self.max_theta)

        # Get a equation of the plane perpendicular to the velocity and passing through the current location
        plane = np.array([velocity.x, velocity.y, velocity.z,
                          -1 * np.dot(position.vector, velocity.vector)])
        # Get a random point in the plane
        plane_point = Vector3D(np_array=get_random_point_on_plane(plane, self.constraints.vector))
        # Get the Random line vector
        rand_line_vector = plane_point.vector - position.vector
        rand_line_vector_norm = rand_line_vector / np.linalg.norm(rand_line_vector)

        radius_magnitude = velocity.magnitude * np.tan(np.deg2rad(theta))
        # Get Radius vector
        radius = Vector3D(np_array=radius_magnitude * rand_line_vector_norm)

        acceleration_vector_norm = (velocity.vector + radius.vector) / np.linalg.norm(velocity.vector + radius.vector)
        acceleration_vector = acceleration_vector_norm * self.acceleration_magnitude

        new_velocity = Vector3D(np_array=velocity.vector + acceleration_vector * self.time_delta)
        if new_velocity.magnitude > self.max_velocity:
            new_velocity = Vector3D(np_array=(new_velocity.vector / new_velocity.magnitude) * self.max_velocity)

        return new_velocity


class FieldViewMotion:
    """
    Objects moved in the same direction as before but with a specified velocity magnitude
    """

    def __init__(self, velocity_magnitude, time_delta, constraints, field_view_angle=20):
        self.velocity_magnitude = velocity_magnitude
        self.time_delta = time_delta
        self.constraints = constraints
        self.field_view_angle = field_view_angle

    def setup(self, position, velocity, other_position=None, other_velocity=None):
        pass

    def update_velocity(self, position, velocity, other_position=None, other_velocity=None):
        z = random.randint(-100,100)/100
        d = np.linalg.norm(velocity.vector)
        x = (d**2 - z**2)**0.5 * math.cos(math.radians(self.field_view_angle))
        y = (d**2 - z**2)**0.5 * math.sin(math.radians(self.field_view_angle))
        return Vector3D(x, y, z)

class HelixMotion:
    """
    Enables helical motion
    """
    def __init__(self, radius_magnitude, velocity_magnitude, time_delta, constraints):
        self.radius_magnitude = radius_magnitude
        self.velocity_magnitude = velocity_magnitude
        self.time_delta = time_delta
        self.constraints = constraints

        self.angular_velocity_vector = None
        self.horizontal_proj_vector = None
        self.vertical_proj_vector = None
        self.delta_theta = None

    def setup(self, position, velocity, other_position=None, other_velocity=None):
        # Change the velocity magnitude to the configured one
        velocity.vector = velocity.vector / velocity.magnitude * self.velocity_magnitude

        # Get a equation of the plane perpendicular to the velocity and passing through the current location
        plane = np.array([velocity.x, velocity.y, velocity.z,
                          -1 * np.dot(position.vector, velocity.vector)])
        # Get a random point in the plane
        plane_point = Vector3D(np_array=get_random_point_on_plane(plane, self.constraints.vector))

        # Get the Random line vector
        rand_line_vector = position.vector - plane_point.vector
        rand_line_vector_norm = rand_line_vector / np.linalg.norm(rand_line_vector)
        # Get Radius vector
        radius = Vector3D(np_array=self.radius_magnitude * rand_line_vector_norm)

        # Get the plane perpendicular to radius and passing through current position
        plane = np.array([radius.x, radius.y, radius.z, -1 * np.dot(position.vector, radius.vector)])
        # Get a random point in the plane
        plane_point = Vector3D(np_array=get_random_point_on_plane(plane, self.constraints.vector))
        # Get the Random line vector
        rand_line_vector = plane_point.vector - position.vector
        rand_line_vector_norm = rand_line_vector / np.linalg.norm(rand_line_vector)

        # The Projection of velocity on the random line vector
        horizontal_proj_vector = rand_line_vector_norm * np.dot(velocity.vector, rand_line_vector_norm) / 1
        vertical_proj_vector = velocity.vector - horizontal_proj_vector

        # Get angular velocity
        angular_velocity_vector = np.cross(radius.vector, horizontal_proj_vector) / radius.magnitude**2
        # Get angular displacement
        delta_theta = np.linalg.norm(angular_velocity_vector) * self.time_delta

        # Properties
        self.angular_velocity_vector = angular_velocity_vector
        self.horizontal_proj_vector = horizontal_proj_vector
        self.vertical_proj_vector = vertical_proj_vector
        self.delta_theta = delta_theta

    def update_velocity(self, position, velocity, other_position=None, other_velocity=None):
        # Rotate horizontal velocity along vertical velocity
        new_horizontal_proj_vector = np.dot(self.horizontal_proj_vector,
                                            rotation_matrix(self.vertical_proj_vector, self.delta_theta))
        self.horizontal_proj_vector = new_horizontal_proj_vector

        return Vector3D(np_array=self.horizontal_proj_vector + self.vertical_proj_vector)


class ConstantAccelerationAway:
    """
    Accelerates at a constant rate away from the other object current location. Can't go beyond a max velocity
    """
    def __init__(self, acceleration_magnitude, max_velocity, time_delta, constraints):
        self.acceleration_magnitude = acceleration_magnitude
        self.max_velocity = max_velocity
        self.time_delta = time_delta
        self.constraints = constraints

    def setup(self, position, velocity, other_position=None):
        return

    def update_velocity(self, position, velocity, other_position=None):
        # Accelerate in the direction away from the other object
        acceleration_vector = position.vector - other_position.vector
        acceleration_vector_norm = acceleration_vector / np.linalg.norm(acceleration_vector)
        acceleration = Vector3D(np_array=acceleration_vector_norm * self.acceleration_magnitude)

        new_velocity = Vector3D(np_array=velocity.vector + acceleration.vector * self.time_delta)
        if new_velocity.magnitude > self.max_velocity:
            new_velocity = Vector3D(np_array=(new_velocity.vector/new_velocity.magnitude) * self.max_velocity)

        return new_velocity


class ConstantAccelerationTo(ConstantAccelerationAway):
    """
       Accelerates at a constant rate away from the other object current location. Can't go beyond a max velocity
    """

    def __init__(self, acceleration_magnitude, max_velocity, time_delta, constraints):
        super(ConstantAccelerationTo, self).__init__(
            acceleration_magnitude=acceleration_magnitude, max_velocity=max_velocity, time_delta=time_delta,
            constraints=constraints
        )

    def setup(self, position, velocity, other_position=None):
        super(ConstantAccelerationTo, self).setup(position=other_position, velocity=velocity, other_position=position)

    def update_velocity(self, position, velocity, other_position=None):
      return super(ConstantAccelerationTo, self).update_velocity(position=other_position, velocity=velocity, other_position=position)


class ConstantAccelerationAwayPredicted:
    """
    Accelerates at a constant rate away from the other object predicted location. Can't go beyond a max velocity
    """
    def __init__(self, acceleration_magnitude, max_velocity, time_delta, constraints):
        self.acceleration_magnitude = acceleration_magnitude
        self.max_velocity = max_velocity
        self.time_delta = time_delta
        self.constraints = constraints
        self.other_position = None

    def setup(self, position, velocity, other_position=None):
        self.other_position = other_position

    def update_velocity(self, position, velocity, other_position=None):
        """
        Accelerates at a constant rate away from the other object predicted location.
        """
        if self.other_position is None:
            # Need atleast two old positions of defender to predict its velocity
            self.other_position = other_position
            return velocity

        # Get the velocity of the the object from his last two known coordinates
        other_velocity_vector = (other_position.vector - self.other_position.vector) / self.time_delta
        # Get the distance between the objects
        separation_vector = position.vector - other_position.vector
        acceleration_vector_norm = (separation_vector - other_velocity_vector * self.time_delta) / \
                                   np.linalg.norm(separation_vector - other_velocity_vector * self.time_delta)
        acceleration_vector = acceleration_vector_norm * self.acceleration_magnitude
        new_velocity = Vector3D(np_array=velocity.vector + acceleration_vector * self.time_delta)
        if new_velocity.magnitude > self.max_velocity:
            new_velocity = Vector3D(np_array=(new_velocity.vector/new_velocity.magnitude) * self.max_velocity)

        self.other_position = other_position  # Update the last known other position
        return new_velocity


class ConstantAccelerationPerpendicular:
    """
       Accelerates at a constant rate in a direction perpendicular to the other object's current location. Can't go beyond a max velocity
    """

    def __init__(self, acceleration_magnitude, max_velocity, time_delta, constraints):
        self.acceleration_magnitude = acceleration_magnitude
        self.max_velocity = max_velocity
        self.time_delta = time_delta
        self.constraints = constraints

    def setup(self, position, velocity, other_position=None):
        return

    def update_velocity(self, position, velocity, other_position=None):
        # Accelerate in the direction away from the other object
        separation_vector = position.vector - other_position.vector
        separation_vector_norm = separation_vector / np.linalg.norm(separation_vector)

        # The Projection of velocity on the random line vector
        horizontal_proj_vector = separation_vector_norm * np.dot(velocity.vector, separation_vector_norm) / 1
        vertical_proj_vector = velocity.vector - horizontal_proj_vector

        acceleration_vector_norm = None
        if np.linalg.norm(vertical_proj_vector) == 0:
            # Get the plane perpendicular to separation vector and passing through current position
            plane = np.array([separation_vector_norm[0], separation_vector_norm[1], separation_vector_norm[2], -1 * np.dot(position.vector, separation_vector_norm)])
            # Get a random point in the plane
            plane_point = Vector3D(np_array=get_random_point_on_plane(plane, self.constraints.vector))
            # Get the Random line vector
            acceleration_vector_norm = (plane_point.vector - position.vector) / np.linalg.norm(plane_point.vector - position.vector)
        else:
            acceleration_vector_norm = vertical_proj_vector / np.linalg.norm(vertical_proj_vector)
        acceleration_vector = acceleration_vector_norm * self.acceleration_magnitude

        new_velocity = Vector3D(np_array=velocity.vector + acceleration_vector * self.time_delta)
        if new_velocity.magnitude > self.max_velocity:
            new_velocity = Vector3D(np_array=(new_velocity.vector / new_velocity.magnitude) * self.max_velocity)

        return new_velocity


class DistanceUtil:
    @staticmethod
    def get_a_point_towards_given_point2_from_given_point1_at_given_distance(position1, position2, d, constraints):
        D = np.linalg.norm(position1.vector, position2.vector)
        if d > D:
            return position2
        while True:
            x = (1 - d / D) * position1.x + (d / D) * position2.x
            y = (1 - d / D) * position1.y + (d / D) * position2.y
            z = (1 - d / D) * position1.z + (d / D) * position2.z
            new_coordinate = [x, y, z]
            if new_coordinate[0] < constraints.x and new_coordinate[1] < constraints.y and new_coordinate.z < constraints.z:
                break
            # elif new_coordinate[0] > SPACE_CONSTRAINTS[0] or new_coordinate[1] > SPACE_CONSTRAINTS[1] or new_coordinate[2] > SPACE_CONSTRAINTS[2]:
            #     x = (1 - d / D) * coordinate1[0] + (d / D) * coordinate2[0]
            #     y = (1 - d / D) * coordinate1[1] + (d / D) * coordinate2[1]
            #     z = (1 - d / D) * coordinate1[2] + (d / D) * coordinate2[2]
            #     new_coordinate = [x, y, z]
            #     break
        return Vector3D(x=new_coordinate[0], y=new_coordinate[1], z=new_coordinate[2])
