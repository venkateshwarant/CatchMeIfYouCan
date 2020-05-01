import sys

import numpy
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import random as rd

np.random.seed(19680801)
A_COUNT = 1
B_COUNT = 1
SIMULATION_TIME = 100
DIMENSION = 3
OPTIMISE_FOR_A = True
OPTIMISE_FOR_B = False

INITIALISE_A_RANDOMLY = False
INITIALISE_B_RANDOMLY = False

INITIAL_LOCATION_A = [0, 0, 0]
INITIAL_LOCATION_B = [50, 50, 50]

SPACE_CONSTRAINTS = (200, 200, 200)
SIMULATION_STOPPED = False

PREV_B_DISTANCE= 99999
class Simulation:
    A_COUNT = 1
    B_COUNT = 1
    SIMULATION_TIME = 5000

    def __init__(self, a, b, t):
        self.A_COUNT = a
        self.B_COUNT = b
        self.SIMULATION_TIME = t

    @staticmethod
    def gen_rand_line(length, dims=2):

        lineData = np.empty((dims, length))
        lineData[:, 0] = np.random.rand(dims)
        for index in range(1, length):
            step = ((np.random.rand(dims) - 0.5) * 0.1)
            lineData[:, index] = lineData[:, index - 1] + step

        return lineData

    def update_lines(self, num, dataLines, lines):
        for line, data in zip(lines, dataLines):
            line.set_data(data[0:2, :num])
            line.set_3d_properties(data[2, :num])
        return lines

    def simulate(self):
        UAV_A = A(40, 10, 3)
        UAV_B = B(40, 10, 3)
        A_PATH = []
        B_PATH = []
        distance = []

        for i in range(0, SIMULATION_TIME):
            print(str(i))
            if SIMULATION_STOPPED:
                break
            else:
                if len(A_PATH) == 0:
                    A_PATH.append(UAV_A.CURRENT_COORDINATES)
                if len(B_PATH) == 0:
                    B_PATH.append(UAV_B.CURRENT_COORDINATES)
                b=UAV_B.step()
                a= UAV_A.step()
                B_PATH.append(b)
                A_PATH.append(a)
                print(a)
                print(b)
                d= DistanceUtil.get_distance_between_two_points(a, b)
                print(d)
                distance.append(d)

        fig = plt.figure(1)
        ax = p3.Axes3D(fig)
        A_X= [i[0] for i in A_PATH]
        A_Y= [i[1] for i in A_PATH]
        A_Z= [i[2] for i in A_PATH]
        A_ARR= [numpy.asarray(A_X), numpy.asarray(A_Y), numpy.asarray(A_Z)]
        B_X= [i[0] for i in B_PATH]
        B_Y= [i[1] for i in B_PATH]
        B_Z= [i[2] for i in B_PATH]
        B_ARR= [numpy.asarray(B_X), numpy.asarray(B_Y), numpy.asarray(B_Z)]
        ALL_LINE_PATH=[numpy.asarray(A_ARR), numpy.asarray(B_ARR)]

        # 2 lines of random 3-D lines
        # data = [self.gen_rand_line(5000, 3) for index in range(3)]

        # Creating fifty line objects.
        # NOTE: Can't pass empty arrays into 3d version of plot()
        lines = [ax.plot(numpy.asarray(dat)[0, 0:1], numpy.asarray(dat)[1, 0:1], numpy.asarray(dat)[2, 0:1])[0] for dat in ALL_LINE_PATH]
        fig2= plt.figure(2)
        plt.plot(distance)
        loc_plot=[]
        lops_plot=[]
        los_plot=[]
        for i in range(0,len(A_PATH)):
            loc_plot.append(A.LINE_OF_CONTROL)
            lops_plot.append(A.LINE_OF_PARTIAL_SIGHT)
            los_plot.append(A.LINE_OF_SIGHT)
        plt.plot(loc_plot, color='red')
        plt.plot(los_plot, color='orange')
        plt.plot(lops_plot, color='yellow')

        # Setting the axes properties
        ax.set_xlim3d([0.0, SPACE_CONSTRAINTS[0]])
        ax.set_xlabel('X')

        ax.set_ylim3d([0.0, SPACE_CONSTRAINTS[1]])
        ax.set_ylabel('Y')

        ax.set_zlim3d([0.0, SPACE_CONSTRAINTS[2]])
        ax.set_zlabel('Z')

        ax.set_title('Catch me if you can')

        # Creating the Animation object
        line_ani = animation.FuncAnimation(fig, self.update_lines, 500, fargs=(numpy.asarray(ALL_LINE_PATH), lines),
                                           interval=500, blit=False)

        plt.show()


class DistanceUtil:
    @staticmethod
    def get_distance_between_two_points(coordinate1, coordinate2):
        x1 = coordinate1[0]
        y1 = coordinate1[1]
        z1 = coordinate1[2]
        x = coordinate2[0]
        y = coordinate2[1]
        z = coordinate2[2]
        d = ((x1 - x) ** 2 + (y1 - y) ** 2 + (z1 - z) ** 2) ** 0.5
        return d

    @staticmethod
    def get_a_point_towards_given_point2_from_given_point1_at_given_distance(coordinate1, coordinate2, d):
        D = DistanceUtil.get_distance_between_two_points(coordinate1, coordinate2)
        if d>D:
            return coordinate2
        while True:
            x = (1 - d / D) * coordinate1[0] + (d / D) * coordinate2[0]
            y = (1 - d / D) * coordinate1[1] + (d / D) * coordinate2[1]
            z = (1 - d / D) * coordinate1[2] + (d / D) * coordinate2[2]
            new_coordinate = [x, y, z]
            if new_coordinate[0] < SPACE_CONSTRAINTS[0] and new_coordinate[1] < SPACE_CONSTRAINTS[1] and new_coordinate[2] < SPACE_CONSTRAINTS[2]:
                break
            # elif new_coordinate[0] > SPACE_CONSTRAINTS[0] or new_coordinate[1] > SPACE_CONSTRAINTS[1] or new_coordinate[2] > SPACE_CONSTRAINTS[2]:
            #     x = (1 - d / D) * coordinate1[0] + (d / D) * coordinate2[0]
            #     y = (1 - d / D) * coordinate1[1] + (d / D) * coordinate2[1]
            #     z = (1 - d / D) * coordinate1[2] + (d / D) * coordinate2[2]
            #     new_coordinate = [x, y, z]
            #     break
        return new_coordinate


class A:
    UAV_A = None
    LINE_OF_SIGHT = 10
    LINE_OF_PARTIAL_SIGHT = 10
    LINE_OF_CONTROL = 10
    VELOCITY = 20

    def __init__(self, lops, los, loc):
        A.LINE_OF_SIGHT = los
        A.LINE_OF_PARTIAL_SIGHT = lops
        A.LINE_OF_CONTROL = loc
        self.PATH_DATA = np.empty((DIMENSION, SIMULATION_TIME))
        self.CURRENT_COORDINATES = []
        self.PATH_DATA = []
        A.UAV_A = self
        self.initialise_start_position()

    def initialise_start_position(self):
        if len(self.CURRENT_COORDINATES) == 0:
            if INITIALISE_A_RANDOMLY:
                x = rd.randint(0, SPACE_CONSTRAINTS[0])
                y = rd.randint(0, SPACE_CONSTRAINTS[1])
                z = rd.randint(0, SPACE_CONSTRAINTS[2])
                self.CURRENT_COORDINATES = [x, y, z]
            else:
                self.CURRENT_COORDINATES = INITIAL_LOCATION_A

    def get_current_coordinates(self):
        return self.CURRENT_COORDINATES

    def set_current_coordinates(self, coordinates):
        self.CURRENT_COORDINATES= coordinates

    @staticmethod
    def randomly_generate_next_point_according_to_configured_velocity(coordinate):
        while True:
            x = rd.uniform(0.001, SPACE_CONSTRAINTS[0])
            y = rd.uniform(0.001, SPACE_CONSTRAINTS[1])
            z = rd.uniform(0.001, SPACE_CONSTRAINTS[2])
            x1 = coordinate[0]
            y1 = coordinate[1]
            z1 = coordinate[2]
            normalised_x = x / (((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5)
            normalised_y = y / (((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5)
            normalised_z = z / (((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5)
            new_x = x1 + normalised_x * A.VELOCITY
            new_y = y1 + normalised_y * A.VELOCITY
            new_z = z1 + normalised_z * A.VELOCITY
            coordinates = [new_x, new_y, new_z]
            if new_x < SPACE_CONSTRAINTS[0] and new_y < SPACE_CONSTRAINTS[1] and new_z < SPACE_CONSTRAINTS[2]:
                break
            elif coordinates[0] > SPACE_CONSTRAINTS[0] or coordinates[1] > SPACE_CONSTRAINTS[1] or coordinates[2] > SPACE_CONSTRAINTS[2]:
                new_x = x1 - normalised_x * A.VELOCITY
                new_y = y1 - normalised_y * A.VELOCITY
                new_z = z1 - normalised_z * A.VELOCITY
                coordinates = [new_x, new_y, new_z]
                break
        return coordinates

    def step(self):
        global PREV_B_DISTANCE
        if B.is_distance_known(self.get_current_coordinates()):
            if B.is_location_known(self.get_current_coordinates()):
                if self.is_intruder_inside_line_of_control(B.get_location_if_known(self.get_current_coordinates())):
                    print("Intruder caught!!!")
                    global SIMULATION_STOPPED
                    SIMULATION_STOPPED = True
                else:
                    new_coordinate = DistanceUtil.get_a_point_towards_given_point2_from_given_point1_at_given_distance(
                        self.get_current_coordinates(), B.get_location_if_known(self.get_current_coordinates()), self.VELOCITY)
                    self.set_current_coordinates(new_coordinate)
                    return new_coordinate
            else:
                d = B.get_distance_if_known(self.get_current_coordinates())
                if not d>PREV_B_DISTANCE:
                    d1 = 0
                    new_coordinate = []
                    if d < B.LINE_OF_SIGHT and d < B.VELOCITY:
                        new_coordinate = B.get_location_if_known(self.get_current_coordinates())
                    else:
                        while True:
                            vel = B.VELOCITY
                            if d < B.VELOCITY:
                                vel = 0.90 * d
                            current_coord = self.get_current_coordinates()
                            x = rd.uniform(0, SPACE_CONSTRAINTS[0])
                            y = rd.uniform(0, SPACE_CONSTRAINTS[1])
                            z = rd.uniform(0, SPACE_CONSTRAINTS[2])
                            normalised_x = x / (((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5)
                            normalised_y = y / (((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5)
                            normalised_z = z / (((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5)
                            new_coordinate = [current_coord[0] + normalised_x * vel,
                                              current_coord[1] + normalised_y * vel,
                                              current_coord[2] + normalised_z * vel]
                            d1 = B.get_distance_if_known(new_coordinate)
                            if d1 is not None and d1 < d and new_coordinate[0] < SPACE_CONSTRAINTS[0] and \
                                    new_coordinate[
                                        1] < SPACE_CONSTRAINTS[1] and new_coordinate[2] < SPACE_CONSTRAINTS[2]:
                                self.set_current_coordinates(new_coordinate)
                                break
                            elif d1 is not None and d1 < d and (
                                    new_coordinate[0] > SPACE_CONSTRAINTS[0] or new_coordinate[
                                1] > SPACE_CONSTRAINTS[1] or new_coordinate[2] > SPACE_CONSTRAINTS[2]):
                                new_coordinate = [current_coord[0] - normalised_x * vel,
                                                  current_coord[1] - normalised_y * vel,
                                                  current_coord[2] - normalised_z * vel]
                                self.set_current_coordinates(new_coordinate)
                                break
                    if d1 < A.LINE_OF_CONTROL:
                        # global SIMULATION_STOPPED
                        SIMULATION_STOPPED = True
                        print("Intruder caught!!!")
                    PREV_B_DISTANCE = d1
                    return self.get_current_coordinates()
                else:
                    while True:
                        new_coordinate = A.randomly_generate_next_point_according_to_configured_velocity(
                            self.get_current_coordinates())
                        new_d = B.get_distance_if_known(new_coordinate)
                        if new_d is None:
                            new_d = 9999
                        if new_d<PREV_B_DISTANCE:
                            PREV_B_DISTANCE=new_d
                            if new_d < A.LINE_OF_CONTROL:
                                # global SIMULATION_STOPPED
                                SIMULATION_STOPPED = True
                                print("Intruder caught!!!")
                            break

                    self.set_current_coordinates(new_coordinate)
                    return new_coordinate

        else:
            new_coordinate = A.randomly_generate_next_point_according_to_configured_velocity(self.get_current_coordinates())
            self.set_current_coordinates(new_coordinate)
            return new_coordinate

    def is_intruder_inside_line_of_control(self, coordinates):
        d = DistanceUtil.get_distance_between_two_points(self.get_current_coordinates(), coordinates)
        if d <= A.LINE_OF_CONTROL:
            return True
        else:
            return False


class B:
    UAV_B = None
    LINE_OF_SIGHT = 10
    LINE_OF_PARTIAL_SIGHT = 10
    LINE_OF_CONTROL = 10
    VELOCITY = 20

    def __init__(self, lops, los, loc):
        B.LINE_OF_SIGHT = los
        B.LINE_OF_PARTIAL_SIGHT = lops
        B.LINE_OF_CONTROL = loc
        self.PATH_DATA = np.empty((DIMENSION, SIMULATION_TIME))
        self.CURRENT_COORDINATES = []
        self.PATH_DATA = []
        B.UAV_B = self
        self.initialise_start_position()

    def initialise_start_position(self):
        if len(self.CURRENT_COORDINATES) == 0:
            if INITIALISE_B_RANDOMLY:
                x = rd.uniform(0.001, SPACE_CONSTRAINTS[0])
                y = rd.uniform(0.001, SPACE_CONSTRAINTS[1])
                z = rd.uniform(0.001, SPACE_CONSTRAINTS[2])
                self.CURRENT_COORDINATES = [x, y, z]
            else:
                self.CURRENT_COORDINATES = INITIAL_LOCATION_B

    @staticmethod
    def randomly_generate_next_point_according_to_configured_velocity(coordinate):
        while True:
            x = rd.uniform(0.001, SPACE_CONSTRAINTS[0])
            y = rd.uniform(0.001, SPACE_CONSTRAINTS[1])
            z = rd.uniform(0.001, SPACE_CONSTRAINTS[2])
            x1 = coordinate[0]
            y1 = coordinate[1]
            z1 = coordinate[2]
            normalised_x = x / (((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5)
            normalised_y = y / (((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5)
            normalised_z = z / (((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5)
            new_x = x1 + normalised_x * B.VELOCITY
            new_y = y1 + normalised_y * B.VELOCITY
            new_z = z1 + normalised_z * B.VELOCITY
            coordinates = [new_x, new_y, new_z]
            if new_x < SPACE_CONSTRAINTS[0] and new_y < SPACE_CONSTRAINTS[1] and new_z < SPACE_CONSTRAINTS[2]:
                break
            elif coordinates[0] > SPACE_CONSTRAINTS[0] or coordinates[1] > SPACE_CONSTRAINTS[1] or coordinates[2] > SPACE_CONSTRAINTS[2]:
                new_x = x1 - normalised_x * A.VELOCITY
                new_y = y1 - normalised_y * A.VELOCITY
                new_z = z1 - normalised_z * A.VELOCITY
                coordinates = [new_x, new_y, new_z]
                break
        return coordinates

    def get_current_coordinates(self):
        return self.CURRENT_COORDINATES

    def set_current_coordinates(self, coordinates):
        self.CURRENT_COORDINATES = coordinates

    @staticmethod
    def is_distance_known(coordinates):
        d = DistanceUtil.get_distance_between_two_points(B.UAV_B.CURRENT_COORDINATES, coordinates)
        if d > B.LINE_OF_PARTIAL_SIGHT:
            return False
        else:
            return True

    @staticmethod
    def is_location_known(coordinates):
        d = DistanceUtil.get_distance_between_two_points(B.UAV_B.CURRENT_COORDINATES, coordinates)
        if d > B.LINE_OF_SIGHT:
            return False
        else:
            return True

    @staticmethod
    def get_distance_if_known(coordinates):
        d = DistanceUtil.get_distance_between_two_points(B.UAV_B.CURRENT_COORDINATES, coordinates)
        if d <= B.LINE_OF_PARTIAL_SIGHT:
            return d
        else:
            return None

    @staticmethod
    def get_location_if_known(coordinates):
        d = DistanceUtil.get_distance_between_two_points(B.UAV_B.CURRENT_COORDINATES, coordinates)
        if d <= B.LINE_OF_SIGHT:
            return B.UAV_B.CURRENT_COORDINATES
        else:
            return None

    def step(self):
        new_coordinate = B.randomly_generate_next_point_according_to_configured_velocity(self.get_current_coordinates())
        self.set_current_coordinates(new_coordinate)
        return new_coordinate


if __name__ == '__main__':
    simulator = Simulation(A_COUNT, B_COUNT, SIMULATION_TIME)
    simulator.simulate()
