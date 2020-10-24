import math
from enum import Enum

import numpy as np
import random as rd

import numpy as np
import matplotlib.pyplot as plt
from numpy import random
from scipy import stats

from A import A, DefenderRegionTypes
from B import B, IntruderTrajectoryTypes
from GlobalConstants import *
from utils import Vector3D, UniformMotion, HelixMotion, ConstantAccelerationAway, \
    ConstantAccelerationTo, ConstantAccelerationPerpendicular, RandomMotion, FieldViewMotion, RandomMotionInsideCone

rd.seed(99999999999)
SIMULATION_LOOPS = round(GlobalConstants.MAX_SIMULATION_TIME / GlobalConstants.TIME_DELTA)
distance = list()
sample_size = 100


class IntruderAlgorithms(Enum):
    ONE = "1"
    TWO = "2"





def get_motion_objects(initial_velocity, algorithm: IntruderAlgorithms):
    f_motion =HelixMotion(radius_magnitude=5, velocity_magnitude=initial_velocity.magnitude, time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD)
    if algorithm == IntruderAlgorithms.ONE:
        f_motion = HelixMotion(radius_magnitude=5, velocity_magnitude=initial_velocity.magnitude, time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD)
    elif algorithm == IntruderAlgorithms.TWO:
        f_motion = RandomMotionInsideCone(acceleration_magnitude=1.5, max_velocity=10, min_theta=75, max_theta=90,
                                         time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD)
    return f_motion


def runStatisticalTest(initial_a, initial_b, algorithm: IntruderAlgorithms):
    catching_time_list = list()
    for i in range(100):
        initial_velocity = Vector3D(1, 2, 3)
        f_motion = get_motion_objects(initial_velocity, algorithm)
        intruder = B(
            position=initial_b,
            velocity=initial_velocity,
            danger_zone=20,
            chase_zone=80,
            free_motion=f_motion,
            chase_motion=ConstantAccelerationAway(acceleration_magnitude=1.5, max_velocity=10,
                                                  time_delta=GlobalConstants.TIME_DELTA,
                                                  constraints=GlobalConstants.WORLD),
            danger_motion=ConstantAccelerationPerpendicular(acceleration_magnitude=2, max_velocity=15,
                                                            time_delta=GlobalConstants.TIME_DELTA,
                                                            constraints=GlobalConstants.WORLD),
        )

        defender = A(
            position=Vector3D(initial_a.x, initial_a.y, initial_a.z),
            velocity=initial_velocity,
            line_of_control=GlobalConstants.LOC,
            line_of_sight=GlobalConstants.LOS,
            line_of_field_view=GlobalConstants.LOFV,
            random_motion=UniformMotion(velocity_magnitude=initial_velocity.magnitude,
                                        time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD),
            chase_motion=ConstantAccelerationTo(acceleration_magnitude=2, max_velocity=15,
                                                time_delta=GlobalConstants.TIME_DELTA,
                                                constraints=GlobalConstants.WORLD),
            field_view_motion=FieldViewMotion(velocity_magnitude=initial_velocity.magnitude,
                                              time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD),
        )

        x_data_intruder, y_data_intruder, z_data_intruder = list(), list(), list()
        x_data_defender, y_data_defender, z_data_defender = list(), list(), list()
        intruder_trajectory_types = list()

        defender.update_trajectory_type(intruder.position)
        intruder.update_trajectory_type(defender.position)
        i = 0
        time_list = list()
        t0 = 0
        simulation_end_time = 0
        for i in range(SIMULATION_LOOPS):
            time_list.append(t0)
            distance.append(np.linalg.norm(defender.position.vector - intruder.position.vector))

            if defender.trajectory_type == DefenderRegionTypes.LOC:
                simulation_end_time = round(i * GlobalConstants.TIME_DELTA + 1)
                break

            x_data_defender.append(defender.position.x)
            y_data_defender.append(defender.position.y)
            z_data_defender.append(defender.position.z)

            x_data_intruder.append(intruder.position.x)
            y_data_intruder.append(intruder.position.y)
            z_data_intruder.append(intruder.position.z)

            intruder_trajectory_types.append(intruder.trajectory_type)

            # They see the last loop position
            intruder_old_position = Vector3D(np_array=intruder.position.vector)
            defender_old_position = Vector3D(np_array=defender.position.vector)

            # Update states for intruder
            intruder.move()
            if intruder.collided():
                intruder.handle_collision()
                intruder.reset_trajectory()
                intruder.update_trajectory_type(defender_old_position)
            else:
                intruder.update_trajectory_type(defender_old_position)
                intruder.update_trajectory_velocity(defender_old_position)

            # Update states for defender
            defender.move()
            if defender.collided():
                defender.handle_collision()
                defender.reset_trajectory()
                defender.update_trajectory_type(intruder_old_position)
            else:
                defender.update_trajectory_type(intruder_old_position)
                defender.update_trajectory_velocity(intruder_old_position)
            t0 = t0 + GlobalConstants.TIME_DELTA
        catching_time_list.append(simulation_end_time)
    return catching_time_list


def correctPValue(p):
    if math.isnan(p[0]):
        p=(999999999999, p[1])
    if math.isnan(p[1]):
        p=(p[0], 999999999999)
    return p

def chooseTheBestAlgorithm(catching_time1, catching_time2):
    catching_time_mean_1 = np.mean(catching_time1)
    catching_time_std_1 = np.std(catching_time1)
    p99_1 = stats.norm.interval(0.99, loc=catching_time_mean_1, scale=catching_time_std_1 / sample_size)
    p99_1 = correctPValue(p99_1)

    catching_time_mean_2 = np.mean(catching_time2)
    catching_time_std_2 = np.std(catching_time2)
    p99_2 = stats.norm.interval(0.99, loc=catching_time_mean_2, scale=catching_time_std_2 / sample_size)
    p99_2 = correctPValue(p99_2)


    if p99_1[1] <= p99_2[1] :
        return 1
    elif p99_2[1] <= p99_1[1]:
        return 2


def testForSingleInitialLocation():
    catching_time_list_1 = runStatisticalTest(Vector3D(0, 0, 0), Vector3D(60, 60, 60), IntruderAlgorithms.ONE)
    catching_time_list_2 = runStatisticalTest(Vector3D(0, 0, 0), Vector3D(60, 60, 60), IntruderAlgorithms.TWO)


    catching_time_list = [catching_time_list_1, catching_time_list_2]
    for i in range(0, 2):
        catching_time = np.array(catching_time_list[i])
        catching_time.sort()
        catching_time_mean = np.mean(catching_time)
        catching_time_std = np.std(catching_time)
        catching_time_var = np.var(catching_time)
        pdf = stats.norm.pdf(catching_time, catching_time_mean, catching_time_std)
        p99 = stats.norm.interval(0.99, loc=catching_time_mean, scale=catching_time_std / sample_size)
        print("Algorithm " + str(i + 1))
        print("Standard deviation     : " + str(catching_time_std))
        print("Mean catching time     : " + str(catching_time_mean))
        print("Variance catching time : " + str(catching_time_var))
        print("p99 value              : " + str(p99))
        print("")
        if i == 0:
            col = "orange"
        elif i == 1:
            col = "red"
        elif i == 2:
            col = "yellow"
        else:
            col = "green"
        plt.plot(catching_time, pdf, color=col)
    plt.xlabel('time (s)')
    plt.show()


def generateInitialPosition():
    while True:
        initial_a = Vector3D(random.randint(0, GlobalConstants.MAX_X), random.randint(0, GlobalConstants.MAX_Y),
                             random.randint(0, GlobalConstants.MAX_Z))
        initial_b = Vector3D(random.randint(0, GlobalConstants.MAX_X), random.randint(0, GlobalConstants.MAX_Y),
                             random.randint(0, GlobalConstants.MAX_Z))

        D = np.linalg.norm(initial_a.vector - initial_b.vector)
        if D > GlobalConstants.LOC:
            return initial_a, initial_b


def testForNRandomInitialLocation():
    bestAlgorithmCount = [0, 0]
    for i in range(100):
        initial_a, initial_b = generateInitialPosition()
        catching_time_list_1 = runStatisticalTest(Vector3D(initial_a.x, initial_a.y, initial_a.z),
                                                  Vector3D(initial_b.x, initial_b.y, initial_b.z),
                                                  IntruderAlgorithms.ONE)
        catching_time_list_2 = runStatisticalTest(Vector3D(initial_a.x, initial_a.y, initial_a.z),
                                                  Vector3D(initial_b.x, initial_b.y, initial_b.z),
                                                  IntruderAlgorithms.TWO)


        catching_time_1 = np.array(catching_time_list_1)
        catching_time_2 = np.array(catching_time_list_2)


        bestAlgorithm = chooseTheBestAlgorithm(catching_time_1, catching_time_2)
        bestAlgorithmCount[bestAlgorithm - 1] = bestAlgorithmCount[bestAlgorithm - 1] + 1

    objects = ('B1', 'B2')
    y_pos = np.arange(len(objects))

    plt.bar(y_pos, bestAlgorithmCount, align='center', alpha=0.5)
    plt.xticks(y_pos, objects)
    plt.ylabel('Count')
    plt.title('Best algorithm for Intruder')
    plt.show()


if __name__ == '__main__':
    testForSingleInitialLocation()
    #testForNRandomInitialLocation()
