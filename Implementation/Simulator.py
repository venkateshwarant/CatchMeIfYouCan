import numpy as np
import random as rd

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

from A import A, DefenderRegionTypes
from B import B, IntruderTrajectoryTypes
from GlobalConstants import *
from utils import Vector3D, UniformMotion, HelixMotion, ConstantAccelerationAway, \
    ConstantAccelerationTo, ConstantAccelerationPerpendicular, RandomMotion, RandomMotionInsideCone, FieldViewMotion

rd.seed(1000)

SIMULATION_LOOPS = round(GlobalConstants.MAX_SIMULATION_TIME / GlobalConstants.TIME_DELTA)

distance= list()
# Run this to simulate and visualise the movement of A and B, and to plot the distance convergence graph
if __name__ == '__main__':

    initial_velocity = Vector3D(1, 2, 3)
    intruder = B(
        position=Vector3D(50, 50, 40),
        velocity=initial_velocity,
        danger_zone=20,
        chase_zone=80,
        free_motion=HelixMotion(radius_magnitude=5, velocity_magnitude=initial_velocity.magnitude, time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD),
        # free_motion=RandomMotionInsideCone(acceleration_magnitude=1.5, max_velocity=10, min_theta=75, max_theta=90, time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD),
        chase_motion=ConstantAccelerationAway(acceleration_magnitude=1.5, max_velocity=10, time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD),
        danger_motion=ConstantAccelerationPerpendicular(acceleration_magnitude=1.8, max_velocity=15, time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD),
    )

    initial_velocity = Vector3D(1, 2, 3)
    defender = A(
        position=Vector3D(0, 0, 0),
        velocity=initial_velocity,
        line_of_control=GlobalConstants.LOC,
        line_of_sight=GlobalConstants.LOS,
        line_of_field_view=GlobalConstants.LOFV,
        random_motion=UniformMotion(velocity_magnitude=initial_velocity.magnitude, time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD),
        chase_motion=ConstantAccelerationTo(acceleration_magnitude=1.8, max_velocity=15, time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD),
        field_view_motion=FieldViewMotion(velocity_magnitude=initial_velocity.magnitude, time_delta=GlobalConstants.TIME_DELTA, constraints=GlobalConstants.WORLD),
    )

    x_data_intruder, y_data_intruder, z_data_intruder = list(), list(), list()
    x_data_defender, y_data_defender, z_data_defender = list(), list(), list()
    intruder_trajectory_types = list()

    defender.update_trajectory_type(intruder.position)
    intruder.update_trajectory_type(defender.position)
    i = 0
    time_list= list()
    t0=0
    simulation_end_time=0
    for i in range(SIMULATION_LOOPS):
        time_list.append(t0)
        print("Defender:", defender.velocity.magnitude, defender)
        print("Intruder:", intruder.velocity.magnitude, intruder)
        distance.append(np.linalg.norm(defender.position.vector - intruder.position.vector))
        print("Distance Between them:", np.linalg.norm(defender.position.vector - intruder.position.vector))
        print("Defender Trajectory Type", defender.trajectory_type)
        print("Intruder Trajectory Type", intruder.trajectory_type)
        print()

        if defender.trajectory_type == DefenderRegionTypes.LOC:
            simulation_end_time = round(i*GlobalConstants.TIME_DELTA+1)
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

    if defender.trajectory_type == DefenderRegionTypes.LOC:
        print("Simulation Ended because of Defender caught intruder.")
        print("It lasted for:", i * GlobalConstants.TIME_DELTA, "seconds")
    else:
        print("Simulation ended because of TIME OUT")
        print("It lasted for:", GlobalConstants.MAX_SIMULATION_TIME, "seconds")

    # Plot Setup
    fig = plt.figure(1)
    ax = p3.Axes3D(fig)
    # Setting the axes properties
    ax.set_xlim3d([0.0, GlobalConstants.WORLD.x])
    ax.set_xlabel('X')
    ax.set_ylim3d([0.0, GlobalConstants.WORLD.y])
    ax.set_ylabel('Y')
    ax.set_zlim3d([0.0, GlobalConstants.WORLD.z])
    ax.set_zlabel('Z')



    # For showing the trajectory of both intruder and defender
    defender_path = [np.asarray(x_data_defender), np.asarray(y_data_defender), np.asarray(z_data_defender)]
    intruder_path = [np.asarray(x_data_intruder), np.asarray(y_data_intruder), np.asarray(z_data_intruder)]
    all_paths = [np.asarray(defender_path), np.asarray(intruder_path)]
    lines = [ax.plot(np.asarray(dat)[0, 0:1], np.asarray(dat)[1, 0:1], np.asarray(dat)[2, 0:1])[0] for dat in all_paths]

    # For showing the current position of intruder
    points, = ax.plot(x_data_intruder, y_data_intruder, z_data_intruder, '*')

    def update_lines(num, dataLines, lines, intruder_trajectory_types, points):
        # print('Function called:', num)
        num = num + 1
        increment_per_call = 1 * (1/GlobalConstants.TIME_DELTA) * GlobalConstants.SPEEDUP_FACTOR * GlobalConstants.FRAME_INTERVAL
        new_index = round(min(increment_per_call * num, len(intruder_trajectory_types)))

        for line, data in zip(lines, dataLines):
            line.set_data(data[0:2, :new_index])
            line.set_3d_properties(data[2, :new_index])

        color = None
        if intruder_trajectory_types[new_index - 1] == IntruderTrajectoryTypes.SAFE:
            color = 'green'
        elif intruder_trajectory_types[new_index - 1] == IntruderTrajectoryTypes.CHASE:
            color = 'blue'
        elif intruder_trajectory_types[new_index - 1] == IntruderTrajectoryTypes.DANGER:
            color = 'red'
        points.set_data(dataLines[1][0][new_index-1], dataLines[1][1][new_index - 1])
        points.set_3d_properties(dataLines[1][2][new_index-1], 'z')
        points.set_color(color)



    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, update_lines, None, fargs=(np.asarray(all_paths), lines, intruder_trajectory_types, points), interval=GlobalConstants.FRAME_INTERVAL*1000, blit=False)

    fig2 = plt.figure("Convergence graph")
    ax = fig2.add_subplot(111)
    fig2.subplots_adjust(top=0.85)
    ax.set_title('Convergence graph')
    loc_plot = []
    lofv_plot = []
    los_plot = []
    for i in range(0, simulation_end_time):
        loc_plot.append(GlobalConstants.LOC)
        los_plot.append(GlobalConstants.LOS)
        lofv_plot.append(GlobalConstants.LOFV)

    plt.plot(loc_plot, color='red')
    plt.annotate('Line of control', xy=(0, 14), xytext=(0, GlobalConstants.LOC+3))
    plt.plot(los_plot, color='orange')
    plt.annotate('Line of sight', xy=(0, 14), xytext=(0, GlobalConstants.LOS+3))
    plt.plot(lofv_plot, color='green')
    plt.annotate('Line of field view', xy=(0, 14), xytext=(0, GlobalConstants.LOFV+3))
    plt.plot(time_list, distance)
    plt.xlabel('time (s)')
    plt.ylabel('Distance (m)')
    plt.show()