import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

np.random.seed(19680801)
A_COUNT = 1
B_COUNT = 1
SIMULATION_TIME = 5000


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
        """
        Create a line using a random walk algorithm

        length is the number of points for the line.
        dims is the number of dimensions the line has.
        """
        lineData = np.empty((dims, length))
        lineData[:, 0] = np.random.rand(dims)
        for index in range(1, length):
            # scaling the random numbers by 0.1 so
            # movement is small compared to position.
            # subtraction by 0.5 is to change the range to [-0.5, 0.5]
            # to allow a line to move backwards.
            step = ((np.random.rand(dims) - 0.5) * 0.1)
            lineData[:, index] = lineData[:, index - 1] + step

        return lineData

    # def simulate_algorithm(self):


    def update_lines(self, num, dataLines, lines):
        for line, data in zip(lines, dataLines):
            # NOTE: there is no .set_data() for 3 dim data...
            line.set_data(data[0:2, :num])
            line.set_3d_properties(data[2, :num])
        return lines

    def simulate(self):
        # Attaching 3D axis to the figure
        fig = plt.figure()
        ax = p3.Axes3D(fig)

        # 2 lines of random 3-D lines
        data = [self.gen_rand_line(5000, 3) for index in range(2)]

        # Creating fifty line objects.
        # NOTE: Can't pass empty arrays into 3d version of plot()
        lines = [ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in data]

        # Setting the axes properties
        ax.set_xlim3d([0.0, 2.0])
        ax.set_xlabel('X')

        ax.set_ylim3d([0.0, 2.0])
        ax.set_ylabel('Y')

        ax.set_zlim3d([0.0, 2.0])
        ax.set_zlabel('Z')

        ax.set_title('3D Test')

        # Creating the Animation object
        line_ani = animation.FuncAnimation(fig, self.update_lines, 500, fargs=(data, lines),
                                           interval=500, blit=False)

        plt.show()


class A:
    LINE_OF_SIGHT = 10
    LINE_OF_PARTIAL_SIGHT = 10
    LINE_OF_CONTROL = 10
    DIMENSION = 3
    SIMULATION_TIME= 5000

    def __init__(self, los, lops, loc, dims, tot_time):
        self.LINE_OF_SIGHT = los
        self.LINE_OF_PARTIAL_SIGHT = lops
        self.LINE_OF_CONTROL = loc
        self.DIMENSION = dims
        self.SIMULATION_TIME = tot_time
        self.PATH_DATA = np.empty((dims, tot_time))


class B:
    LINE_OF_SIGHT = 10
    LINE_OF_PARTIAL_SIGHT = 10
    LINE_OF_CONTROL = 10
    DIMENSION = 3
    SIMULATION_TIME= 5000

    def __init__(self, los, lops, loc, dims, tot_time):
        self.LINE_OF_SIGHT = los
        self.LINE_OF_PARTIAL_SIGHT = lops
        self.LINE_OF_CONTROL = loc
        self.DIMENSION = dims
        self.SIMULATION_TIME = tot_time
        self.PATH_DATA = np.empty((dims, tot_time))


if __name__ == '__main__':
    simulator = Simulation(A_COUNT, B_COUNT, SIMULATION_TIME)
    simulator.simulate()
