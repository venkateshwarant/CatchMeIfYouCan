from utils import Vector3D


class GlobalConstants:
    TIME_DELTA = 0.2
    MAX_X = 200
    MAX_Y = 200
    MAX_Z = 200

    WORLD = Vector3D(MAX_X, MAX_Y, MAX_Z)

    # MOTION Constants
    MAX_SIMULATION_TIME = 1000

    # ANIMATION Constants
    FRAME_INTERVAL = 0.2  # Seconds after which the frame is refreshed
    SPEEDUP_FACTOR = 5  # Every actual second of real life is like SPEEDUP_FACTOR seconds in animation

    #Constants for Defender
    LOC= 5
    LOS= 80
    LOFV= 100
    FIELD_ANGLE= 40