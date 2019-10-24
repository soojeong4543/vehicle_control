#######
# This is class containing all constants relevant to the test scene

import math


class scene_constants:

    # List of constants

    # client ID
    clientID            = -1

    # object Handles
    vehicle_handle      = None
    goal_handle         = None

    colHandle           = None
    refHandle           = None

    refPos              = None
    refOri              = None

    # Maximum steering angle in degrees
    max_steer           = 30
    min_steer           = -1*max_steer

    # Simulation Parameters
    dt = 0.025                      # dt of the vrep simulation

    CTR_FRQ = 0.2

    Vx=2.77
