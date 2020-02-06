import math
import numpy
from scipy.special import lambertw
import warnings

# Variables for artificial position field calculations
AGENT_COUNT = 1  # Number of Agents/Quadcopters
NODE_COUNT = 1  # Number of Attractive Nodes
DIMENSION_COUNT = 3  # Dimensions

MAX_ATTRACTIVE_FORCE = 10       # Max Attractive Force
EQUILIBRIUM_DISTANCE = 2        # 
STEEPNESS_COEFFICIENT = 100     #
SMOOTHNESS_COEFFICIENT = 5      #
ATTRACTIVE_FRACTION = (1 / (2 * math.e)) * (NODE_COUNT / AGENT_COUNT)
REPULSIVE_FRACTION = (1 / (2 * math.e)) * (AGENT_COUNT / NODE_COUNT)

ATTRACTIVE_GAIN = MAX_ATTRACTIVE_FORCE / NODE_COUNT  # Attractive Gain

# compiler gives a warning that the following is stored as a real value from a complex value
# which discards the imaginary part, this should be fine since the imaginary portion should be 0 because the
# branch value passed to the function, the warning below should fix this
warnings.filterwarnings("ignore", message="Casting complex values to real discards the imaginary part")

# lambert(z, k) = lambert of zed for the kth branch
gamma = float((-SMOOTHNESS_COEFFICIENT / (math.pow(EQUILIBRIUM_DISTANCE, 2) + SMOOTHNESS_COEFFICIENT))
              * lambertw(-1 / (math.e * STEEPNESS_COEFFICIENT), k=-1))

repulsive_gain = float(
    ATTRACTIVE_GAIN * ((1 - ATTRACTIVE_FRACTION) / (1 - REPULSIVE_FRACTION)) * math.sqrt(
        (math.e * STEEPNESS_COEFFICIENT * SMOOTHNESS_COEFFICIENT) / (gamma * math.exp(gamma))))

repulsive_aoe = float(
    2 * SMOOTHNESS_COEFFICIENT / lambertw(math.pow(math.e * STEEPNESS_COEFFICIENT * SMOOTHNESS_COEFFICIENT * (
            (ATTRACTIVE_GAIN * (1 - ATTRACTIVE_FRACTION)) / (repulsive_gain * (1 - REPULSIVE_FRACTION))), 2),
                                          k=0))
attractive_aoe = repulsive_aoe
long_range_repulsive = ATTRACTIVE_FRACTION * ATTRACTIVE_GAIN
short_range_attractive = REPULSIVE_FRACTION * repulsive_gain

# TODO: Better name?
xx = numpy.linspace(-10, 10, 1001)

# TODO: Purpose of the variables below, as they don't seem to be used in the function

Va = ATTRACTIVE_GAIN * numpy.sqrt(numpy.multiply(xx, xx) + SMOOTHNESS_COEFFICIENT) - short_range_attractive * numpy.exp(
    -numpy.multiply(xx, xx) / attractive_aoe)
Vr = repulsive_gain * numpy.exp(-numpy.multiply(xx, xx) / repulsive_aoe) - long_range_repulsive * numpy.sqrt(
    numpy.multiply(xx, xx) + SMOOTHNESS_COEFFICIENT)
V = Vr + Va

# TODO: Sliding gains and reaching gains remain constant per run? If not revert change and parameterize them again.

# SMC Gains
SATURATION_LEVEL = 1
# SLIDING_GAINS consists of [ Kx, Ky, Kz]
SLIDING_GAINS = numpy.array([3, 3, 20])

# REACHING_GAINS consists of [Cx, Cy, Cz]
REACHING_GAINS = numpy.array([1.5, 1.5, 1.5])

# Node Initial positions
# TODO: purpose of xad and why are all the initial node positions consists of 6 different values, expected 3.
xad = [0, 0, 0]
A1 = [0, 0, 5] + xad
A2 = [0, 4, 5] + xad
A3 = [4, 20, 5] + xad
A4 = [0, 20, 5] + xad
A5 = [0, 0, 5] + xad
A6 = [0, 0, 15] + xad
A7 = [0, 0, 18] + xad
A8 = [0, 0, 21] + xad
AX = [A1, A2, A3, A4, A5, A6, A7, A8]

# TODO: For testing purposes...)
NODE_LOCATION = [[10, 3, 5]]
# AX3d = [A1;A2;A3;A4;A5;A6;A7;A8]

# Agent Initial Positions
# Should come from GPSs
X1 = [0, 2, 0]
X2 = [0, - 2, 0]
X3 = [2, 0, 0]
X4 = [-2, 0, 0]
X5 = [0, 0, 0]
X6 = [10, - 10, 0]
X7 = [-10, - 10, 0]
X8 = [-10, 10, 0]
X0 = [0, 0, 0]

X0 = numpy.append(X0, X1)
X0 = numpy.append(X0, X2)
X0 = numpy.append(X0, X3)
X0 = numpy.append(X0, X4)
X0 = numpy.append(X0, X5)
X0 = numpy.append(X0, X6)
X0 = numpy.append(X0, X7)
X0 = numpy.append(X0, X8)

agents_loc = [X1]
agents_vel = [2, 2, 2]
angles = [0, 0, 0]


def swarm_next_location(node_locations, agents_locations, agents_velocities, agents_angles):
    """
    Swarm function

    go through each agent and compare positions with other quads
    find the difference between two locations
    find the norm (euclidean distance) between two agents


    :param node_locations: represents location of the target node represented as a matrix
    :param agents_locations: locations of the agents in an 2 dimensional array
    :param agents_velocities: velocities of the agents in an array
    :param agents_angles: TODO: Numpy uses radians, this isn' used at all currently from what I know?
    """

    # Potential Field Gradient Calculation

    # Gradient of potential field
    dv = numpy.zeros((AGENT_COUNT, DIMENSION_COUNT)) # create an array of values

    for agent_it_1 in range(0, AGENT_COUNT):
        # Inter-Agent Forces
        for agent_it_2 in range(0, AGENT_COUNT):
            n_x = int(numpy.linalg.norm(numpy.subtract(agents_locations[agent_it_1], agents_locations[agent_it_2])))

            for dimension_it in range(0, DIMENSION_COUNT):
                delta_x = agents_locations[agent_it_1][dimension_it] - agents_locations[agent_it_2][dimension_it]
                dv[agent_it_1][dimension_it] = dv[agent_it_1][dimension_it] - long_range_repulsive * (
                        delta_x / numpy.sqrt((SMOOTHNESS_COEFFICIENT ^ 2) + n_x ^ 2)) - 2 * (
                                     repulsive_gain / repulsive_aoe) * delta_x * numpy.exp((-n_x ^ 2) / repulsive_aoe)
        # Formation Attraction Forces
        if NODE_COUNT > 0:
            for node_it in range(0, NODE_COUNT):
                n_x = int(
                    numpy.linalg.norm(
                        numpy.subtract(agents_locations[agent_it_1],
                                       node_locations[node_it])))  # norm of the vector between two bots

                for dimension_it in range(0, DIMENSION_COUNT):
                    delta_x = agents_locations[agent_it_1][dimension_it] - node_locations[node_it][dimension_it]
                    dv[agent_it_1][dimension_it] = dv[agent_it_1][dimension_it] + ATTRACTIVE_GAIN * (
                            delta_x / numpy.sqrt((SMOOTHNESS_COEFFICIENT ^ 2) + n_x ^ 2)) + (
                                         short_range_attractive / attractive_aoe) * delta_x * numpy.exp((-n_x ^ 2) /
                                                                                                        attractive_aoe)
    sliding_surface = numpy.add(agents_velocities, dv)
    # Saturation Block [sat(s)]

    sx = numpy.zeros(numpy.size(sliding_surface[0]))
    for agent_it_1 in range(0, AGENT_COUNT):
        for dimension_it in range(0, DIMENSION_COUNT):
            if abs(sliding_surface[agent_it_1][dimension_it]) > SATURATION_LEVEL:
                # FIXME: not sure if this fix was correct but I changed Sx(ip, di) -> sx[ip+di] based on values found
                #  in MATLAB code sample
                sx[agent_it_1 + dimension_it] = numpy.sign(sliding_surface[agent_it_1][dimension_it]) * SATURATION_LEVEL
            else:
                sx[agent_it_1 + dimension_it] = sliding_surface[agent_it_1][dimension_it]
    # Gains

    c = numpy.zeros((AGENT_COUNT, DIMENSION_COUNT))
    k = numpy.zeros((AGENT_COUNT, DIMENSION_COUNT))

    # TODO: should be able to make the loop faster somehow
    # row by row multiplication
    for agent_it_1 in range(0, AGENT_COUNT):
        c[agent_it_1] = numpy.multiply(agents_velocities[agent_it_1], REACHING_GAINS)
        k[agent_it_1] = numpy.multiply(sx[agent_it_1], SLIDING_GAINS)

    u0 = k + c

    print(u0)

    return u0


swarm_next_location(NODE_LOCATION, agents_loc, agents_vel, angles)
