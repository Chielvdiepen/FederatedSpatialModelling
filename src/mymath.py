import math
import numpy as np
from math import log10, sqrt
from random import normalvariate
from main import RSSI

# Convert the length between two coordinates to RSSI
def distance_to_rssi(nodeCoord1, nodeCoord2):
    c1x, c1y, c1z = nodeCoord1
    c2x, c2y, c2z = nodeCoord2
    distance = sqrt((c1x-c2x)**2 + (c1y-c2y)**2 + (c1z-c2z)**2)
    # noise = normalvariate(0, sqrt(20))
    noise = 0
    rssi = -70 - 10 * 2.0 * log10(distance) + noise
    if RSSI:
        return rssi
    else:
        return distance


# Convert RSSI into distance, for now a log function or later try neural net conversion
def rssi_to_distance(rssi):
    if RSSI:
        return (10**((-rssi-70)/(20)))
    else:
        return rssi

# Planar Roll rotation will turn plane YZ around X
def roll(phi):
    phi = np.deg2rad(phi)
    return np.array([[1, 0, 0],
                     [0, np.cos(phi), -np.sin(phi)],
                     [0, np.sin(phi), np.cos(phi)]])

# Planar PITCH rotation will turn plane XZ around Y
def pitch(theta):
    theta = np.deg2rad(theta)
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

# Planar YAW rotation will turn plane XY around Z
def yaw(psi):
    psi = np.deg2rad(psi)
    return np.array([[np.cos(psi), -np.sin(psi), 0],
                    [np.sin(psi), np.cos(psi), 0],
                    [0, 0, 1]])