'''
  Author:   Navil Abdeselam Abdel-lah
  Date:     06/03/2023
  Org:      Robesafe
 
  Library for waypoint interpolation using spline curves
'''

from math import cos, sin, pi, sqrt
import numpy as np

import matplotlib.pyplot as plt


def n_spline(waypoints, nu, lateral_offsets = None):
    '''
    Input:
        · waypoints: list of waypoints (x,y,o)
        · nu: number of points per segment
        · lateral_offset: lateral offset of the waypoints
    Output:
        · coefs: list of coefficients of the spline
    '''

    # 0. Aux variables
    m = len(waypoints)
    lam = []
    deta = []
    D = [0 for _ in range(m)]
    coefs = []

    # 1. Adding offset to the waypoints
    if lateral_offsets is not None:
        for offset, waypoint in zip(lateral_offsets, waypoints):
            waypoint.x = waypoint.x + offset*sin(waypoint.o)
            waypoint.y = waypoint.y - offset*cos(waypoint.o)

    # 2. Calculating splines for x-axis
    # 2.1 Calculating lambda and deta
    lam.append(0)
    deta.append(nu * cos(waypoints[0].o))
    for i in range(1, m - 1):
        lam.append(1 / (4 - lam[i - 1]))
        deta.append((3 * (waypoints[i + 1].x - waypoints[i - 1].x) - deta[i - 1]) * lam[i])
    deta.append(nu * cos(waypoints[-1].o))
    # 2.2 Calculating D
    D[m - 1] = deta[m - 1]
    for i in range(m - 2, 0, -1):
        D[i] = deta[i] - lam[i] * D[i + 1]
    # 2.3 Calculating coefficients
    for i in range(m-1):
        coef = [
        waypoints[i].x,  
        D[i],
        3 * (waypoints[i + 1].x - waypoints[i].x) - 2 * D[i] - D[i + 1],
        2 * (waypoints[i].x - waypoints[i + 1].x) + D[i] + D[i + 1],
        ]
        coefs.append(coef)

    # 3. Calculating splines for y-axis
    # 3.1 Calculating lambda and deta
    lam[0] = 0
    deta[0] = nu*sin(waypoints[0].o)
    for i in range(1, m - 1):
        lam[i] = 1 / (4 - lam[i - 1])
        deta[i] = ((3 * (waypoints[i + 1].y - waypoints[i - 1].y) - deta[i - 1]) * lam[i])
    deta[-1] = nu * sin(waypoints[-1].o)
    # 3.2 Calculating D
    D[m - 1] = deta[m - 1]
    for i in range(m - 2, 0, -1):
        D[i] = deta[i] - lam[i] * D[i + 1]
    # 3.3 Calculating coefficients
    for i in range(m - 1):
        coef = [
            waypoints[i].y,  # Add lateral offset
            D[i],
            3 * (waypoints[i + 1].y - waypoints[i].y) - 2 * D[i] - D[i + 1],
            2 * (waypoints[i].y - waypoints[i + 1].y) + D[i] + D[i + 1],
        ]
        coefs[i].extend(coef)

    return coefs

# Define a class for Rbs2_ControlPose for clarity
class Rbs2_ControlPose:
    def __init__(self, x, y, o):
        self.x = x
        self.y = y
        self.o = o

if __name__ == '__main__':
    # Plots
    fig, axis = plt.subplots()

    # Example usage:
    path = [    Rbs2_ControlPose(0.0, 0.0, pi/4), 
            Rbs2_ControlPose(0.5, 0.5, pi/4), 
            Rbs2_ControlPose(1.0, 1.0, pi/4),
            Rbs2_ControlPose(1.5, 1.5, pi/4), 
            Rbs2_ControlPose(2.0, 2.0, pi/4),
            Rbs2_ControlPose(2.5, 2.5, pi/4),
            Rbs2_ControlPose(3.0, 3.0, pi/4),
            Rbs2_ControlPose(3.5, 3.5, pi/4),
            Rbs2_ControlPose(4.0, 4.0, pi/4),
            Rbs2_ControlPose(4.5, 4.5, pi/4),
            ]

    offsets = [0,
            0,
            0,
            +0.5,
            +0.5,
            +0.5,
            0,
            0,
            0,           
            0]

    # Plot nominal path
    nu = 1
    coefs = n_spline(path, nu)
    for coef in coefs:
        ax, bx, cx, dx, ay, by, cy, dy  = coef
        u = np.linspace(0, 1, 100)
        x_spline = ax + bx*u + cx*u**2 + dx*u**3
        y_spline = ay + by*u + cy*u**2 + dy*u**3
        axis.plot(x_spline, y_spline,color='blue')

    x_path = [pose.x for pose in path]
    y_path = [pose.y for pose in path]
    axis.scatter(x_path, y_path, color='blue', label='Path')

    # Plot new path
    coefs = n_spline(path, nu, offsets)
    for coef in coefs:
        ax, bx, cx, dx, ay, by, cy, dy  = coef
        u = np.linspace(0, 1, 100)
        x_spline = ax + bx*u + cx*u**2 + dx*u**3
        y_spline = ay + by*u + cy*u**2 + dy*u**3
        axis.plot(x_spline, y_spline, color='red')

    x_path = [pose.x for pose in path]
    y_path = [pose.y for pose in path]
    axis.scatter(x_path, y_path, color='red', label='New path')

    axis.set_xlabel('X')
    axis.set_ylabel('Y')
    axis.legend()
    plt.grid()
    plt.show()