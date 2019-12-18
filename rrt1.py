#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Rapidly Expanding Random Tree: Empty Map

@author: Maurice Rahme
"""

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import math as math


def main():
    q_init = (50, 50)  # initial configuration, starting node
    nodes = [q_init]  # list containing nodes
    K = 0  # number of nodes/vertices in RRT (num of repitions), start at 0
    delta = 3  # incremental distance between steps
    """
    (in 2D)
    1. Pick a starting vertex and a goal vertex.
    2. Randomly pick a position (p) in the 2D sapce.
    3. Find the closes point (q) on an edge of the tree to position (p).
    4. Check if can move from (q) to (p) and if so add a new edge {(p), (q)}
    5. Go back to 2.
    """

    for K in range(250):

        q_rand = (np.random.uniform(10.1, 89.9), np.random.uniform(10.1, 89.9))
        # now find point closest to qrand

        # minimum distance from random point qrand to existing node
        mind = np.inf
        for n in range(len(nodes)):

            # distance from node n to qrand
            newd = math.sqrt(((nodes[n][0] - q_rand[0])**2) +
                             ((nodes[n][1] - q_rand[1])**2))
            # if new minimum distance found
            if newd < mind:
                mind = newd
                # record the index of the closest node
                mindex = n
        # closest node
        q_near = nodes[mindex]
        # vector from qrand to qnear
        q_vector = (q_rand[0] - q_near[0], q_rand[1] - q_near[1])

        # initialize into vector
        q_v = np.array(q_vector)
        q_n = np.array(q_near)

        mag = np.sqrt(q_v.dot(q_v))  # magnitude of vector

        # now move in direction of q vector by step delta by doing:
        q_new = q_n + (delta / mag) * q_v

        # now add q_new to nodes
        nodes.append(q_new)

        # plot newnode
        newnodes_x = (q_near[0], q_new[0])
        newnodes_y = (q_near[1], q_new[1])
        plt.plot(newnodes_x, newnodes_y,
                 color="darkviolet")  # Update in realtime

        xnodes = [xn[0] for xn in nodes]
        ynodes = [yn[1] for yn in nodes]

        # plt.plot(xnodes, ynodes, 'bo', markersize=2)
        plt.xlim((0, 100))
        plt.ylim((0, 100))
        plt.title('RRT: ' + str(K + 1) + ' Iterations')
        plt.ylabel('y domain')
        plt.xlabel('x domain')
        plt.pause(0.001)

    plt.show()


if __name__ == "__main__":
    main()