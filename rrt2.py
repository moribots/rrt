#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Rapidly Expanding Random Tree: Circular Obstacles

@author: mori
"""

import matplotlib.pyplot as plt
import numpy as np
import math as math
from matplotlib.patches import Circle
"""
(in 2D),
1. Pick a starting vertex and a goal vertex.
2. Randomly pick a position (p) in the 2D sapce.
3. Find the closes point (q) on an edge of the tree to position (p).
4. Check if can move from (q) to (p) (no collisions): add a edge {(p), (q)}.
5. Once new node added, record position and parent.
6. Go back to 2.
"""


class Node():
    def __init__(self, position, parent, children):
        # each node has one parent, but can have multiple children
        self.position = position  # x,y
        self.parent = parent  # return index of nodes[] array
        self.children = children  # return list of indices of nodes[] array


def main():
    # initial configuration, starting node
    q_init = Node((50, 50), 0, None)
    nodes = [q_init]  # list containing nodes
    delta = 1  # incremental distance between steps

    # Draw circles of random sizes and centres, circle collision irrelevant
    # Randomly generate goal node
    q_goal = Node(
        (np.random.uniform(10.1, 89.9), np.random.uniform(10.1, 89.9)), -1,
        'end')

    centres = []
    radii = []
    C = 20  # number of circles desired
    K_trials = 1000  # number of nodes/vertices in RRT (num of repitions)
    """
    try:
        statements # statements that can raise exceptions
    except:
        statements # statements that will be executed to handle exceptions
    else:
        statements # statements that will be executed if there is no exception
    """

    # SETUP OBSTACLES
    for c in range(C):  # num of circles
        while True:
            try:  # try below methods

                # print('attempt', c)
                centre = (np.random.uniform(10.1, 89.9),
                          np.random.uniform(10.1, 89.9))
                radius = np.random.uniform(5.0, 10.0)

                # Circles not on starting node and randomly generated end node

                # circle eqn: (x-x_c)^2+(y-y_c)^2 = r^2

                # check for collision with init
                q_i_chk = ((q_init.position[0] - centre[0])**2 +
                           (q_init.position[1] - centre[1])**2)
                # check for collision with goal
                q_g_chk = ((q_goal.position[0] - centre[0])**2 +
                           (q_goal.position[1] - centre[1])**2)

                if q_i_chk <= radius**2 or q_g_chk <= radius**2:
                    # collision
                    raise Exception  # raise Exception if collision found
            except:  # if exception raised, continue while loop (ie try again)
                continue
            else:  # if exception not raised, break while loop (ie next circle)
                break
        # store centre and radius of each circle w same index
        centres.append(centre)
        radii.append(radius)

    # Create a figure. Equal aspect so circles look circular
    # https://stackoverflow.com/questions/34902477/drawing-circles-on-image-with-matplotlib-and-numpy
    fig, ax = plt.subplots(1)
    ax.set_aspect('equal')

    # PLOT OBSTACLES
    for ci in range(len(centres)):
        circ = Circle((centres[ci][0], centres[ci][1]),
                      radii[ci],
                      color="darkviolet")
        ax.add_patch(circ)
        #ax.set_color('black') #how to change color?

    for K in range(K_trials):
        # Check if q_new falls within an obstacle
        while True:
            try:
                q_rand = (np.random.uniform(10.1, 89.9),
                          np.random.uniform(10.1, 89.9))

                # minimum distance from random point qrand to existing node
                mind = np.inf

                # now find point closest to qrand
                for n in range(len(nodes)):
                    # distance from node n to qrand
                    newd = math.sqrt(((nodes[n].position[0] - q_rand[0])**2) +
                                     ((nodes[n].position[1] - q_rand[1])**2))
                    # if new minimum distance found
                    if newd < mind:
                        # record the index of the closest node
                        mind = newd
                        mindex = n
                # closest node
                q_near = nodes[mindex].position
                # vector from qrand to qnear
                q_vector = (q_rand[0] - q_near[0], q_rand[1] - q_near[1])

                # initialize into vector
                q_v = np.array(q_vector)
                q_n = np.array(q_near)

                mag = np.sqrt(q_v.dot(q_v))  # magnitude of vector

                # now move in direction of q vector by step delta by doing:
                q_new = q_n + (delta / mag) * q_v

                # Ensure no collision with circles
                for cen in range(len(centres)):
                    q_n_chk = ((q_new[0] - centres[cen][0])**2 +
                               (q_new[1] - centres[cen][1])**2)

                    if q_n_chk <= radii[cen]**2:

                        raise Exception  # raise Exception if collision found

            except:
                continue
            else:
                q_new = Node(
                    (q_new[0], q_new[1]), mindex,
                    None)  # parent of q_new is q_near, only need index
                break

        # Check if q_new has a clear line of sight to the target, if so, append
        # end goal to nodes list and plot straight line to finish. Do before
        # adding q_new since there may be no obstacles to begin with

        # Need to use http://paulbourke.net/geometry/pointlineplane/

        # compare latest node to q_goal

        q_latest = nodes[-1].position

        P1 = np.array(q_latest)
        P2 = np.array(q_goal.position)

        # P12 = P1 - P2

        # mag_P1P2 = np.sqrt(P12.dot(P12))

        # compare line intersect to each obstacle
        # initialize to true, if clear is still true after this loop
        # then straight shot to goal
        Clear = True
        for cent in range(len(centres)):

            P3 = np.array(centres[cent])

            u = np.dot(P3 - P1, P2 - P1) / np.dot(P2 - P1, P2 - P1)

            P4 = np.array(
                (P1[0] + u * (P2[0] - P1[0]), P1[1] + u * (P2[1] - P1[1])))

            d_intx = np.sqrt(np.dot(P4 - P3, P4 - P3))

            # u cond bcs if u<0 then the circle not even btw q_latest & q_goal
            if d_intx <= radii[cent] and 0 <= u <= 1:
                # Not a clear shot to goal from new node
                Clear = False
                # print('Obstacle!')
                # print('attempt', cent)
                # print('\n')
                break

        # replace q_new with q_goal instead of q_rand step if straight shot
        if Clear is True:
            # index of latest, which will be -2 when q_new/goal is appended
            q_goal.parent = -2
            q_new = q_goal

        # If no obstacle, add q_new to nodes
        nodes.append(q_new)

        if Clear is False:
            newnodes_x = (q_near[0], q_new.position[0])
            newnodes_y = (q_near[1], q_new.position[1])
            plt.plot(newnodes_x, newnodes_y, color="orange")
        else:
            newnodes_x = (q_latest[0], q_new.position[0])
            newnodes_y = (q_latest[1], q_new.position[1])
            plt.plot(newnodes_x, newnodes_y, 'g')  # draw path to goal in green

        # newnodes = [q_near, q_new]
        # line_segments = mc.LineCollection(newnodes)
        # ax.add_collection(line_segments)

        xnodes = [xn.position[0] for xn in nodes]
        ynodes = [yn.position[1] for yn in nodes]

        # plt.plot(xnodes, ynodes, 'bo', markersize=2)
        plt.plot(q_init.position[0], q_init.position[1], 'ro', markersize=7)
        plt.plot(q_goal.position[0], q_goal.position[1], 'go', markersize=7)
        plt.xlim((0, 100))
        plt.ylim((0, 100))
        plt.title('RRT :' + str(K + 1) + 'iterations')
        plt.ylabel('y domain')
        plt.xlabel('x domain')
        plt.pause(0.001)

        # Then break loop and end algorithm
        if Clear is True:
            # print('Clear!')
            break

    # Finally, draw path from start to goal in red via node parents
    # note that q_new = q_goal

    path = [q_goal.position]  # since q_new = q_goal at this point
    q_next = Node(nodes[-2].position, nodes[-2].parent,
                  None)  # since index -2 is next after goal (index -1)
    while True:

        if q_next.position == q_init.position:
            path.append(q_next.position)
            break
        else:
            path.append(q_next.position)
            q_next = Node(nodes[q_next.parent].position,
                          nodes[q_next.parent].parent, None)

    # PLOT
    xpath = [xp[0] for xp in path]
    ypath = [yp[1] for yp in path]
    plt.plot(xpath, ypath, 'g', markersize=2)
    plt.plot(q_init.position[0], q_init.position[1], 'ro', markersize=7)
    plt.plot(q_goal.position[0], q_goal.position[1], 'go', markersize=7)

    plt.show()


if __name__ == "__main__":
    main()
