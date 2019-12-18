#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Rapidly Expanding Random Tree: Image Obstacle

@author: mori
"""

import matplotlib.pyplot as plt
import numpy as np
import math as math
import cv2
from pdb import set_trace as bp
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


def bres_line(Clear, q_latest, q_goal, im_g):
    # bp()
    """ Bresenham's line algorithm for checking collisions with straight lines
    """
    # Implement Bresenham's Line Algorithm to determine if a straight line path is possible!
    # round down since q 40.8 is still in cell 40, not 41
    x0 = int(np.floor(q_latest[0]))
    y0 = int(np.floor(q_latest[1]))
    x1 = int(np.floor(q_goal[0]))
    y1 = int(np.floor(q_goal[1]))
    # x0 = int(q_latest[0])
    # y0 = int(q_latest[1])
    # x1 = int(q_goal[0])
    # y1 = int(q_goal[1])

    if im_g[y1, x1] == 0:  # implement line check and point check in one go!
        Clear = False
    # Vertical Check
    elif x1 - x0 == 0:  # deltax = 0 so line is vertical (inf. slope)
        # The sign function returns -1 if x < 0, 0 if x==0, 1 if x > 0.
        # nan is returned for nan inputs.
        y_incr = np.sign(y1 - y0)
        y = y0
        while y != y1:
            if im_g[y, x0] == 0:
                Clear = False
                break
            y = y + y_incr
    # Horizontal Check
    elif y1 - y0 == 0:  # deltay = 0 so line is horizontal (0 slope)
        # The sign function returns -1 if x < 0, 0 if x==0, 1 if x > 0.
        # nan is returned for nan inputs.
        x_incr = np.sign(x1 - x0)
        x = x0
        while x != x1:
            if im_g[y0, x] == 0:
                Clear = False
                break
            x = x + x_incr

    # Sloped Checks (B Algo)
    else:
        # dy < dx so slope < +/-45deg so m<1 (Octant 0,3,4,7).
        # If in Oct 0,4, reverse x0y0 and x1y1 (trajectory)
        # to operate in oct 3,7 (maintain +slope)
        if abs(y1 - y0) < abs(x1 - x0):
            if x0 > x1:
                dx = x0 - x1
                dy = y0 - y1
                # the following two lines are for octant 7
                y_incr = np.sign(dy)
                dy = dy * y_incr
                D = 2 * dy - dx
                y = y1

                for x in range(x1, x0):
                    if im_g[y, x] == 0:
                        Clear = False
                        break
                    if D > 0:
                        y = y + y_incr
                        D = D - 2 * dx
                    D = D + 2 * dy
            else:
                dx = x1 - x0
                dy = y1 - y0
                # the following two lines are for octant 7
                y_incr = np.sign(dy)
                dy = dy * y_incr
                D = 2 * dy - dx
                y = y0

                for x in range(x0, x1):
                    if im_g[y, x] == 0:
                        Clear = False
                        break
                    if D > 0:
                        y = y + y_incr
                        D = D - 2 * dx
                    D = D + 2 * dy

        else:
            # slope > +/-45deg so m>1 (Octant 1,2,5,6). If in Oct 5,6, reverse x0y0 and x1y1 (trajectory) to operate in oct 1,2 (maintain +slope)
            if y0 > y1:
                dx = x0 - x1
                dy = y0 - y1
                # the following two lines are for octant 2
                x_incr = np.sign(dx)
                dx = dx * x_incr
                D = 2 * dx - dy
                x = x1

                for y in range(y1, y0):
                    if im_g[y, x] == 0:
                        Clear = False
                        break
                    if D > 0:
                        x = x + x_incr
                        D = D - 2 * dy
                    D = D + 2 * dx
            else:
                dx = x1 - x0
                dy = y1 - y0
                # the following two lines are for octant 2
                x_incr = np.sign(dx)
                dx = dx * x_incr
                D = 2 * dx - dy
                x = x0

                for y in range(y0, y1):
                    if im_g[y, x] == 0:
                        Clear = False
                        break
                    if D > 0:
                        x = x + x_incr
                        D = D - 2 * dy
                    D = D + 2 * dx
    return Clear


def main():
    # initial configuration, starting node
    q_init = Node((40, 40), 0, None)
    nodes = [q_init]  # list containing nodes
    delta = 3  # incremental distance between steps

    # LOAD IMAGE
    # When accessing an image as an array, rows: y values and columns: x values
    im = plt.imread('N_Map.png')
    # print(np.shape(im))
    # since images by default have (0,0) at the top
    im = np.flipud(im)
    plt.imshow(im)
    # now indexing the array only returns 0 or 1
    im_g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    # Add obstacles
    obstacles = []
    for i in range(100):
        for j in range(100):
            if im_g[i, j] == 0.0:
                # change to row:x, col:y
                obstacles.append([i, j])
    # print(len(obstacles))

    # goal node
    q_goal = Node((60, 60), -1, 'end')

    K_trials = 10000  # number of nodes/vertices in RRT (num of repitions)

    for K in range(K_trials):
        # bp()
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

                clear = bres_line(True, q_n, q_new, im_g)

                if clear is False:
                    raise Exception
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

        # compare line intersect to each obstacle
        # initialize to true, if clear is still true after this loop
        # then straight shot to goal
        Clear = bres_line(True, q_latest, q_goal.position, im_g)

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
            plt.plot(newnodes_x, newnodes_y, color="darkviolet")
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
