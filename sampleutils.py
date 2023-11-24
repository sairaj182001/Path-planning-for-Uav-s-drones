class Env:
    def __init__(self):
        self.x_range = (0, 50)
        self.y_range = (0, 50)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

        return obs_cir

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import sys



class Plotting:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.env = Env()
        self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle

    def animation(self, nodelist, path, name, animation=False):
        self.plot_grid(name)
        self.plot_visited(nodelist, animation)
        self.plot_path(path)

    def animation_connect(self, V1, V2, path, name):
        self.plot_grid(name)
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)

    def plot_grid(self, name):
        fig, ax = plt.subplots()

        for (ox, oy, w, h) in self.obs_bound:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obs_circle:
            ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3)
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    @staticmethod
    def plot_visited(nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)

    @staticmethod
    def plot_path(path):
        if len(path) != 0:
            plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
            plt.pause(0.01)
        plt.show()

import math
import numpy as np
import os
import sys



class Utils:
    def __init__(self):
        self.env = Env()

        self.delta = 0.5
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def update_obs(self, obs_cir, obs_bound, obs_rec):
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec

    def get_obs_vertex(self):
        delta = self.delta
        obs_list = []

        for (ox, oy, w, h) in self.obs_rectangle:
            vertex_list = [[ox - delta, oy - delta],
                           [ox + w + delta, oy - delta],
                           [ox + w + delta, oy + h + delta],
                           [ox - delta, oy + h + delta]]
            obs_list.append(vertex_list)

        return obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_intersect_circle(self, o, d, a, r):
        d2 = np.dot(d, d)
        delta = self.delta

        if d2 == 0:
            return False

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        if 0 <= t <= 1:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            if self.get_dist(shot, Node(a)) <= r + delta:
                return True

        return False

    def is_collision(self, start, end):
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obs_vertex()

        for (v1, v2, v3, v4) in obs_vertex:
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True

        for (x, y, r) in self.obs_circle:
            if self.is_intersect_circle(o, d, [x, y], r):
                return True

        return False

    def is_inside_obs(self, node):
        delta = self.delta

        for (x, y, r) in self.obs_circle:
            if math.hypot(node.x - x, node.y - y) <= r + delta:
                return True

        for (x, y, w, h) in self.obs_rectangle:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        for (x, y, w, h) in self.obs_boundary:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        return False

    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None