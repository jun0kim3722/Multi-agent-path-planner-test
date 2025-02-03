import sys
sys.path.append('/home/j0k/coral/ompl-1.5.2/py-bindings')

from ompl import base as ob
from ompl import geometric as og
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.patches as patches
from copy import deepcopy

def animate_motion(paths, space, file_name, obs=[]):
    # AGT_R = 0.2
    # NUM_AGT = 2
    max_dist = 0
    for path in paths:
        dist = 0
        for i in range(len(path[1:])):
            dist += np.sqrt((path[i][0] - path[i-1][0]) ** 2 + (path[i][1] - path[i-1][1]) ** 2)
        max_dist = max(dist, max_dist)
    num_frame = int(max_dist // 0.1 + 5)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(space[0][0], space[0][1])
    ax.set_ylim(space[1][0], space[1][1])
    color = ["red", "blue", "green", "pink", "orange", "cyan", "brown", "purple"]

    s_points = []
    for path in paths:
        s_points.append(path[0])

    # Set up lists for animation elements
    prev_pos = np.array(s_points)
    prev_idx = [0] * len(paths)
    is_done = [False] * len(paths)
    agent_patches = [plt.Circle((pos[0], pos[1]), 0.2, color=color[i]) for i, pos in enumerate(s_points)]

    for patch in agent_patches:
        ax.add_patch(patch)

    def update(frame):
        ax.clear()
        ax.set_xlim(space[0][0], space[0][1])
        ax.set_ylim(space[1][0], space[1][1])
        for i in range(len(paths)):
            path = paths[i]
            ax.plot([x[0] for x in path], [y[1] for y in path], color=color[i])
            ax.plot([x[0] for x in path], [y[1] for y in path], ".", color=color[i])

            for cx, cy, hx, hy in obs:
                temp_obs = patches.Rectangle((cx - hx, cy - hy), hx * 2, hy * 2, facecolor = 'red')
                plt.gca().add_patch(temp_obs)

            if is_done[i]:
                plot_agt_pos(ax, paths[i][-1], color[i])
                continue

            vec = np.array(paths[i][prev_idx[i] + 1]) - prev_pos[i]
            dist = np.linalg.norm(vec)
            # print(i, dist, prev_pos[i], prev_idx[i])
            
            if dist < 0.1:
                if prev_idx[i] + 1 >= len(paths[i]):
                    is_done[i] = True
                    prev_pos[i] = paths[i][-1]
                else:
                    prev_idx[i] += 1
                    vec = np.array(paths[i][prev_idx[i]]) - prev_pos[i]
                    new_dist = 0.1 - dist

                    while np.linalg.norm(vec) == 0:
                        prev_idx[i] += 1
                        vec = np.array(paths[i][prev_idx[i]]) - prev_pos[i]

                    move = vec / np.linalg.norm(vec) * new_dist
                    prev_pos[i] = prev_pos[i] + move
            else:
                move = vec / np.linalg.norm(vec) / 10
                prev_pos[i] = deepcopy(np.array(prev_pos[i] + move))
                # prev_idx[i] += 1

            plot_agt_pos(ax, prev_pos[i], color[i])

            if prev_idx[i] + 1 >= len(paths[i]):
                is_done[i] = True

    ani = FuncAnimation(fig, update, frames=range(num_frame), repeat=False)
    ani.save(file_name, writer=PillowWriter(fps=10))
    plt.cla()
    plt.close()

def plot_agt_pos(ax, point, color=None):
    cir = plt.Circle((point[0], point[1]), 0.2, color=color)
    ax.add_patch(cir)
 
if __name__ == "__main__":
    s = [[1.6, -1.6],
         [-1.6, -1.6]]

    g = [[-1.6, 1.6],
         [1.6, 1.6]]

    AGT_R = 0.2
    NUM_AGT = 2

    # multi_agt_plan(s, g)