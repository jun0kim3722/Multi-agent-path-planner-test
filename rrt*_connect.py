import numpy as np
import itertools
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
from main import animate_motion
from test_case_generator import create_env, get_config
import sys
import pdb

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0  # Cost to reach this node
        self.edge = 0.0
        self.connected = False
        self.conflict = []
        self.inGoal = False

    def copy(self):
        return Node(self.x, self.y)

class RRTConnect:
    def __init__(self, space, starts, goals, obs=[], step_size=1.0, goal_bias=0.3, neighbor_radius=0.2, similarities=0.7):
        self.space = space
        self.num_agt = len(starts)
        self.solutions = []
        self.tree_starts = []
        self.tree_goals = []
        self.starts = []
        self.goals = []
        self.min_paths = []
        self.check_path = False
        self.similarities = similarities

        for i in range(self.num_agt):
            start = Node(starts[i][0], starts[i][1])
            goal = Node(goals[i][0], goals[i][1])
            goal.inGoal = True
            self.starts.append(start)
            self.goals.append(goal)
            self.tree_starts.append([start])
            self.tree_goals.append([goal])
            self.solutions.append({})
            self.min_paths.append([])

        self.step_size = step_size
        self.goal_bias = goal_bias
        self.neighbor_radius = neighbor_radius  # RRT* rewiring radius
        self.obs = obs
        self.run_time = 0

    def distance(self, node1, node2):
        return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    def sample_point(self, is_start, agt_idx):
        if np.random.rand() < self.goal_bias and not self.check_path:
            # Goal-biased sampling
            if is_start:
                return Node(self.goals[agt_idx].x, self.goals[agt_idx].y)
            else:
                return Node(self.starts[agt_idx].x, self.starts[agt_idx].y)

        else:
            # Uniform random sampling
            return Node(
                np.random.uniform(self.space[0][0], self.space[0][1]),
                np.random.uniform(self.space[1][0], self.space[1][1])
            )

    def nearest_node(self, tree, node):
        return min(tree, key=lambda n: self.distance(n, node))

    def nearest_node_off(self, tree, node):
        return min([i for i in tree], key=lambda n: self.distance(n, node))

    def steer(self, from_node, to_node):
        dist = self.distance(from_node, to_node)
        if dist <= self.step_size:
            return to_node
        theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.step_size * np.cos(theta)
        new_y = from_node.y + self.step_size * np.sin(theta)

        return Node(new_x, new_y)

    def is_collision(self, sp, ep, agt_idx, start_time, end_time):
        if self.is_collision_w_obs([sp.x, sp.y],[ep.x, ep.y]):
            return True

        if self.check_path and self.is_collision_w_path(sp, ep, agt_idx, start_time, end_time):
            # plt.figure(figsize=(20, 20))
            # plt.xlim(self.space[0])
            # plt.ylim(self.space[1])
            # plt.plot([sp.x, ep.x], [sp.y, ep.y], color='red')
            # start_time = sp.cost
            # end_time = ep.cost
            # print(start_time, end_time)
            # for path in self.min_paths:
            #     for node in path[1:]:
            #         node_et = node.cost
            #         node_st = node_et - node.edge
            #         if node.inGoal and node_et < end_time:
            #             plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color="blue")
            #         if (node_et > start_time and node_et < end_time
            #         or node_st > start_time and node_st < end_time
            #         or node_st < start_time and node_et > end_time):
            #             plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color="blue")
                        
            # plt.show()
            return True

        return False

    def is_collision_w_path(self, sp, ep, agt_idx, start_time, end_time):
        buffer = AGT_R
        curr_pos = [sp.x, sp.y]
        parent_pos = [ep.x, ep.y]

        # calc box
        vec = curr_pos[0] - parent_pos[0], curr_pos[1] - parent_pos[1]
        dist = np.sqrt(vec[0] * vec[0] + vec[1] * vec[1])

        vec_h = [vec[0] / dist, vec[1] / dist]
        vec_v = [-vec_h[1], vec_h[0]]
        gap = AGT_R * 2 + buffer

        front = [curr_pos[0] + vec_h[0] * gap, curr_pos[1] + vec_h[1] * gap]
        rear = [parent_pos[0] - vec_h[0] * gap, parent_pos[1] - vec_h[1] * gap]
        fr = [front[0] + vec_v[0] * gap, front[1] + vec_v[1] * gap]
        fl = [front[0] - vec_v[0] * gap, front[1] - vec_v[1] * gap]
        rl = [rear[0] - vec_v[0] * gap, rear[1] - vec_v[1] * gap]
        rr = [rear[0] + vec_v[0] * gap, rear[1] + vec_v[1] * gap]
        box = [fr, fl, rl, rr]

        result = False

        for i, path in enumerate(self.min_paths):
            if i == agt_idx: continue

            for node in path[1:]:
                node_et = node.cost
                node_st = node_et - node.edge
                n_pos = [node.x, node.y]

                # check goal point collision
                if node.inGoal and node_et < end_time:
                    in_box = True
                    for i in range(4):
                        p1 = box[i]
                        p2 = box[(i + 1) % 4]

                        in_box = ccw(p2, p1, n_pos) and in_box

                    if in_box:
                        result = True
                        break

                # check when path cross the box
                if (node_et >= start_time and node_et <= end_time
                    or node_st >= start_time and node_st <= end_time
                    or node_st <= start_time and node_et >= end_time):
                    np_pos = [node.parent.x, node.parent.y]
                    
                    in_box = True
                    for i in range(4):
                        p1 = box[i]
                        p2 = box[(i + 1) % 4]

                        if intersect(p1, p2, n_pos, np_pos):
                            result = True
                        else:
                            in_box = ccw(p2, p1, n_pos) and in_box
                            in_box = ccw(p2, p1, np_pos) and in_box

                    if in_box:
                        result = True

                if result:
                    break

        return False

    def is_collision_w_obs(self, sp, ep):
        for cx, cy, hx, hy in self.obs:
            x_edge = hx + AGT_R
            y_edge = hy + AGT_R
            box = [[cx+x_edge, cy+y_edge], [cx+x_edge, cy-y_edge], [cx-x_edge, cy-y_edge], [cx-x_edge, cy+y_edge]]
            
            for i in range(4):
                p1 = box[i]
                p2 = box[(i + 1) % 4]

                if (intersect(p1, p2, sp, ep)):
                    return True

            if in_box(box[0], box[1], box[2], box[3], sp, ep):
                return True

        return False

    def get_nearby_nodes(self, tree, node):
        return [n for n in tree if self.distance(n, node) <= self.neighbor_radius]

    def rewire(self, tree, new_node):
        nearby_nodes = self.get_nearby_nodes(tree, new_node)
        for near_node in nearby_nodes:
            if near_node == new_node: continue
            potential_edge = self.distance(new_node, near_node)
            potential_cost = new_node.cost + potential_edge
            if potential_cost < near_node.cost:
                near_node.parent = new_node
                near_node.cost = potential_cost
                near_node.edge = potential_edge

    def connect_rrt_star(self, tree, node, agt_idx, is_start):
        nearby_nodes = self.get_nearby_nodes(tree, node)

        min_near_node = self.nearest_node(tree, node)
        min_dist = self.distance(min_near_node, node)

        if min_dist == 0:
            return None, False

        for near in nearby_nodes:
            dist = self.distance(near, node)
            if dist <= self.step_size * 2 and dist < min_dist:
                min_near_node = near
                min_dist = dist

        new_node = self.steer(min_near_node, node)
        start_time = min_near_node.cost
        edge = self.distance(min_near_node, new_node)
        cost = start_time + edge
        if not self.is_collision(min_near_node, new_node, agt_idx, min_near_node.cost, cost):
            tree.append(new_node)
            new_node.parent = min_near_node
            new_node.edge = edge
            new_node.cost = cost

            is_inGoal = False
            if is_start and new_node.x == self.goals[agt_idx].x and new_node.y == self.goals[agt_idx].y:
                new_node.inGoal = True
                is_inGoal = True

            if (not is_start) and (new_node.x == self.starts[agt_idx].x and new_node.y == self.starts[agt_idx].y):
                is_inGoal = True

            return new_node, is_inGoal

        return None, False

    def path_to_root(self, node):
        path = []
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]

    def node_path_list(self, start_node, goal_node):
        path = []
        cost = 0
        
        if start_node:
            cost += start_node.cost

            if goal_node:
                next_edge = self.distance(start_node, goal_node)
                cost += goal_node.cost + next_edge
        else:
            new_node = Node(goal_node.x, goal_node.y)
            next_edge = goal_node.edge
            path.append(new_node)
            cost = goal_node.cost
            goal_node = goal_node.parent

        while start_node:
            path.append(start_node)
            start_node = start_node.parent

        path = path[::-1]
        while goal_node:
            new_node = Node(goal_node.x, goal_node.y)
            new_node.cost = path[-1].cost + next_edge
            new_node.edge = next_edge
            new_node.inGoal = goal_node.inGoal
            new_node.parent = path[-1]
            next_edge = new_node.cost + goal_node.edge
            path.append(new_node)
            goal_node = goal_node.parent

        return path, cost

    def path_connect_true(self, node):
        while node:
            node.connected = True
            node = node.parent

    def count_connect_true(self, node):
        num = 0
        total = 0
        while node:
            total += 1
            if node.connected: num += 1
            node = node.parent
        return num / total

    def trees_connected(self, node1, node2):
        return self.distance(node1, node2) <= self.step_size

    def build(self, run_time, viz=False):
        st = time.time()
        while time.time() - st < run_time:
            check_sol = False
            for agt_idx in range(self.num_agt):
                # Extend tree_start toward a random node using RRT* logic
                rand_node = self.sample_point(is_start=True, agt_idx=agt_idx)
                new_start_node, is_compelete_st = self.connect_rrt_star(self.tree_starts[agt_idx], rand_node, agt_idx, True)

                # Extend tree_goal toward another random node using RRT* logic
                rand_node = self.sample_point(is_start=False, agt_idx=agt_idx)
                new_goal_node, is_compelete_gl = self.connect_rrt_star(self.tree_goals[agt_idx], rand_node, agt_idx, False)

                if is_compelete_st and self.count_connect_true(new_start_node) < self.similarities:
                    self.path_connect_true(new_start_node)
                    uni_path, cost = self.node_path_list(new_start_node, None)
                    self.solutions[agt_idx][cost] = uni_path

                    if not self.check_path:
                        check_sol = not any(not d for d in self.solutions)
                    else:
                        check_sol = True

                # Check if the trees are connected
                elif new_start_node is not None:
                    goal_nearest = self.nearest_node_off(self.tree_goals[agt_idx], new_start_node)
                    edge = self.distance(goal_nearest, new_start_node)
                    if goal_nearest and edge <= self.step_size:
                        if (self.count_connect_true(new_start_node) + self.count_connect_true(goal_nearest) < self.similarities
                                and not self.is_collision(new_start_node, goal_nearest, agt_idx, new_start_node.cost, new_start_node.cost + edge)):
                            self.path_connect_true(new_start_node)
                            self.path_connect_true(goal_nearest)
                            uni_path, cost = self.node_path_list(new_start_node, goal_nearest)

                            # add solution
                            self.solutions[agt_idx][cost] = uni_path
                            
                            if not self.check_path:
                                check_sol = not any(not d for d in self.solutions)
                            else:
                                check_sol = True

                            # self.visualize([path_start])
                            # self.visualize([path_goal[::-1]])
                            # self.visualize([path_start + path_goal[::-1]])
                            # self.visualize(path_nodes=[uni_path])
                            # self.visualize(is_tree_viz=agt_idx, path_nodes=[uni_path])
                            # return
                if is_compelete_gl and self.count_connect_true(new_goal_node) < self.similarities:
                    self.path_connect_true(new_goal_node)
                    uni_path, cost = self.node_path_list(None, new_goal_node)
                    self.solutions[agt_idx][cost] = uni_path

                    if not self.check_path:
                        check_sol = not any(not d for d in self.solutions)
                    else:
                        check_sol = True

                elif new_goal_node is not None:
                    start_nearest = self.nearest_node_off(self.tree_starts[agt_idx], new_goal_node)
                    edge = self.distance(start_nearest, new_goal_node)
                    if start_nearest and edge <= self.step_size:
                        if (self.count_connect_true(start_nearest) + self.count_connect_true(new_goal_node) < self.similarities
                                and not self.is_collision(start_nearest, new_goal_node, agt_idx, start_nearest.cost, start_nearest.cost + edge)):
                            self.path_connect_true(start_nearest)
                            self.path_connect_true(new_goal_node)
                            uni_path, cost = self.node_path_list(start_nearest, new_goal_node)

                            # add solution
                            self.solutions[agt_idx][cost] = uni_path
                            
                            if not self.check_path:
                                check_sol = not any(not d for d in self.solutions)

                            # self.visualize([path_start])
                            # self.visualize([path_goal[::-1]])
                            # self.visualize([path_start + path_goal[::-1]])
                            # self.visualize(path_nodes=[uni_path])
                            # self.visualize(is_tree_viz=agt_idx, path_nodes=[uni_path])
                            # return

            # check for soultion
            if check_sol:
                combinations = list(itertools.product(*self.solutions))
                comb_ordered = sorted(combinations, key=lambda x: (max(x), x))
                for temp_sol in comb_ordered:
                    path_list = []
                    for i, key in enumerate(temp_sol):
                        path_list.append(self.solutions[i][key])

                    is_conflict, dependency = self.check_path_collision(path_list)
                    if not is_conflict:
                        new_paths = self.get_path(path_list)
                        self.run_time = time.time()- st
                        print("plan done")
                        return new_paths, max(temp_sol)

                    # else:
                    #     # check for best combo??
                    #     free_set = [[] for i in range(self.num_agt)]
                    #     min_cost = float('inf')
                    #     for agt_idx, c_list in dependency.items():
                    #         if not c_list:
                    #             free_set[agt_idx] = self.solutions[agt_idx][temp_sol[agt_idx]]
                    #             min_cost = min(min_cost, temp_sol[agt_idx])
                    #         for agt_idx2, c_list2 in dependency.items():
                    #             if agt_idx2 == agt_idx: continue

                    #             if not agt_idx in c_list2:
                    #                 free_set[agt_idx] = self.solutions[agt_idx][temp_sol[agt_idx]]
                    #                 min_cost = min(min_cost, temp_sol[agt_idx])

                        
                    #     if min_cost < cost:
                    #         min_paths = free_set

                    
                # failed to find solution. Set min path to avoid
                for i in range(self.num_agt):
                    min_key = min(self.solutions[i])
                    self.min_paths[i] = self.solutions[i][min_key]
                self.check_path = True
                # print(time.time()- st)
                print("init plan failed")


        # if viz:
        #     for agt_idx in range(self.num_agt):
        #         self.visualize(path_sol=self.solutions[agt_idx])
            
    def convert_path_nodes(self, start_node, goal_node):
        parent = start_node
        while goal_node:
            new_node = goal_node.copy()
            new_node.parent = parent

            parent = goal_node
            goal_node = goal_node.parent

        return parent

    def visualize(self, path_list=[], path_sol=[], path_nodes=[], is_tree_viz=-1):
        plt.figure(figsize=(20, 20))

        if is_tree_viz > -1:
            for node in self.tree_starts[is_tree_viz]:
                if node.parent:
                    plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color="blue", linewidth=1)
                    plt.plot([node.x, node.parent.x], [node.y, node.parent.y], ".", color="blue", linewidth=1)

            for node in self.tree_goals[is_tree_viz]:
                if node.parent:
                    plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color="red", linewidth=1)
                    plt.plot([node.x, node.parent.x], [node.y, node.parent.y], ".", color="red", linewidth=1)

        # Plot paths
        if path_list:
            for path in path_list:
                px, py = zip(*path)
                plt.plot(px, py, linewidth=1)

            # Plot start and goal
            plt.plot(px[0], py[0], "ro", label="Start")
            plt.plot(px[-1], py[-1], "bo", label="Goal")

        if path_sol:
            for path in path_sol.values():
                px, py = zip(*[[i.x, i.y] for i in path])
                plt.plot(px, py, linewidth=1)

            # Plot start and goal
            plt.plot(px[0], py[0], "ro", label="Start")
            plt.plot(px[-1], py[-1], "bo", label="Goal")

        if path_nodes:
            for path in path_nodes:
                px, py = zip(*[[i.x, i.y] for i in path])
                plt.plot(px, py, linewidth=2)

            # Plot start and goal
            plt.plot(px[0], py[0], "ro", label="Start")
            plt.plot(px[-1], py[-1], "bo", label="Goal")

        for cx, cy, hx, hy in self.obs:
            # print(cx, cy, hx, hy)
            temp_obs = patches.Rectangle((cx - hx, cy - hy), hx * 2, hy * 2, facecolor = 'red')
            plt.gca().add_patch(temp_obs)

        plt.gca().set_xlim([-5, 5])
        plt.gca().set_ylim([-5, 5])
        # plt.legend()
        plt.grid()
        plt.show()

    def get_path(self, path_list):
        new_paths = []
        for path in path_list:
            new_path = []
            for node in path:
                new_path.append([node.x, node.y])
            new_paths.append(new_path)

        return new_paths
    
    def check_path_collision(self, path_list):
        num_agt = len(path_list)
        conflict = {}
        dependency = {i:set() for i in range(num_agt)}
        total_conflict = False
        for agt_idx in range(num_agt):
            
            # node to check
            for checker in path_list[agt_idx][1:]:
                sp = checker.parent
                ep = checker
                start_time = sp.cost
                end_time = ep.cost
                box = make_box(sp, ep, 0.2)
                is_conflict = False

                for agt2_idx in range(num_agt):
                    if agt_idx == agt2_idx: continue

                    for node in path_list[agt2_idx][1:]:
                        node_et = node.cost
                        node_st = node_et - node.edge
                        n_pos = [node.x, node.y]

                        # check goal point collision
                        if node.inGoal and node_et < end_time:
                            in_box = True
                            for i in range(4):
                                p1 = box[i]
                                p2 = box[(i + 1) % 4]

                                in_box = ccw(p2, p1, n_pos) and in_box

                            if in_box:
                                # return True, None
                                if not is_conflict:
                                    conflict[checker] = [node]
                                else:
                                    conflict[checker].append(node)
                                is_conflict = True
                                dependency[agt_idx].add(agt2_idx)

                            # plt.figure(figsize=(20,20))
                            # plt.title("goal pos: " + str(in_box) + str(agt_idx))
                            # for i in range(4):
                            #     p1 = box[i]
                            #     p2 = box[(i + 1) % 4]
                            #     plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='black')
                            # plt.plot(n_pos[0], n_pos[1], "o", color="r")
                            # plt.plot([sp.x, ep.x], [sp.y, ep.y], color='blue')
                            # plt.xlim(-5, 5)
                            # plt.ylim(-5, 5)
                            # plt.show()

                        # check when path cross the box
                        if (node_et >= start_time and node_et <= end_time
                            or node_st >= start_time and node_st <= end_time
                            or node_st <= start_time and node_et >= end_time):
                            np_pos = [node.parent.x, node.parent.y]
                            
                            in_box = True
                            for i in range(4):
                                p1 = box[i]
                                p2 = box[(i + 1) % 4]

                                if intersect(p1, p2, n_pos, np_pos):
                                    # return True, None
                                    if not is_conflict:
                                        conflict[checker] = [node]
                                    else:
                                        conflict[checker].append(node)
                                    is_conflict = True
                                    dependency[agt_idx].add(agt2_idx)
                                else:
                                    in_box = ccw(p2, p1, n_pos) and in_box
                                    in_box = ccw(p2, p1, np_pos) and in_box

                            if in_box:
                                # return True, None
                                if not is_conflict:
                                    conflict[checker] = [node]
                                else:
                                    conflict[checker].append(node)
                                is_conflict = True
                                dependency[agt_idx].add(agt2_idx)

                            # plt.figure(figsize=(20,20))
                            # plt.title(str(in_box) + str(agt_idx))
                            # for i in range(4):
                            #     p1 = box[i]
                            #     p2 = box[(i + 1) % 4]
                            #     plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='black')
                            # plt.plot(n_pos[0], n_pos[1], "o", color="r")
                            # plt.plot(np_pos[0], np_pos[1], "o", color="r")
                            # plt.plot([n_pos[0], np_pos[0]], [n_pos[1], np_pos[1]], color='r')
                            # plt.plot([sp.x, ep.x], [sp.y, ep.y], color='blue')
                            # plt.xlim(-5, 5)
                            # plt.ylim(-5, 5)
                            # plt.show()
                if is_conflict:
                    total_conflict = True

        return total_conflict, dependency


def ccw(A,B,C):
    return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def in_box(A,B,C,D,P1,P2):
    is_pass = True
    is_pass = is_pass and ccw(B,A,P1)
    is_pass = is_pass and ccw(B,A,P2)
    is_pass = is_pass and ccw(C,B,P1)
    is_pass = is_pass and ccw(C,B,P2)
    is_pass = is_pass and ccw(D,C,P1)
    is_pass = is_pass and ccw(D,C,P2)
    is_pass = is_pass and ccw(A,D,P1)
    is_pass = is_pass and ccw(A,D,P2)

    return is_pass

def make_box(sp, ep, buffer):
    curr_pos = [sp.x, sp.y]
    parent_pos = [ep.x, ep.y]

    # calc box
    vec = curr_pos[0] - parent_pos[0], curr_pos[1] - parent_pos[1]
    dist = np.sqrt(vec[0] * vec[0] + vec[1] * vec[1])
    vec_h = [vec[0] / dist, vec[1] / dist]
    vec_v = [-vec_h[1], vec_h[0]]
    gap = AGT_R * 2 + buffer

    front = [curr_pos[0] + vec_h[0] * gap, curr_pos[1] + vec_h[1] * gap]
    rear = [parent_pos[0] - vec_h[0] * gap, parent_pos[1] - vec_h[1] * gap]
    fr = [front[0] + vec_v[0] * gap, front[1] + vec_v[1] * gap]
    fl = [front[0] - vec_v[0] * gap, front[1] - vec_v[1] * gap]
    rl = [rear[0] - vec_v[0] * gap, rear[1] - vec_v[1] * gap]
    rr = [rear[0] + vec_v[0] * gap, rear[1] + vec_v[1] * gap]
    box = [fr, fl, rl, rr]

    return box

# Example Usage
if __name__ == "__main__":
    case_id = int(sys.argv[1])
    space, env_id, start_list, goal_list = get_config(case_id)
    center_points, obs_points = create_env(env_id)
    AGT_R = 0.2

    rrt = RRTConnect(space, start_list, goal_list, obs=center_points, step_size=0.3, neighbor_radius=0.7, similarities=0.8)
    path_list, total_cost = rrt.build(30, viz=True)

    print("TOTAL TIME:", rrt.run_time)
    print("TOTAL COST:", total_cost)

    file_name = "test_gif/rrt*_connect_" + str(case_id) + "_w_" + str(round(rrt.run_time, 3)) + "s" + ".gif"
    animate_motion(path_list, space, file_name, obs=center_points)
