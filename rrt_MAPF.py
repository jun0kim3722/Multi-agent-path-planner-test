import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import sys
from main import animate_motion
from test_case_generator import create_env, get_config
import pdb

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0
        self.edge = 0.0
        self.inGoal = False

class RRT:
    def __init__(self, space, start, goal, obs=[], min_paths=[], step_size=1.0, goal_bias=0.2):
        self.space = space
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.tree_start = [self.start]
        self.obs = obs
        self.min_paths = min_paths

    def distance(self, node1, node2):
        return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def sample_point(self):
        if np.random.rand() < self.goal_bias:
            # Goal-biased sampling
            return Node(self.goal.x, self.goal.y)

        else:
            # Uniform random sampling
            return Node(
                np.random.uniform(self.space[0][0], self.space[0][1]),
                np.random.uniform(self.space[1][0], self.space[1][1])
            )

    def nearest_node(self, tree, node):
        return min(tree, key=lambda n: self.distance(n, node))

    def steer(self, from_node, to_node):
        dist = self.distance(from_node, to_node)
        if dist <= self.step_size:
            return to_node
        theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.step_size * np.cos(theta)
        new_y = from_node.y + self.step_size * np.sin(theta)
        return Node(new_x, new_y)

    def connect(self, tree, node):
        nearest = self.nearest_node(tree, node)
        new_node = self.steer(nearest, node)

        new_node.parent = nearest
        new_node.edge = self.distance(nearest, new_node)
        new_node.cost = nearest.cost + new_node.edge
        if not self.is_collision(nearest, new_node):
            tree.append(new_node)
            return new_node
        
        return None

    def path_to_root(self, node):
        path = []
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]
    
    def node_path(self, node):
        path = []
        while node:
            path.append(node)
            node = node.parent
        return path[::-1]

    def trees_connected(self, node1, node2):
        return self.distance(node1, node2) <= self.step_size
    
    def is_collision(self, sp, ep):
        if self.is_collision_w_obs([sp.x, sp.y],[ep.x, ep.y]):
            return True

        if self.min_paths and self.is_collision_w_path(sp, ep):
            return True

        return False
    
    def is_collision_w_path(self, sp, ep):
        buffer = AGT_R
        curr_pos = [sp.x, sp.y]
        parent_pos = [ep.x, ep.y]

        # calc collision box
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

        start_time = sp.cost
        end_time = ep.cost

        for path in self.min_paths:
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
                        return True

                # check when path cross the collision box
                if (node_et >= start_time and node_et <= end_time
                    or node_st >= start_time and node_st <= end_time
                    or node_st <= start_time and node_et >= end_time):
                    np_pos = [node.parent.x, node.parent.y]
                    
                    in_box = True
                    for i in range(4):
                        p1 = box[i]
                        p2 = box[(i + 1) % 4]

                        if intersect(p1, p2, n_pos, np_pos):
                            return True
                        else:
                            in_box = ccw(p2, p1, n_pos) and in_box
                            in_box = ccw(p2, p1, np_pos) and in_box

                    if in_box:
                        return True

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

    def build(self, run_time=0, viz=False):
        while time.time() - st < run_time:
            rand_node = self.sample_point()
            new_start_node = self.connect(self.tree_start, rand_node)

            if new_start_node and new_start_node.x == self.goal.x and new_start_node.y == self.goal.y:
                new_start_node.inGoal = True
                path_nodes = self.node_path(new_start_node)
                if viz:
                    path_list = self.path_to_root(new_start_node)
                    self.visualize([path_list])
                
                return path_nodes, new_start_node.cost

        self.visualize(is_tree_viz=True)
                
        return None, None

    def visualize(self, path_list=[], is_tree_viz=False):
        plt.figure(figsize=(20, 20))
        plt.xlim(self.space[0])
        plt.ylim(self.space[1])

        # Plot the trees
        # for tree in [self.tree_start, self.tree_goal]:
        if is_tree_viz:
            for node in self.tree_start:
                if node.parent:
                    plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color="blue", linewidth=1)
                    plt.plot([node.x, node.parent.x], [node.y, node.parent.y], ".", color="blue", linewidth=1)

        # Plot start and goal
        plt.plot(self.start.x, self.start.y, "ro", label="Start")
        plt.plot(self.goal.x, self.goal.y, "bo", label="Goal")

        # Plot path
        for path in path_list:
            px, py = zip(*path)
            plt.plot(px, py, linewidth=2, label="Path", color="blue")
            plt.plot(px, py, ".", label="Path")

        for path in self.min_paths:
            x_l = []
            y_l = []
            for node in path:
                x_l.append(node.x)
                y_l.append(node.y)

            plt.plot(x_l, y_l, linewidth=2, label="Path", color='black')

        for cx, cy, hx, hy in self.obs:
            print(cx, cy, hx, hy)
            temp_obs = patches.Rectangle((cx - hx, cy - hy), hx * 2, hy * 2, facecolor = 'red')
            plt.gca().add_patch(temp_obs)

        plt.legend()
        plt.grid()
        plt.show()

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

if __name__ == "__main__":
    case_id = int(sys.argv[1])
    space, env_id, start_list, goal_list = get_config(case_id)
    center_points, obs_points = create_env(env_id)
    AGT_R = 0.2
    num_agt = len(start_list)

    total_cost = 0
    st = time.time()
    solution = {}
    while not len(solution) == num_agt:
        min_cost = sys.maxsize
        min_path = []
        min_agt = -1

        for i in range(len(start_list)):
            if i in solution.keys(): continue

            rrt = RRT(space, start_list[i], goal_list[i], obs=center_points, min_paths=[*solution.values()], step_size=0.3, goal_bias=0.2)
            path, cost = rrt.build(3000)

            if cost < min_cost:
                min_cost = cost
                min_path = path
                min_agt = i
                
            if min_cost > total_cost:
                total_cost = min_cost
    
        solution[min_agt] = min_path
        # rrt.visualize()

        print("Sol Added out of", num_agt, ":", min_agt, solution.keys())
    
    total_time = time.time() - st
    print("TOTAL TIME:", total_time)
    print("TOTAL COST:", total_cost)
    rrt = RRT(space, start_list[0], goal_list[0], obs=center_points, step_size=0.3)
    # rrt.visualize()
    
    cord_path = []
    for path in solution.values():
        new_path = []
        for node in path:
            new_path.append([node.x, node.y])

        cord_path.append(new_path)

    # rrt.visualize(path_list=cord_path)

    file_name = "test_gif/rrt_MARF_" + str(case_id) + "_w_" + str(round(total_time, 3)) + "s" + ".gif"
    animate_motion(cord_path, space, file_name, obs=center_points)
