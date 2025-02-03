import numpy as np
import matplotlib.pyplot as plt
import pdb

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.connected = False

class RRTConnect:
    def __init__(self, space, start, goal, step_size=1.0, max_samples=1000, goal_bias=0.0):
        self.space = space
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.step_size = step_size
        self.max_samples = max_samples
        self.goal_bias = goal_bias
        self.tree_start = [self.start]
        self.tree_goal = [self.goal]

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
        tree.append(new_node)
        return new_node

    def path_to_root(self, node):
        path = []
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]

    def trees_connected(self, node1, node2):
        return self.distance(node1, node2) <= self.step_size

    def build(self):
        solution = []
        for _ in range(self.max_samples):
            rand_node = self.sample_point()
            new_start_node = self.connect(self.tree_start, rand_node)

            rand_node = self.sample_point()
            new_goal_node = self.connect(self.tree_goal, rand_node)

            if self.trees_connected(new_start_node, new_goal_node):
                path_start = self.path_to_root(new_start_node)
                path_goal = self.path_to_root(new_goal_node)
                solution.append(path_start + path_goal[::-1])

        return solution

    def visualize(self, path_list=None):
        plt.figure(figsize=(20, 20))
        plt.xlim(self.space[0])
        plt.ylim(self.space[1])

        # Plot the trees
        # for tree in [self.tree_start, self.tree_goal]:
        # for node in self.tree_start:
        #     if node.parent:
        #         plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color="blue", linewidth=1)
        #         plt.plot([node.x, node.parent.x], [node.y, node.parent.y], ".", color="blue", linewidth=1)

        # for node in self.tree_goal:
        #     if node.parent:
        #         plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color="red", linewidth=1)
        #         plt.plot([node.x, node.parent.x], [node.y, node.parent.y], ".", color="red", linewidth=1)

        # Plot start and goal
        plt.plot(self.start.x, self.start.y, "ro", label="Start")
        plt.plot(self.goal.x, self.goal.y, "bo", label="Goal")

        # Plot path
        for path in path_list:
            px, py = zip(*path)
            plt.plot(px, py, linewidth=2, label="Path")
            plt.plot(px, py, ".", label="Path")

        plt.legend()
        plt.grid()
        plt.show()


# Example Usage
if __name__ == "__main__":
    space = [[0, 20], [0, 20]]  # X and Y ranges
    start = [2, 2]
    goal = [18, 18]

    rrt = RRTConnect(space, start, goal, step_size=1.0, max_samples=1500)
    path = rrt.build()
    if path:
        print("Path found!")
        rrt.visualize(path)
    else:
        print("No path found.")
