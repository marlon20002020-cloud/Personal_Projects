import cv2
import random
import math
import matplotlib.pyplot as plt
import os
import time
import numpy as np

#SELECTION OF THE MAP
MAPS_DIR = "/home/marlon/ros2_ws/src/my_maps/maps"

print("Make a selection of the map:")
print("1 → map_a.png")
print("2 → map_b.png")
print("3 → map_c.png")

choice = input("Write the number: ")

if choice == "1":
    map_name = "map_a.png"
    START = (25, 25)
    GOAL = (475, 475)

elif choice == "2":
    map_name = "map_b.png"
    START = (250, 250)
    GOAL = (750, 750)

elif choice == "3":
    map_name = "map_c.png"
    START = (500, 500)
    GOAL = (2500, 2500)

else:
    raise ValueError("Invalid selection")

MAP_PATH = os.path.join(MAPS_DIR, map_name)

#PARAMETERS
NODE_BUDGET = 5000 # vary [500,1000,3000, 5000]
STEP_SIZE = 60 #[15,30,45,60]
OBSTACLE_THRESHOLD = 100

#NUMBER OF EXPERIMENTAL RUNS FOR METRICS
NUM_RUNS = 20


class Node:
    def __init__(self, x, y, parent=None):
        self.x = int(x)
        self.y = int(y)
        self.parent = parent #previous node


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y) #euclidean distance


def is_free(img, x, y):
    h, w = img.shape

    if x < 0 or x >= w or y < 0 or y >= h:
        return False

    return img[y, x] > OBSTACLE_THRESHOLD


def collision_free(img, n1, n2):
    steps = int(distance(n1, n2))

    if steps == 0:
        return is_free(img, n1.x, n1.y)

    for i in range(steps + 1):
        t = i / steps
        x = int(n1.x + t * (n2.x - n1.x))
        y = int(n1.y + t * (n2.y - n1.y))

        if not is_free(img, x, y):
            return False

    return True


def nearest_node(nodes, random_node):
    return min(nodes, key=lambda node: distance(node, random_node))


def steer(from_node, to_node):
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    d = math.hypot(dx, dy)

    if d == 0:
        return None

    theta = math.atan2(dy, dx)

    # Fixed step-size expansion
    new_x = from_node.x + STEP_SIZE * math.cos(theta)
    new_y = from_node.y + STEP_SIZE * math.sin(theta)

    return Node(new_x, new_y, from_node)


def extend(img, tree, target_node):
    #EXTEND ONE TREE TOWARDS A TARGET NODE
    nearest = nearest_node(tree, target_node)
    new_node = steer(nearest, target_node)

    if new_node is None:
        return None, False

    if not is_free(img, new_node.x, new_node.y):
        return None, False

    if not collision_free(img, nearest, new_node):
        return None, False

    tree.append(new_node)

    return new_node, True


def connect(img, tree, target_node):
    #TRY TO CONNECT A TREE TO A TARGET NODE USING REPEATED EXTENSIONS
    last_node = None

    while True:
        new_node, success = extend(img, tree, target_node)

        if not success:
            return last_node, False

        last_node = new_node

        if distance(new_node, target_node) <= STEP_SIZE:
            if collision_free(img, new_node, target_node):
                connected_node = Node(target_node.x, target_node.y, new_node)
                tree.append(connected_node)
                return connected_node, True


def build_path(node):
    path = []

    while node is not None:
        path.append((node.x, node.y))
        node = node.parent

    path.reverse()
    return path


def path_length(path):
    if len(path) < 2:
        return 0.0

    total = 0.0

    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        total += math.hypot(x2 - x1, y2 - y1)

    return total


def build_rrt_connect_path(node_from_start_tree, node_from_goal_tree):
    #PATH FROM START TREE
    path_start = build_path(node_from_start_tree)

    #PATH FROM GOAL TREE
    path_goal = build_path(node_from_goal_tree)

    #path_goal goes from GOAL to connection, so reverse it to go connection -> GOAL
    path_goal.reverse()

    #Avoid duplicating the connection point
    return path_start + path_goal[1:]


def rrt_connect(img):
    h, w = img.shape

    start_node = Node(*START)
    goal_node = Node(*GOAL)

    if not is_free(img, start_node.x, start_node.y):
        print("ERROR: START is located at an obstacle")
        return [], [], False, None, None

    if not is_free(img, goal_node.x, goal_node.y):
        print("ERROR: GOAL is located at an obstacle")
        return [], [], False, None, None

    tree_start = [start_node]
    tree_goal = [goal_node]

    start_time = time.time()
    k = 0

    while len(tree_start) + len(tree_goal) < NODE_BUDGET:
        k += 1

        rand_node = Node(
            random.randint(0, w - 1),
            random.randint(0, h - 1)
        )

        #Grow first tree toward random node
        new_node, success = extend(img, tree_start, rand_node)

        if success:
            #Try to connect the second tree to the new node
            connected_node, connected = connect(img, tree_goal, new_node)

            if connected:
                elapsed_time = time.time() - start_time

                path = build_rrt_connect_path(new_node, connected_node)

                all_nodes = tree_start + tree_goal

                print(f"Path found with {len(all_nodes)} nodes after {k} iterations")
                return all_nodes, path, True, elapsed_time, k

        #Swap trees
        tree_start, tree_goal = tree_goal, tree_start

    elapsed_time = time.time() - start_time

    all_nodes = tree_start + tree_goal

    print("Cannot find a path.")
    return all_nodes, [], False, elapsed_time, k


def plot_result(img, nodes, path, param):
    plt.figure(figsize=(9, 9))
    plt.imshow(img, cmap="gray", origin="lower")

    for node in nodes:
        if node.parent is not None:
            plt.plot(
                [node.x, node.parent.x],
                [node.y, node.parent.y],
                color='blue',
                linewidth=3
            )

    plt.scatter(START[0], START[1], s=180, color='green', label="Start")
    plt.scatter(GOAL[0], GOAL[1], s=180, color='purple', label="Goal")

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        plt.plot(xs, ys, color='red', linewidth=5, label="Final Path")

    plt.title(f"RRT-Connect Path Planning with Step-Size = {param}")
    #plt.title(f"RRT-Connect Path Planning with Node budget = {param}")
    plt.xlabel("x [pixels]")
    plt.ylabel("y [pixels]")
    plt.grid(False)
    plt.legend()
    plt.show()


def print_metric(name, values, unit=""):
    if len(values) == 0:
        print(f"{name}: N/A")
        return

    values = np.array(values, dtype=float)
    mean_value = np.mean(values)
    std_value = np.std(values)

    print(f"{name}: {mean_value:.3f} ± {std_value:.3f} {unit}")


def run_experiments(img):
    success_flags = []
    times_to_solution = []
    iterations_to_solution = []
    final_path_lengths = []
    nodes_used = []

    best_nodes = []
    best_path = []
    best_length = float("inf")

    last_nodes = []
    last_path = []

    for run in range(NUM_RUNS):
        random.seed(run)
        print(f"\nRun {run + 1}/{NUM_RUNS}")

        nodes, path, success, elapsed_time, iter_count = rrt_connect(img)

        success_flags.append(1 if success else 0)
        nodes_used.append(len(nodes))

        last_nodes = nodes
        last_path = path

        if success:
            L = path_length(path)

            times_to_solution.append(elapsed_time)
            iterations_to_solution.append(iter_count)
            final_path_lengths.append(L)

            if L < best_length:
                best_length = L
                best_nodes = nodes
                best_path = path

        print(f"Success: {success}")
        print(f"Nodes used: {len(nodes)}")

    success_rate = 100.0 * np.mean(success_flags)

    print("\n========== RRT-CONNECT METRICS ==========")
    print(f"Success Rate: {success_rate:.2f}%")
    print_metric("Time to First Solution", times_to_solution, "s")
    print_metric("Iterations to First Solution", iterations_to_solution, "iterations")
    print_metric("Final Path Length", final_path_lengths, "pixels")
    print_metric("Nodes Used", nodes_used, "nodes")
    print("=========================================\n")

    if best_path:
        return best_nodes, best_path
    else:
        return last_nodes, last_path


def main():
    img = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)

    if img is None:
        raise FileNotFoundError(f"Cannot load the map: {MAP_PATH}")

    nodes, path = run_experiments(img)

    param = STEP_SIZE #NODE_BUDGET
    plot_result(img, nodes, path, param)


if __name__ == "__main__":
    main()
