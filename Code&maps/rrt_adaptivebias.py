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
    START = (25, 25)      # x, y on pixels
    GOAL = (475, 475)     # x, y on pixels

elif choice == "2":
    map_name = "map_b.png"
    START = (250, 250)    # x, y on pixels
    GOAL = (750, 750)     # x, y on pixels

elif choice == "3":
    map_name = "map_c.png"
    START = (500, 500)    # x, y on pixels
    GOAL = (2500, 2500)   # x, y on pixels

else:
    raise ValueError("Invalid selection")

MAP_PATH = os.path.join(MAPS_DIR, map_name)

#PARAMETERS
NODE_BUDGET = 5000  # vary [500,1000,3000, 5000]
STEP_SIZE = 60 # vary[15, 30, 45, 60]
GOAL_RADIUS = 20
OBSTACLE_THRESHOLD = 100
#ADAPTIVE GOAL BIAS PARAMETERS
GOAL_BIAS_MIN = 0.05
GOAL_BIAS_MAX = 0.60
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

    if x < 0 or x >= w or y < 0 or y >= h: #check if it is inside the frame
        return False

    return img[y, x] > OBSTACLE_THRESHOLD # Check if the pixel goes over 100 it is free


def collision_free(img, n1, n2):
    steps = int(distance(n1, n2))

    if steps == 0:
        return is_free(img, n1.x, n1.y)

    for i in range(steps + 1): #check intermidiate points in order to check if it is compleetly free
        t = i / steps
        x = int(n1.x + t * (n2.x - n1.x))
        y = int(n1.y + t * (n2.y - n1.y))

        if not is_free(img, x, y):
            return False

    return True


def nearest_node(nodes, random_node):
    return min(nodes, key=lambda node: distance(node, random_node))# look for the node most closest to the random node


def steer(from_node, to_node): #move the desired distance stepsize in the direction
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    d = math.hypot(dx, dy)

    if d == 0:
        return None

    theta = math.atan2(dy, dx)
    new_x = from_node.x + STEP_SIZE * math.cos(theta)
    new_y = from_node.y + STEP_SIZE * math.sin(theta)
    new_node = Node(new_x, new_y, from_node)
    
    return new_node


def build_path(goal_node): #BUILTS THE PATH BASED ON THE ROOT THAT FOUND THE GOAL
    path = []
    node = goal_node

    while node is not None:
        path.append((node.x, node.y))
        node = node.parent

    path.reverse()
    return path


def path_length(path):
    #COMPUTE THE EUCLIDEAN LENGTH OF THE RETURNED PATH
    if len(path) < 2:
        return 0.0

    total = 0.0

    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        total += math.hypot(x2 - x1, y2 - y1)

    return total


def rrt(img):
    h, w = img.shape

    start_node = Node(*START)
    goal_node = Node(*GOAL)

    if not is_free(img, start_node.x, start_node.y):
        print("ERROR: START is located at an obstacle")
        return [], [], False, None, None

    if not is_free(img, goal_node.x, goal_node.y):
        print("ERROR: GOAL is located at an obstacle")
        return [], [], False, None, None

    nodes = [start_node]

    start_time = time.time()
    k = 0  # iteration counter

    # 
    while len(nodes) < NODE_BUDGET:
        k += 1

        #ADAPTIVE GOAL BIAS WITH TIME / ITERATIONS
        progress = len(nodes) / NODE_BUDGET
        goal_bias = GOAL_BIAS_MIN + (GOAL_BIAS_MAX - GOAL_BIAS_MIN) * progress

        if random.random() < goal_bias:
            rand_node = goal_node
        else:
            rand_node = Node(
                random.randint(0, w - 1),
                random.randint(0, h - 1)
             )
        nearest = nearest_node(nodes, rand_node)
        new_node = steer(nearest, rand_node)

        if new_node is None:
            continue

        if not is_free(img, new_node.x, new_node.y):
            continue

        if not collision_free(img, nearest, new_node):
            continue

        nodes.append(new_node)

        if distance(new_node, goal_node) < GOAL_RADIUS:
            if collision_free(img, new_node, goal_node):
                goal_node.parent = new_node
                nodes.append(goal_node)

                elapsed_time = time.time() - start_time
                path = build_path(goal_node)

                print(f"Path found with {len(nodes)} nodes after {k} iterations")
                return nodes, path, True, elapsed_time, k

    elapsed_time = time.time() - start_time

    print("Cannot find a path.")
    return nodes, [], False, elapsed_time, k

def compute_convergence_rate(nodes, costs):
    if len(nodes) < 2:
        print("Not enough data to compute convergence rate.")
        return

    nodes = np.array(nodes)
    costs = np.array(costs)

    #  absolute value of the rate
    rates = np.abs(np.diff(costs) / np.diff(nodes))

    mean_rate = np.mean(rates)

    print(f"Average convergence rate (abs): {mean_rate:.6f} (cost change per node)")
    print(f"Final convergence rate (abs): {rates[-1]:.6f}")

    return rates
def plot_result(img, nodes, path, param):
    plt.figure(figsize=(9, 9))
    plt.imshow(img, cmap="gray", origin="lower")

    for node in nodes:
        if node.parent is not None:
            plt.plot(
                [node.x, node.parent.x],
                [node.y, node.parent.y],
                color='blue',
                linewidth=3#0.5
            )

    plt.scatter(START[0], START[1], s=180,color='green', label="Start")
    plt.scatter(GOAL[0], GOAL[1], s=180,color='purple', label="Goal")

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        plt.plot(xs, ys,color='red', linewidth=5, label="Final Path")

    #plt.title(f"RRT Path Planning with Node Budget = {param}")
    plt.title(f"RRT Path Planning with step-size = {param}")
    plt.xlabel("x [pixels]")
    plt.ylabel("y [pixels]")
    plt.grid(False)
    plt.legend()
    plt.show()


def print_metric(name, values, unit=""):
    #PRINT MEAN ± STANDARD DEVIATION
    if len(values) == 0:
        print(f"{name}: N/A")
        return

    values = np.array(values, dtype=float)
    mean_value = np.mean(values)
    std_value = np.std(values)

    print(f"{name}: {mean_value:.3f} ± {std_value:.3f} {unit}")


def run_experiments(img):
    #RUN MULTIPLE RRT TRIALS TO COMPUTE METRICS
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

        nodes, path, success, elapsed_time, iter_count = rrt(img)

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

    print("\n========== RRT METRICS ==========")
    print(f"Success Rate: {success_rate:.2f}%")
    print_metric("Time to First Solution", times_to_solution, "s")
    print_metric("Iterations to First Solution", iterations_to_solution, "iterations")
    print_metric("Final Path Length", final_path_lengths, "pixels")
    print_metric("Nodes Used", nodes_used, "nodes")
    print("=================================\n")

    if best_path:
        return best_nodes, best_path
    else:
        return last_nodes, last_path


def main():
    img = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)

    if img is None:
        raise FileNotFoundError(f"Cannot load the map: {MAP_PATH}")

    nodes, path = run_experiments(img)

    param =STEP_SIZE
    plot_result(img, nodes, path, param)


if __name__ == "__main__":
    main()
