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
NODE_BUDGET = 5000# vary [500,1000,3000, 5000]
STEP_SIZE = 60# vary[15, 30, 45, 60]
GOAL_RADIUS = 20
REWIRE_RADIUS = 50
OBSTACLE_THRESHOLD = 100

#NUMBER OF EXPERIMENTAL RUNS FOR METRICS
NUM_RUNS = 20


class Node:
    def __init__(self, x, y, parent=None):
        self.x = int(x)
        self.y = int(y)
        self.parent = parent #previous node
        self.cost = 0.0      #cost from START to this node


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


def near_nodes(nodes, new_node):
    #LOOK FOR NEIGHBOR NODES INSIDE THE REWIRE RADIUS
    return [node for node in nodes if distance(node, new_node) <= REWIRE_RADIUS]


def steer(from_node, to_node):
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    d = math.hypot(dx, dy)

    if d == 0:
        return None

  
    else:
        theta = math.atan2(dy, dx)
        new_x = from_node.x + STEP_SIZE * math.cos(theta)
        new_y = from_node.y + STEP_SIZE * math.sin(theta)
        new_node = Node(new_x, new_y, from_node)

    new_node.cost = from_node.cost + distance(from_node, new_node)
    return new_node


def choose_best_parent(img, near_list, nearest, new_node):
    #RRT* CHOOSES THE PARENT THAT GIVES THE LOWEST COST
    best_parent = nearest
    best_cost = nearest.cost + distance(nearest, new_node)

    for node in near_list:
        if collision_free(img, node, new_node):
            candidate_cost = node.cost + distance(node, new_node)

            if candidate_cost < best_cost:
                best_parent = node
                best_cost = candidate_cost

    new_node.parent = best_parent
    new_node.cost = best_cost

    return new_node


def rewire(img, nodes, near_list, new_node):
    #RRT* REWIRE STEP: TRY TO IMPROVE THE COST OF NEARBY NODES
    for node in near_list:
        new_cost = new_node.cost + distance(new_node, node)

        if new_cost < node.cost:
            if collision_free(img, new_node, node):
                node.parent = new_node
                node.cost = new_cost


def build_path(goal_node):
    path = []
    node = goal_node

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


def rrt_star(img):
    h, w = img.shape

    start_node = Node(*START)
    goal_node = Node(*GOAL)

    if not is_free(img, start_node.x, start_node.y):
        print("ERROR: START is located at an obstacle")
        return [], [], False, None, None, [], []

    if not is_free(img, goal_node.x, goal_node.y):
        print("ERROR: GOAL is located at an obstacle")
        return [], [], False, None, None, [], []

    nodes = [start_node]

    best_goal_node = None
    best_path = []
    best_cost = float("inf")

    convergence_nodes = []
    convergence_costs = []

    start_time = time.time()
    k = 0 #iteration counter

    while len(nodes) < NODE_BUDGET:
        k += 1

        if random.random() < 0.1:
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

        near_list = near_nodes(nodes, new_node)

        new_node = choose_best_parent(img, near_list, nearest, new_node)

        nodes.append(new_node)

        rewire(img, nodes, near_list, new_node)

        #CHECK IF THIS NEW NODE CAN CONNECT TO THE GOAL
        if distance(new_node, goal_node) < GOAL_RADIUS:
            if collision_free(img, new_node, goal_node):
                candidate_goal = Node(goal_node.x, goal_node.y, new_node)
                candidate_goal.cost = new_node.cost + distance(new_node, candidate_goal)

                if candidate_goal.cost < best_cost:
                    best_goal_node = candidate_goal
                    best_cost = candidate_goal.cost
                    best_path = build_path(best_goal_node)

        #SAVE CONVERGENCE HISTORY
        if best_goal_node is not None:
            convergence_nodes.append(k)#in reality we go through iterations not nodes
            convergence_costs.append(best_cost)

    elapsed_time = time.time() - start_time

    if best_goal_node is not None:
        print(f"Best path found with cost {best_cost:.3f}")
        return nodes, best_path, True, elapsed_time, k, convergence_nodes, convergence_costs

    print("Cannot find a path.")
    return nodes, [], False, elapsed_time, k, convergence_nodes, convergence_costs

def compute_convergence_rate(nodes, costs):
    if len(nodes) < 2:
        print("Not enough data to compute convergence rate.")
        return

    nodes = np.array(nodes)
    costs = np.array(costs)

    # Absolute value of changing
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
                linewidth=3
            )

    plt.scatter(START[0], START[1], s=180,color='green', label="Start")
    plt.scatter(GOAL[0], GOAL[1], s=180,color='purple', label="Goal")

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        plt.plot(xs, ys,color='red', linewidth=5, label="Final Path")

    #plt.title(f"RRT* Path Planning with Node Budget = {param}")
    plt.title(f"RRT* Path Planning with Step-Size = {param}")
    plt.xlabel("x [pixels]")
    plt.ylabel("y [pixels]")
    plt.grid(False)
    plt.legend()
    plt.show()


def plot_convergence(convergence_nodes, convergence_costs):
    if len(convergence_nodes) == 0:
        print("No convergence plot available because no path was found.")
        return

    plt.figure(figsize=(8, 5))
    plt.plot(convergence_nodes, convergence_costs, linewidth=2)
    plt.title("RRT* Convergence Over Iterations")
    plt.xlabel("Number of Iterations")
    plt.ylabel("Best Path Length [pixels]")
    plt.grid(True)
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
    avg_rates = []
    final_rates = []

    best_nodes = []
    best_path = []
    best_length = float("inf")
    best_convergence_nodes = []
    best_convergence_costs = []

    last_nodes = []
    last_path = []
    last_convergence_nodes = []
    last_convergence_costs = []

    for run in range(NUM_RUNS):
        random.seed(run)
        print(f"\nRun {run + 1}/{NUM_RUNS}")

        nodes, path, success, elapsed_time, iter_count, conv_nodes, conv_costs = rrt_star(img)
        if len(conv_nodes) > 1:
        	rates = np.abs(np.diff(conv_costs) / np.diff(conv_nodes)) # the true  is that it goes through iterations

        	avg_rates.append(np.mean(rates)*100)
        	final_rates.append(rates[-1]*100)

        success_flags.append(1 if success else 0)
        nodes_used.append(len(nodes))

        last_nodes = nodes
        last_path = path
        last_convergence_nodes = conv_nodes
        last_convergence_costs = conv_costs

        if success:
            L = path_length(path)

            times_to_solution.append(elapsed_time)
            iterations_to_solution.append(iter_count)
            final_path_lengths.append(L)

            if L < best_length:
                best_length = L
                best_nodes = nodes
                best_path = path
                best_convergence_nodes = conv_nodes
                best_convergence_costs = conv_costs

        print(f"Success: {success}")
        print(f"Nodes used: {len(nodes)}")

    success_rate = 100.0 * np.mean(success_flags)

    print("\n========== RRT* METRICS ==========")
    print(f"Success Rate: {success_rate:.2f}%")
    print_metric("Computation Time", times_to_solution, "s")
    print_metric("Iterations Used", iterations_to_solution, "iterations")
    print_metric("Final Path Length", final_path_lengths, "pixels")
    print_metric("Nodes Used", nodes_used, "nodes")
    print_metric("Average Convergence Rate", avg_rates, "cost/ 100 iterations")
    print_metric("Final Convergence Rate", final_rates, "cost/ 100 iterations")
    print("=================================\n")

    if best_path:
        return best_nodes, best_path, best_convergence_nodes, best_convergence_costs
    else:
        return last_nodes, last_path, last_convergence_nodes, last_convergence_costs


def main():
    img = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)

    if img is None:
        raise FileNotFoundError(f"Cannot load the map: {MAP_PATH}")

    nodes, path, convergence_nodes, convergence_costs = run_experiments(img)

    param = STEP_SIZE #Change to STEP_SIZE

    plot_result(img, nodes, path, param)
    plot_convergence(convergence_nodes, convergence_costs)


if __name__ == "__main__":
    main()
