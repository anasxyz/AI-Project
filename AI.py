import json
from math import radians, sin, cos, sqrt, atan2
from collections import deque
import time
import tkinter as tk
from tkinter import ttk
import tkinter.messagebox as messagebox
import tkinter.simpledialog as simpledialog

PRICES = [137.1, 142.2, 143.1, 143.3, 144.1, 145.7, 146.5, 147.9, 149.5]

class Problem:
    global goal_node

    def __init__(self, adjacency_matrix, initial_node, goal, location_list):
        self.adjacency_matrix = adjacency_matrix
        self.initial_node = initial_node
        self.goal = goal
        self.location_list = location_list
        global goal_node

    def goal_test(self, state):
        return state.price == self.goal and state.type == "ps"

class Node:
    def __init__(self, state):
        self.state = state

    def __str__(self):
        return f"ID: {self.state.ID}, type: {self.state.type}, latitude: {self.state.latitude}, longitude: {self.state.longitude}, price: {self.state.price}"

class State:
    def __init__(self, ID, type, latitude, longitude, price):
        self.ID = ID
        self.type = type
        self.latitude = latitude
        self.longitude = longitude
        self.price = price

    def __str__(self):
        return f"ID: {self.ID}, type: {self.type}, latitude: {self.latitude}, longitude: {self.longitude}, price: {self.price}"

class Search:
    def __init__(self):
        pass

    # @staticmethod
    # def breadth_first_search(problem):
    #     adjacency_matrix = problem.adjacency_matrix
    #     start_node = problem.initial_node
    #     goal_price = problem.goal

    #     queue = deque([(start_node, [start_node], 0)]) 
    #     visited = set()  # Set to keep track of visited nodes

    #     while queue:
    #         current_node, path, _ = queue.popleft()  
    #         current_state = current_node.state

    #         # Check if the current state satisfies the goal test
    #         if problem.goal_test(current_state):
    #             return current_node, path  

    #         visited.add(current_state.ID)  

    #         for neighbor_index, connected in enumerate(adjacency_matrix[current_state.ID-1]):
    #             if connected == 1:
    #                 neighbor_node = problem.location_list[neighbor_index] 
    #                 neighbor_state = neighbor_node.state

    #                 # Check if the neighbor node has not been visited
    #                 if neighbor_state.ID not in visited:
    #                     new_path = path + [neighbor_node] 
    #                     queue.append((neighbor_node, new_path, 0)) 

    #     return None, [] 
    
    @staticmethod
    def bidirectional_search(problem):
        start_node = problem.initial_node

        forward_queue = deque([(start_node, [start_node])])
        backward_queue = deque([(goal_node, [goal_node])])

        forward_visited = set()
        backward_visited = set()

        intersection_node = None
        min_distance = float('inf')

        while forward_queue and backward_queue:
            current_node_fwd, path_fwd = forward_queue.popleft()
            current_state_fwd = current_node_fwd.state

            forward_visited.add(current_state_fwd.ID)

            if current_state_fwd.ID in backward_visited:
                total_distance = len(path_fwd) - 1 + len(path_bwd) - 1
                if total_distance < min_distance:
                    min_distance = total_distance
                    intersection_node = current_node_fwd

            for neighbor_index, connected in enumerate(problem.adjacency_matrix[current_state_fwd.ID-1]):
                if connected == 1:
                    neighbor_node_fwd = problem.location_list[neighbor_index]
                    neighbor_state_fwd = neighbor_node_fwd.state

                    if neighbor_state_fwd.ID not in forward_visited:
                        new_path_fwd = path_fwd + [neighbor_node_fwd]
                        forward_queue.append((neighbor_node_fwd, new_path_fwd))

            current_node_bwd, path_bwd = backward_queue.popleft()
            current_state_bwd = current_node_bwd.state

            backward_visited.add(current_state_bwd.ID)

            if current_state_bwd.ID in forward_visited:
                total_distance = len(path_fwd) - 1 + len(path_bwd) - 1
                if total_distance < min_distance:
                    min_distance = total_distance
                    intersection_node = current_node_bwd

            for neighbor_index, connected in enumerate(problem.adjacency_matrix[current_state_bwd.ID-1]):
                if connected == 1:
                    neighbor_node_bwd = problem.location_list[neighbor_index]
                    neighbor_state_bwd = neighbor_node_bwd.state

                    if neighbor_state_bwd.ID not in backward_visited:
                        new_path_bwd = path_bwd + [neighbor_node_bwd]
                        backward_queue.append((neighbor_node_bwd, new_path_bwd))

            if intersection_node:
                path_fwd = path_fwd[:-1] 
                path = path_fwd + list(reversed(path_bwd)) 
                return intersection_node, path

        return None, []

    @staticmethod
    def depth_limited_search(problem, depth_limit):
        adjacency_matrix = problem.adjacency_matrix
        start_node = problem.initial_node
        goal_price = problem.goal

        stack = [(start_node, [start_node])]
        visited = set()

        while stack:
            current_node, path = stack.pop() 
            current_state = current_node.state

            if problem.goal_test(current_state):
                return current_node, path

            visited.add(current_state.ID)

            if len(path) <= depth_limit:
                for neighbor_index, connected in enumerate(adjacency_matrix[current_state.ID-1]):
                    if connected == 1:
                        neighbor_node = problem.location_list[neighbor_index] 
                        neighbor_state = neighbor_node.state

                        if neighbor_state.ID not in visited:
                            new_path = path + [neighbor_node] 
                            stack.append((neighbor_node, new_path))

        return None, [] 

    @staticmethod
    def heuristic(node, goal_node):
        lat1 = node.state.latitude
        lon1 = node.state.longitude
        lat2 = goal_node.state.latitude
        lon2 = goal_node.state.longitude

        R = 6371.0

        lat1_rad = radians(lat1)
        lon1_rad = radians(lon1)
        lat2_rad = radians(lat2)
        lon2_rad = radians(lon2)

        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        a = sin(dlat / 2)**2 + cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        distance = R * c

        return distance

    @staticmethod
    def a_star_search(problem):
        start_node = problem.initial_node

        priority_queue = [(0 + Search.heuristic(start_node, goal_node), 0, start_node, [start_node])]
        visited = set()

        while priority_queue:
            _, cost, current_node, path = priority_queue.pop(0)
            current_state = current_node.state

            if problem.goal_test(current_state):
                return current_node, path

            visited.add(current_state.ID)

            for neighbor_index, connected in enumerate(problem.adjacency_matrix[current_state.ID-1]):
                if connected == 1:
                    neighbor_node = problem.location_list[neighbor_index]
                    neighbor_state = neighbor_node.state

                    if neighbor_state.ID not in visited:
                        new_cost = cost + findDistance(neighbor_node, goal_node)
                        new_path = path + [neighbor_node] 
                        priority_queue.append((new_cost + Search.heuristic(neighbor_node, goal_node), new_cost, neighbor_node, new_path))

            priority_queue.sort(key=lambda x: x[0])

        return None, [] 
    
    @staticmethod
    def greedy_best_first_search(problem):
        start_node = problem.initial_node

        priority_queue = [(Search.heuristic(start_node, goal_node), start_node, [start_node])]
        visited = set()

        while priority_queue:
            _, current_node, path = priority_queue.pop(0) 
            current_state = current_node.state

            if problem.goal_test(current_state):
                return current_node, path 

            visited.add(current_state.ID) 

            for neighbor_index, connected in enumerate(problem.adjacency_matrix[current_state.ID-1]):
                if connected == 1:
                    neighbor_node = problem.location_list[neighbor_index] 
                    neighbor_state = neighbor_node.state

                    if neighbor_state.ID not in visited:
                        new_path = path + [neighbor_node] 
                        priority_queue.append((Search.heuristic(neighbor_node, goal_node), neighbor_node, new_path))

            priority_queue.sort(key=lambda x: x[0])

        return None, [] 

def read_json_file(file_path):
        nodes = []

        with open(file_path, 'r') as file:
            data = json.load(file)
            for i in data:
                state = State(i["ID"], i["type"], i["latitude"], i["longitude"], i["fuel_price"])
                nodes.append(Node(state))
                
        return nodes

def createAdjacencyMatrix(location_list, max_distance=125):
    num_locations = len(location_list)
    adjacency_matrix = [[0] * num_locations for _ in range(num_locations)]
    
    for i in range(num_locations):
        for j in range(i+1, num_locations):
            location1 = location_list[i]
            location2 = location_list[j]
            distance = findDistance(location1, location2)
            if distance <= max_distance:
                adjacency_matrix[i][j] = 1
                adjacency_matrix[j][i] = 1
    
    return adjacency_matrix

def scan(node, adjacency_matrix, location_list):
    connected_node_indices = []
    node_index = location_list.index(node)

    for i, connected in enumerate(adjacency_matrix[node_index]):
        if connected:
            connected_node_indices.append(i)

    connected_nodes_list = [location_list[index] for index in connected_node_indices]
    return connected_nodes_list

def findDistance(node1, node2):
    R = 6371.0

    lat1_rad = radians(node1.state.latitude)
    lon1_rad = radians(node1.state.longitude)
    lat2_rad = radians(node2.state.latitude)
    lon2_rad = radians(node2.state.longitude)
    
    dlon = lon2_rad - lon1_rad
    dlat = lat2_rad - lat1_rad
    
    a = sin(dlat / 2)**2 + cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    
    distance = R * c
    return distance

''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

# def nodes_closest_to_price(location_list, custom_price):
#     closest_price_diff = float('inf')
#     closest_price_nodes = []

#     for node in location_list:
#         price_diff = abs(node.state.price - custom_price)
#         if price_diff < closest_price_diff:
#             closest_price_diff = price_diff

#     for node in location_list:
#         if abs(node.state.price - custom_price) == closest_price_diff:
#             if node.state.type == "ps":
#                 closest_price_nodes.append(node)

#     return closest_price_nodes

# def nearest_closest_to_price_node(initial_node, location_list, custom_price):
#     closest_price_diff = float('inf') 
#     closest_price_nodes = []

#     for node in location_list:
#         price_diff = abs(node.state.price - custom_price)
#         if price_diff < closest_price_diff:
#             closest_price_diff = price_diff

#     for node in location_list:
#         if abs(node.state.price - custom_price) == closest_price_diff:
#             if node.state.type == "ps":
#                 closest_price_nodes.append(node)

#     closest_node = None
#     closest_distance = float('inf')

#     for node in closest_price_nodes:
#         distance = findDistance(initial_node, node) 
#         if distance < closest_distance:  
#             closest_distance = distance
#             closest_node = node

#     return closest_node

def nearest_custom_price(location_list, initial_node, custom_price):
    closest_price_diff = float('inf') 
    closest_price_nodes = []

    for node in location_list:
        price_diff = abs(node.state.price - custom_price)
        if price_diff < closest_price_diff:
            closest_price_diff = price_diff

    for node in location_list:
        if abs(node.state.price - custom_price) == closest_price_diff:
            if node.state.type == "ps":
                closest_price_nodes.append(node)

    closest_node = None
    closest_distance = float('inf')

    for node in closest_price_nodes:
        distance = findDistance(initial_node, node) 
        if distance < closest_distance:  
            closest_distance = distance
            closest_node = node

    return closest_node

''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

# def nodes_with_lowest_price(location_list):
#     lowest_price = float('inf')  
#     lowest_price_nodes = []

#     for node in location_list:
#         if node.state.price < lowest_price:
#             lowest_price = node.state.price

#     for node in location_list:
#         if node.state.price == lowest_price:
#             if node.state.type == "ps":
#                 lowest_price_nodes.append(node)

#     return lowest_price_nodes

def nearest_lowest_price_node(initial_node, location_list):
    lowest_price = float('inf') 
    lowest_price_nodes = []

    for node in location_list:
        if node.state.price < lowest_price:
            lowest_price = node.state.price

    for node in location_list:
        if node.state.price == lowest_price:
            if node.state.type == "ps":
                lowest_price_nodes.append(node)

    distances_from_goal = []

    for node in lowest_price_nodes:
        distance = findDistance(initial_node, node)
        distances_from_goal.append(distance)

    shortest_distance = min(distances_from_goal)
    index = distances_from_goal.index(shortest_distance)

    return lowest_price_nodes[index]

''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

def calculate_path_distance(path):
    total_distance = 0

    for i in range(len(path) - 1):
        node1 = path[i]
        node2 = path[i + 1]
        distance = findDistance(node1, node2)  
        total_distance += distance

    return round(total_distance)

def calculate_fuel_consumed(path, fuel_consumption_rate):
    total_distance = 0

    for i in range(len(path) - 1):
        node1 = path[i]
        node2 = path[i + 1]
        distance = findDistance(node1, node2)  
        total_distance += distance

    fuel_consumed = total_distance * fuel_consumption_rate
    return round(fuel_consumed)

def print_path_in_line(path):
    print("Path:", end=" ")
    for node in path:
        print(node.state.ID, end=" ")
    print()  

def print_nodes(list):
    for node in list:
        print(node.state)

def run_search_algorithm(algorithm, problem):
    start_time = time.perf_counter()
    answer, path = algorithm(problem)
    end_time = time.perf_counter()
    runtime = end_time - start_time
    runtime *= 1000000
    return answer, path, runtime

def print_search_results(algorithm_name, answer, path, runtime, fuel_consumption_rate, price, depth_limit=None):
    print("========================================================================")
    print(algorithm_name + ":")
    if answer:
        if algorithm_name == "BIDIRECTIONAL SEARCH": print("Intersection Node:", answer)
        else: print("Goal node reached:", answer)
        if algorithm_name == "DEPTH LIMITED SEARCH": print("Depth:", depth_limit)
        print_path_in_line(path)
        print("Distance:", calculate_path_distance(path), "KM")
        fuel_consumed = calculate_fuel_consumed(path, fuel_consumption_rate)
        price_in_pounds = price / 100
        print("Fuel consumed: " + str(fuel_consumed) + "L")
        print("TOTAL PATH COST: $" + str(round(price_in_pounds, 2)) + " * " + str(fuel_consumed) + "L = $" + str(round(price_in_pounds, 2) * fuel_consumed))
    else:
        print("No path found")
    print("Runtime: {:.6f} microseconds".format(runtime))
    print("========================================================================\n")

def perform_search():
    initial_node_ID = int(initial_node_ID_entry.get())
    
    if initial_node_ID < 0 or initial_node_ID >= len(location_list):
        messagebox.showerror("Error", "Invalid initial node index.")
        return

    initial_node = location_list[initial_node_ID - 1]
    
    max_distance = int(max_distance_entry.get())
    selected_price = float(price_combobox.get())

    problem.goal = selected_price
    problem.initial_node = initial_node  
    global goal_node
    goal_node = nearest_custom_price(location_list, initial_node, selected_price)

    algorithm_name = selected_algorithm.get().upper()
    if algorithm_name == "BIDIRECTIONAL SEARCH":
        algorithm = Search.bidirectional_search
    elif algorithm_name == "DEPTH LIMITED SEARCH":
        depth_limit = int(depth_limit_var.get())  
        algorithm = lambda problem: Search.depth_limited_search(problem, depth_limit)
    elif algorithm_name == "A* SEARCH":
        algorithm = Search.a_star_search
    elif algorithm_name == "GREEDY BEST FIRST SEARCH":
        algorithm = Search.greedy_best_first_search
    else:
        messagebox.showerror("Error", "Invalid algorithm selected.")
        return

    answer, path, runtime = run_search_algorithm(algorithm, problem)

    if answer:
        if algorithm_name != "DEPTH LIMITED SEARCH":
            messagebox.showinfo("Search Results", f"Algorithm: {algorithm_name}\nPath: {' -> '.join(str(node.state.ID) for node in path)}\nDistance: {calculate_path_distance(path)} KM\nFuel consumed: {calculate_fuel_consumed(path, fuel_consumption_rate)}L\nTotal path cost: ${round(selected_price/100, 2) * calculate_fuel_consumed(path, fuel_consumption_rate)}\nRuntime: {runtime:.6f} microseconds")
        else:
            messagebox.showinfo("Search Results", f"Algorithm: {algorithm_name}\nDepth: {depth_limit}\nPath: {' -> '.join(str(node.state.ID) for node in path)}\nDistance: {calculate_path_distance(path)} KM\nFuel consumed: {calculate_fuel_consumed(path, fuel_consumption_rate)}L\nTotal path cost: ${round(selected_price/100, 2) * calculate_fuel_consumed(path, fuel_consumption_rate)}\nRuntime: {runtime:.6f} microseconds")

    else:
        messagebox.showinfo("Search Results", f"Algorithm: {algorithm_name}\nNo path found.\nRuntime: {runtime:.6f} microseconds")

def update_adjacency_matrix():
    global adjacency_matrix
    max_distance = int(max_distance_entry.get())
    adjacency_matrix = createAdjacencyMatrix(location_list, max_distance)

    problem.adjacency_matrix = adjacency_matrix

    perform_search()

if __name__ == "__main__":
    location_list = read_json_file("data.json")
    adjacency_matrix = createAdjacencyMatrix(location_list, 75)

    fuel_consumption_rate = 0.5

    initial_node = location_list[12]
    lowest_price_node = nearest_lowest_price_node(initial_node, location_list)
    custom_price_node = nearest_custom_price(location_list, initial_node, 137.1)

    price = custom_price_node.state.price

    problem = Problem(adjacency_matrix, initial_node, price, location_list)

    goal_node = custom_price_node

    algorithm_name = "BIDIRECTIONAL SEARCH"
    answer, path, runtime = run_search_algorithm(Search.bidirectional_search, problem)
    print_search_results(algorithm_name, answer, path, runtime, fuel_consumption_rate, price)

    algorithm_name = "DEPTH LIMITED SEARCH"
    depth_limit = 7
    answer, path, runtime = run_search_algorithm(lambda problem: Search.depth_limited_search(problem, depth_limit), problem)
    print_search_results(algorithm_name, answer, path, runtime, fuel_consumption_rate, price, depth_limit)

    algorithm_name = "A* SEARCH"
    answer, path, runtime = run_search_algorithm(Search.a_star_search, problem)
    print_search_results(algorithm_name, answer, path, runtime, fuel_consumption_rate, price)

    algorithm_name = "GREEDY BEST FIRST SEARCH"
    answer, path, runtime = run_search_algorithm(Search.greedy_best_first_search, problem)
    print_search_results(algorithm_name, answer, path, runtime, fuel_consumption_rate, price)

    print("Branching factor:")
    print(len(scan(initial_node, adjacency_matrix, location_list)))

    print("\nConsistent heuristic?")
    node13h = Search.heuristic(location_list[12], goal_node)
    node116h = Search.heuristic(location_list[115], goal_node)
    cost13to116 = Search.heuristic(location_list[12], location_list[115])

    print("h(13):", node13h)
    print("c(13, 116)", cost13to116)
    print("h(116):", node116h)
    j = node116h + cost13to116
    print(j)
    print("Consistent?", node13h <= j)

    root = tk.Tk()
    root.title("AI Search Algorithms")

    root.geometry("320x220")
    root.resizable(False, False)

    root.configure(padx=10, pady=10)

    textbox_style = ttk.Style()
    textbox_style.configure("Custom.TEntry")

    initial_node_ID_label = tk.Label(root, text="Initial Node ID:")
    initial_node_ID_label.grid(row=0, column=0, padx=10, pady=5)
    initial_node_ID_entry = ttk.Entry(root, style="Custom.TEntry")
    initial_node_ID_entry.grid(row=0, column=1, padx=10, pady=5)
    initial_node_ID_entry.insert(0, "13") 

    max_distance_label = tk.Label(root, text="Max Travel Distance:")
    max_distance_label.grid(row=1, column=0, padx=10, pady=5)
    max_distance_entry = ttk.Entry(root, style="Custom.TEntry")
    max_distance_entry.grid(row=1, column=1, padx=10, pady=5)
    max_distance_entry.insert(0, "75")  
    max_distance_entry.bind("<Return>", lambda event: update_adjacency_matrix())

    price_label = tk.Label(root, text="Select Price:")
    price_label.grid(row=2, column=0, padx=10, pady=5)
    price_combobox = ttk.Combobox(root, values=PRICES)
    price_combobox.grid(row=2, column=1, padx=10, pady=5)
    price_combobox.set(str(min(PRICES)))  

    algorithm_label = tk.Label(root, text="Select Algorithm:")
    algorithm_label.grid(row=3, column=0, padx=10, pady=5)
    selected_algorithm = tk.StringVar()
    algorithm_combobox = ttk.Combobox(root, textvariable=selected_algorithm, values=["Bidirectional Search", "Depth Limited Search", "A* Search", "Greedy Best First Search"])
    algorithm_combobox.grid(row=3, column=1, padx=10, pady=5)
    algorithm_combobox.current(0)

    depth_limit_label = tk.Label(root, text="Depth Limit:")
    depth_limit_label.grid(row=4, column=0, padx=10, pady=5)
    depth_limit_entry = ttk.Entry(root, style="Custom.TEntry")
    depth_limit_entry.grid(row=4, column=1, padx=10, pady=5)
    depth_limit_entry.insert(0, "7")  

    depth_limit_var = tk.StringVar()
    depth_limit_var.set("7") 

    depth_limit_entry = ttk.Entry(root, style="Custom.TEntry", textvariable=depth_limit_var)
    depth_limit_entry.grid(row=4, column=1, padx=10, pady=5)

    button_style = ttk.Style()
    button_style.configure("Custom.TButton")

    search_button = ttk.Button(root, text="Search", command=update_adjacency_matrix, style="Custom.TButton")
    search_button.grid(row=5, column=0, columnspan=2, padx=10, pady=15)

    root.mainloop()
    