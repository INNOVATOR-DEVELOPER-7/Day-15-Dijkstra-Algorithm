import heapq

def dijkstra(graph, start_node, target_node):
    # Priority queue to store (distance, vertex) tuples
    priority_queue = [(0, start_node)]
    # Dictionary to store the shortest distance to each vertex
    distances = {vertex: float('infinity') for vertex in graph}
    distances[start_node] = 0
    # Dictionary to store the path
    previous_nodes = {vertex: None for vertex in graph}

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # Early exit if we reach the target node
        if current_node == target_node:
            break

        if current_distance > distances[current_node]:
            continue

        # Explore neighbors
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight

            # Only consider this new path if it's better
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    # Reconstruct path
    path, current_node = [], target_node
    while previous_nodes[current_node] is not None:
        path.insert(0, current_node)
        current_node = previous_nodes[current_node]
    if path:
        path.insert(0, current_node)

    return path, distances[target_node]

# Get the graph from the user
user_input = input("Enter the graph as adjacency list with weights (e.g., {'A': {'B': 1, 'C': 4}, 'B': {'A': 1, 'D': 2}, ...}): ")
graph = eval(user_input)

# Get the starting node from the user
start_node = input("Enter the starting node: ")

# Get the target node from the user
target_node = input("Enter the target node: ")

# Perform Dijkstra's algorithm
path, distance = dijkstra(graph, start_node, target_node)

# Print the shortest path and distance
print("Shortest path from", start_node, "to", target_node, ":", path)
print("Total distance:", distance)
