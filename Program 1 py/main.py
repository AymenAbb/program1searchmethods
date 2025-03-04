import math
import time
from collections import deque

# Majority of this is written by ChatGPT o1 unless otherwise stated.
# I have done error checking, padding, and some switching around of words to sound better
# Everything has been manually checked by me to make sure the algorithms are proper

def read_adjacencies(adj_file_path="adjacencies.txt"):
    """
    Reads adjacency data from a file (assumed to be space-separated lines of two city names),
    then returns a dictionary of adjacency lists.
    Also ensures bidirectional adjacency.
    """
    adjacencies = {}

    with open(adj_file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) == 2:
                city1, city2 = parts
                if city1 not in adjacencies:
                    adjacencies[city1] = []
                if city2 not in adjacencies:
                    adjacencies[city2] = []
                if city2 not in adjacencies[city1]:
                    adjacencies[city1].append(city2)
                if city1 not in adjacencies[city2]:
                    adjacencies[city2].append(city1)
            else:
                pass

    return adjacencies

def read_coordinates(coord_file_path="coordinates.csv"):
    """
    Reads city coordinates from a CSV file.
    Expects each line to contain city, lat, lon.
    Returns a dictionary: coords[city] = (lat, lon)
    """
    coords = {}
    import csv
    with open(coord_file_path, 'r', encoding='utf-8-sig') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            # Expect row like ["CityName", "lat", "lon"]
            if len(row) < 3:
                continue
            city = row[0].strip()
            try:
                lat = float(row[1])
                lon = float(row[2])
                coords[city] = (lat, lon)
            except ValueError:
                continue
    return coords

def distance_between(city1, city2, coords):
    """
    Returns the Euclidean distance (approx) between two cities,
    given their lat/lon in the coords dictionary.
    If either city is not in coords, returns a large value.
    """
    if city1 not in coords or city2 not in coords:
        return 9999999 
    lat1, lon1 = coords[city1]
    lat2, lon2 = coords[city2]
    return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

def reconstruct_path(parents, start, goal):
    """
    Reconstructs the path from start to goal using the parents dictionary.
    parents[city] = the predecessor of that city in the route.
    """
    path = []
    current = goal
    while current is not None:
        path.append(current)
        if current in parents:
            current = parents[current]
        else:
            # e.g. if current == start
            if current == start:
                break
            current = None
    path.reverse()
    return path

########################################################
# Search algorithms
########################################################

def bfs(adj, start, goal, coords, timeout=5.0):
    """
    Breadth-first search.
    Returns (route, distance, time_elapsed_in_us).
    If no route found or times out, returns (None, 0, time_elapsed_in_us).
    """
    # Double checked by using https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/
    start_time = time.time()

    visited = set()
    parents = {}
    queue = deque([start])
    visited.add(start)

    while queue:
        # Check timeout in seconds
        if time.time() - start_time > timeout:
            elapsed_us = (time.time() - start_time) * 1_000_000
            return (None, 0, elapsed_us)

        current = queue.popleft() # First in, first out
        if current == goal:
            # Found route
            path = reconstruct_path(parents, start, goal)
            dist = 0.0
            for i in range(len(path) - 1):
                dist += distance_between(path[i], path[i+1], coords)
            elapsed_us = (time.time() - start_time) * 1_000_000
            return (path, dist, elapsed_us)

        for neighbor in adj.get(current, []):
            if neighbor not in visited:
                visited.add(neighbor)
                parents[neighbor] = current
                queue.append(neighbor)

    elapsed_us = (time.time() - start_time) * 1_000_000
    return (None, 0, elapsed_us)

def dfs(adj, start, goal, coords, timeout=5.0):
    """
    Depth-first search (using a stack) to find a route.
    Returns (route, distance, time_elapsed_in_us).
    """
    # Double checked by using https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/
    start_time = time.time()
    visited = set()
    parents = {}
    stack = [start]
    visited.add(start)

    while stack: # starting a list
        if time.time() - start_time > timeout:
            elapsed_us = (time.time() - start_time) * 1_000_000
            return (None, 0, elapsed_us)

        current = stack.pop() # gets current node
        if current == goal:
            path = reconstruct_path(parents, start, goal)
            dist = 0.0
            for i in range(len(path) - 1):
                dist += distance_between(path[i], path[i+1], coords)
            elapsed_us = (time.time() - start_time) * 1_000_000
            return (path, dist, elapsed_us)

        for neighbor in adj.get(current, []):
            if neighbor not in visited:
                visited.add(neighbor)
                parents[neighbor] = current
                stack.append(neighbor)

    elapsed_us = (time.time() - start_time) * 1_000_000
    return (None, 0, elapsed_us)

def id_dfs(adj, start, goal, coords, timeout=5.0, max_depth=50):
    """
    Iterative Deepening DFS. We'll increment depth from 0 up to max_depth.
    Returns (route, distance, time_elapsed_in_us) or (None,0,time_elapsed_in_us).
    """
    start_time = time.time()

    def dls(current, depth, visited, parents):
        if time.time() - start_time > timeout:
            return None

        if current == goal:
            return [current]
        if depth == 0:
            return None
        visited.add(current)
        for neighbor in adj.get(current, []):
            if neighbor not in visited:
                parents[neighbor] = current
                result = dls(neighbor, depth-1, visited, parents)
                if result is not None:
                    return [current] + result
        return None

    for depth in range(max_depth+1): # depth limited search increases iteratively 
        if time.time() - start_time > timeout:
            elapsed_us = (time.time() - start_time) * 1_000_000
            return (None, 0, elapsed_us)
        visited = set()
        parents = {}
        path = dls(start, depth, visited, parents)
        if path is not None:
            dist = 0.0
            for i in range(len(path) - 1):
                dist += distance_between(path[i], path[i+1], coords)
            elapsed_us = (time.time() - start_time) * 1_000_000
            return (path, dist, elapsed_us)

    elapsed_us = (time.time() - start_time) * 1_000_000
    return (None, 0, elapsed_us)

def best_first_search(adj, start, goal, coords, timeout=5.0):
    """
    Best-first search, using a priority queue with heuristic = straight-line distance to goal.
    Returns (route, distance, time_elapsed_in_us).
    """
    # Double checked by using https://www.geeksforgeeks.org/best-first-search-informed-search/
    start_time = time.time()
    import heapq

    visited = set()
    parents = {}
    pq = []

    h_start = distance_between(start, goal, coords)
    heapq.heappush(pq, (h_start, start))

    while pq: #Priority queue, insert start, go till queue empty
        if time.time() - start_time > timeout:
            elapsed_us = (time.time() - start_time) * 1_000_000
            return (None, 0, elapsed_us)

        _, current = heapq.heappop(pq)
        if current == goal:
            path = reconstruct_path(parents, start, goal)
            dist = 0.0
            for i in range(len(path) - 1):
                dist += distance_between(path[i], path[i+1], coords)
            elapsed_us = (time.time() - start_time) * 1_000_000
            return (path, dist, elapsed_us)

        visited.add(current)
        for neighbor in adj.get(current, []):
            if neighbor not in visited:
                visited.add(neighbor)
                parents[neighbor] = current
                h_val = distance_between(neighbor, goal, coords) # Heuristic, prioritizes only estimated distance
                heapq.heappush(pq, (h_val, neighbor))

    elapsed_us = (time.time() - start_time) * 1_000_000
    return (None, 0, elapsed_us)

def a_star_search(adj, start, goal, coords, timeout=5.0):
    """
    A* search with cost so far + heuristic to goal.
    Returns (route, distance, time_elapsed_in_us).
    """
    # Double checked by using https://www.geeksforgeeks.org/a-search-algorithm-in-python/
    start_time = time.time()
    import heapq

    g_cost = {start: 0.0}  # Tracking cost from the beginning
    parents = {start: None}
    visited = set()

    pq = []  # initialize pq
    f_start = distance_between(start, goal, coords)
    heapq.heappush(pq, (f_start, start))

    while pq: # loop till node is found or empties
        if time.time() - start_time > timeout:
            elapsed_us = (time.time() - start_time) * 1_000_000
            return (None, 0, elapsed_us)

        f_current, current = heapq.heappop(pq) # pops minimum value
        if current == goal: 
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = parents[node]
            path.reverse()
            dist = g_cost[current]
            elapsed_us = (time.time() - start_time) * 1_000_000
            return (path, dist, elapsed_us) # return if found node

        visited.add(current)

        for neighbor in adj.get(current, []): # loop if goal node not found
            tentative_g = g_cost[current] + distance_between(current, neighbor, coords)
            if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                g_cost[neighbor] = tentative_g
                parents[neighbor] = current
                h_val = distance_between(neighbor, goal, coords)
                f_val = tentative_g + h_val # Combined with heuristic
                heapq.heappush(pq, (f_val, neighbor))

    elapsed_us = (time.time() - start_time) * 1_000_000
    return (None, 0, elapsed_us)

def main():
    adj_file = "adjacencies.txt"
    coord_file = "coordinates.csv"

    adj = read_adjacencies(adj_file)
    coords = read_coordinates(coord_file)

    print("Welcome to the Route-Finding Program.")
    print("Cities recognized:", list(adj.keys()))

    while True:
        start_city = input("\nEnter your start city (or type 'quit' to exit): ").strip()
        if start_city.lower() == 'quit':
            print("Exiting.")
            break
        if start_city not in adj:
            print("City not recognized. Please try again.")
            continue
        goal_city = input("Enter your destination city: ").strip()
        if goal_city not in adj:
            print("City not recognized. Please try again.")
            continue
        while True:
            print("\nSelect a search method:")
            print("1) BFS")
            print("2) DFS")
            print("3) ID-DFS")
            print("4) Best-First Search")
            print("5) A*")
            print("6) Return to city selection")
            print("7) Quit the program")
            choice = input("Enter your choice (7): ").strip() # Fixed input range mistake

            if choice == '6':
                break
            elif choice == '7':
                print("Quitting program.")
                import sys
                sys.exit(0)  # go back to city selection

            route, dist, elapsed_us = None, 0, 0

            if choice == '1':
                print("Performing BFS...")
                route, dist, elapsed_us = bfs(adj, start_city, goal_city, coords)
            elif choice == '2':
                print("Performing DFS...")
                route, dist, elapsed_us = dfs(adj, start_city, goal_city, coords)
            elif choice == '3':
                print("Performing ID-DFS...")
                route, dist, elapsed_us = id_dfs(adj, start_city, goal_city, coords)
            elif choice == '4':
                print("Performing Best-First Search...")
                route, dist, elapsed_us = best_first_search(adj, start_city, goal_city, coords)
            elif choice == '5':
                print("Performing A* Search...")
                route, dist, elapsed_us = a_star_search(adj, start_city, goal_city, coords)
            else:
                print("Invalid choice. Try again.")
                continue

            if route is None: # Padded some of this section and changed names manually to look nicer
                print(f"No route found or search timed out. Elapsed: {elapsed_us:.2f} µs")
            else:
                print("Route found:", " -> ".join(route))
                print(f"Total distance: {dist:.2f}")
                print(f"\nTime elapsed: {elapsed_us:.2f} µs")

if __name__ == "__main__":
    main()
