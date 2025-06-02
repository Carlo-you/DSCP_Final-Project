import pandas as pd
import heapq
from collections import defaultdict

# Configuration
speed = 3  # m/s

# Load the CSV with Start_Time
df = pd.read_csv('road.csv')

# Build graph: each edge = (to, green, red, start_offset, distance)
graph = defaultdict(list)
for _, row in df.iterrows():
    from_node = row['From']
    to_node = row['To']
    green = row['Green_Time']
    red = row['Red_Time']
    start_offset = row['Start_Time']
    distance = row['Distant']

    graph[from_node].append((to_node, green, red, start_offset, distance))
    graph[to_node].append((from_node, green, red, start_offset, distance))  # bidirectional

# Time-dependent Dijkstra with Start_Time and logging
def time_dependent_dijkstra_with_offset(start, end):
    heap = [(0, start, [])]  # (current_time, node, path_so_far)
    visited = dict()
    step_log = dict()

    while heap:
        current_time, node, path = heapq.heappop(heap)

        if node in visited and visited[node] <= current_time:
            continue
        visited[node] = current_time
        path = path + [node]

        if node == end:
            print("\n🛣️ Path Trace with Timing:")
            total_time = 0
            for i in range(1, len(path)):
                prev = path[i-1]
                curr = path[i]
                arrival, wait, travel, _ = step_log[curr]
                print(f"→ {prev} to {curr}: wait {wait:.2f}s, travel {travel:.2f}s → arrived at {arrival:.2f}s")
                total_time = arrival
            return total_time, path

        for neighbor, green, red, start_offset, distance in graph[node]:
            cycle = green + red
            # Compute phase time of light including its start offset
            phase_time = (current_time + start_offset) % cycle
            wait_time = 0 if phase_time < green else cycle - phase_time
            travel_time = distance / speed
            arrival_time = current_time + wait_time + travel_time

            if neighbor in visited and visited[neighbor] <= arrival_time:
                continue

            step_log[neighbor] = (arrival_time, wait_time, travel_time, node)
            heapq.heappush(heap, (arrival_time, neighbor, path))

    return float('inf'), []

# Run example
start_node = 'A'
end_node = 'F'

total_time, best_path = time_dependent_dijkstra_with_offset(start_node, end_node)
print(f"\n✅ Fastest time from {start_node} to {end_node}: {total_time:.2f} seconds")
print("🏁 Path:", " → ".join(best_path))
