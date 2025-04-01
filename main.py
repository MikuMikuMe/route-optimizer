Creating a complete route optimizer for delivery using dynamic routing algorithms involves several steps, such as modeling the delivery network, computing optimal routes, and handling real-time data. Here, I will provide a simplified Python program to illustrate a basic approach using Dijkstra's algorithm to find the shortest path, which is a common strategy for route optimization. We will simulate a network of locations (nodes) and roads (edges) between them with weights representing travel costs or times.

Firstly, ensure you have the necessary packages. We will use NetworkX, a library suitable for creating and manipulating graphs and complex networks:

You can install it via pip:
```bash
pip install networkx
```

Now, here's a basic Python program implementing a route optimization system:

```python
import networkx as nx
import heapq

# Function to calculate the shortest path using Dijkstra's algorithm
def dijkstra(graph, start, end):
    """
    Calculate the shortest path in a graph using Dijkstra's algorithm.
    
    :param graph: A NetworkX graph
    :param start: The starting node
    :param end: The destination node
    :return: tuple (path as a list of nodes, cost of the path) 
    """
    # Priority queue to store the nodes to explore
    queue = []
    heapq.heappush(queue, (0, start))
    
    # Dictionary to store the shortest path to each node
    shortest_paths = {node: float('inf') for node in graph.nodes}
    shortest_paths[start] = 0
    
    # Dictionary to store the optimal previous node in the path
    previous_nodes = {node: None for node in graph.nodes}
    
    while queue:
        # Current node to explore
        current_cost, current_node = heapq.heappop(queue)
        
        # If we reached the end node, return the path and its cost
        if current_node == end:
            path = []
            while previous_nodes[current_node] is not None:
                path.append(current_node)
                current_node = previous_nodes[current_node]
            path.append(start)
            return path[::-1], current_cost
        
        # Explore neighbors
        for neighbor, attributes in graph[current_node].items():
            weight = attributes.get("weight", 1)
            cost = current_cost + weight
            
            # If a cheaper path to the neighbor is found
            if cost < shortest_paths[neighbor]:
                shortest_paths[neighbor] = cost
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (cost, neighbor))
    
    raise nx.NetworkXNoPath(f"No path from {start} to {end}")

def main():
    # Simulating a graph for delivery locations (as nodes) 
    # and roads/connections (edges with weights)
    G = nx.Graph()
    
    # Add nodes
    locations = ['A', 'B', 'C', 'D', 'E']
    G.add_nodes_from(locations)

    # Add edges with weights representing travel cost/duration
    edges = [
        ('A', 'B', 4), ('A', 'C', 2),
        ('B', 'C', 1), ('B', 'D', 5),
        ('C', 'D', 8), ('C', 'E', 10),
        ('D', 'E', 2), ('E', 'D', 6)
    ]
    
    G.add_weighted_edges_from(edges)

    # Input: starting location and ending location
    start_location = 'A'
    end_location = 'E'
    
    try:
        # Calculate the shortest path and its cost
        path, cost = dijkstra(G, start_location, end_location)
        print(f"Optimal route from {start_location} to {end_location}: {' -> '.join(path)} with cost {cost}")
    except nx.NetworkXNoPath as e:
        print(e)

# Entry point for the script
if __name__ == '__main__':
    main()
```

### Explanation:

1. **Graph Representation**: We use the NetworkX library to create a graph representing the network of delivery locations. Nodes represent locations, and edges between them have weights representing the travel time or cost.

2. **Dijkstra's Algorithm**: This algorithm finds the shortest path between two nodes in a weighted graph. It uses a priority queue (heapq) for efficiency.

3. **Error Handling**: The program handles exceptions when no path is available between the given start and end nodes.

4. **Main Function**: The main function sets up the graph with example nodes and edges, specifying connections and weights (distances or costs), and finds the shortest path between two nodes. 

This basic model can be expanded with more sophisticated algorithms, real data integration, and dynamic updates as new data (such as traffic conditions) becomes available.