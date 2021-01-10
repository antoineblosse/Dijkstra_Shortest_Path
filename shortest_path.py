import requests
import networkx as nx
from queue import PriorityQueue
from math import inf
import matplotlib.pyplot as plt
import timeit

# start timer for program runtime
start = timeit.default_timer()

def help_message():
    print("This program determines the shortest path between two metro stations in Paris, using Dijkstra's algorithm. "
          "Enter the start station and destination station, with the first letter capitalized.")
    print("A GET request obtains the metro stations from an API. A Networkx graph of all the stations is then created. "
          "The function dijkstra works by creating a 'dist' dictionary containing optimal distance between start_node "
          "and current_node, initialized at d=inf.")
    print("Another dictionary 'prev' is created for the nodes on the shortest path. As dijkstra function runs, the "
          "algorithm starts at the start node and updates the distance between itself and all its neighbors, which "
          "update the distances in 'dist' dictionary.")
    print("A priority queue keeps track of which nodes to visit next. The dijkstra function stops when the priority "
          "queue is empty, and returns the shortest path.")


print("-------------- Dijkstra's Shortest Path --------------")
print("This program determines the shortest path between two metro stations in Paris, using Dijkstra's algorithm. "
      "Type 'help' for a help message.")
print("Please enter the start station and destination station.")

start_node = input("Start station: ")
end_node = input("End station: ")
if start_node == "help":
    help_message()
    quit()

# every Paris metro line number
line_numbers = ['1', '2', '3', '3B', '4', '5', '6', '7', '7B', '8', '9', '10', '11', '12', '13', '14']
metro_line = []    # list for all stations in one metro line
metro_lines = []    # list of lists, each list is appended metro_line for each line
nodes = []    # list for nodes of adjacency matrix
G = nx.Graph()    # create Networkx graph

# GET request stations from Paris metro API
for n in range(len(line_numbers)):
    url = f'https://api-ratp.pierre-grimaud.fr/v4/stations/metros/{line_numbers[n]}'
    response = requests.get(url)
    stations = response.json()['result']['stations']    # Extract names of stations

    # add station names to list of nodes for adjacency matrix
    for s in stations:
        station = s['name']
        metro_line.append(station)    # names of stations in metro line
    metro_lines.append(metro_line)
    metro_line = []

# create Networkx graph
for line in metro_lines:
    # check to make sure station is in metro_lines, else quit()
    verify_user_input = any(start_node and end_node in line for line in metro_lines)
    if verify_user_input is False:
        print("User input is not a valid metro station. Please try again.")
        quit()

    # add nodes and edges to Networkx graph
    previous = None  # previous station in metro line
    for station in line:
        # skip add_edge if first station in line
        # # avoids self-link and link between last station of line and first station of next line
        if station == line[0]:
            pass
        else:
            G.add_edge(previous, station)    # add_edge between adjacent metro stations; add_edge also adds nodes
        previous = station

# G is an unweighted Networkx Graph, with 302 stations and 356 edges
# d=2 between every stations, so calculate unweighted shortest path then multiply number of nodes in shortest path by 2
def dijkstra(graph, start_node, end_node):
    # Credits: https://gist.github.com/aeged/db5bfda411903ecd89a3ba3cb7791a05
    # Credits: https://www.youtube.com/watch?v=pSqmAO-m7Lk

    visited = set()  # set of visited nodes
    prev = {}  # dictionary containing key-pair of previous node on shortest path
    dist = {}  # dictionary containing optimal distance between start_node and current_node; initialize at d=inf
    cost = 2  # for simplicity, we assume that distance between each station is constant, d=2min
    pq = PriorityQueue()  # prioritize nodes from start -> current node with the shortest distance

    # backtrace returns shortest path of nodes by going backwards through prev list
    def backtrace(prev, start_node, end_node):
        node = end_node
        path = []
        while node != start_node:
            path.append(node)
            node = prev[node]
        path.append(node)
        path.reverse()
        return path

    # initialize distances from start to every node at inf
    for vertex in list(G.nodes):
        dist.update({vertex: inf})

    dist[start_node] = 0  # dist from start to start is zero
    pq.put((dist[start_node], start_node))    # add start_node to priority queue

    # priority queue tells us which node to visit next, based on which key:value pair has the lowest value
    # when priority queue is empty, every possible path has been checked
    while pq.qsize() > 0:
        current_node_cost, current_node = pq.get()
        visited.add(current_node)    # add current_node to list of visited nodes
        # look at adjacent nodes of current_node
        for neighbor in dict(graph.adjacency()).get(current_node):
            path = dist[current_node] + cost
            if path < dist[neighbor]:
                dist[neighbor] = path    # if found a shorter path, update distance with new shorter distance
                prev[neighbor] = current_node    # update the previous node to be prev on new shortest path
                if neighbor not in visited:
                    # if neighbor not yet visited, insert into priority queue and mark as visited
                    visited.add(neighbor)
                    pq.put((dist[neighbor], neighbor))
                # else update entry in priority queue
                else:
                    _ = pq.get((dist[neighbor], neighbor))    # remove old neighbor
                    pq.put((dist[neighbor], neighbor))    # insert new neighbor

    # after every possible path has been checked, return shortest path
    return backtrace(prev, start_node, end_node)


shortest_path = dijkstra(G, start_node, end_node)
print(f"Shortest path between {start_node} and {end_node}: {' -> '.join(shortest_path)}")
print(f"Time to destination: {(len(shortest_path)-1) * 2} minutes")

fig = plt.figure(figsize=(10, 10))
nx.draw(G, node_size=20)
plt.axis('equal')
fig.savefig('Paris_metro_lines.svg')

stop = timeit.default_timer()
print()
print(f'Program runtime: {stop - start} s')
