"""
    Python Algorithms Module

    This module contains the Python algorithms used in the project.
"""

import heapq

def dfs(graph: list, node: int) -> list:
    """
    Depth-first search algorithm
    :param graph: graph to search
    :param node: node to start search from
    :return: list of nodes in order of visit

    Needs the adjacency list
    """
    visited = []
    stack = [node]
    while stack:
        node = stack.pop()
        if node not in visited:
            visited.append(node)
            stack.extend(graph[node])
    return visited

def bfs(graph: list, node: int) -> list:
    """
    Breadth-first search algorithm
    :param graph: graph to search
    :param node: node to start search from
    :return: list of nodes in order of visit

    Needs the adjacency list
    """
    visited = []
    queue = [node]
    while queue:
        node = queue.pop(0)
        if node not in visited:
            visited.append(node)
            queue.extend(graph[node])
    return visited

def dijkstra(graph: list, start: int, end: int) -> list:
    """
    Dijkstra's algorithm
    :param graph: graph to search
    :param start: node to start search from
    :param end: node to end search at
    :return: list of nodes in order of visit

    Needs the adjacency list
    """
    distances = {vertex: float('inf') for vertex in graph}
    distances[start] = 0
    pq = [(0, start)]
    while len(pq) > 0:
        current_distance, current_vertex = heapq.heappop(pq)
        if current_distance > distances[current_vertex]:
            continue
        for neighbor, weight in graph[current_vertex]:
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(pq, (distance, neighbor))
    return distances[end]

def bellman_ford(graph: list, start: int, end: int) -> list:
    """
    Bellman-Ford algorithm
    :param graph: graph to search
    :param start: node to start search from
    :param end: node to end search at
    :return: list of nodes in order of visit

    Needs the adjacency list
    """
    distances = {vertex: float('inf') for vertex in graph}
    distances[start] = 0
    for _ in range(len(graph) - 1):
        for node in graph:
            for neighbor, weight in graph[node]:
                if distances[node] + weight < distances[neighbor]:
                    distances[neighbor] = distances[node] + weight
    return distances[end]

def floyd_warshall(graph):
    """
    Floyd-Warshall algorithm
    :param graph: graph to search
    :param start: node to start search from
    :param end: node to end search at
    :return: list of nodes in order of visit
    """
    v = len(graph)

    dist = [[float('inf')] * v for _ in range(v)]
    for i in range(v):
        for j in range(v):
            if i == j:
                dist[i][j] = 0
            elif graph[i][j] is not None:
                dist[i][j] = graph[i][j]

    for k in range(v):
        for i in range(v):
            for j in range(v):
                if dist[i][k] != float('inf') and dist[k][j] != float('inf') and dist[i][k] + dist[k][j] < dist[i][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]

    return dist
