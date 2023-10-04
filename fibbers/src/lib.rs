use pyo3::prelude::*;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// Represents a node with an ID and cost for Dijkstra's algorithm.
#[derive(Debug, Clone, Eq, PartialEq, Ord, PartialOrd)]
struct Node {
    id: usize,
    cost: i32,
}

impl Node {
    fn new(id: usize, cost: i32) -> Self {
        Node { id, cost }
    }
}

/// Implements Dijkstra's algorithm to find the shortest path in a weighted graph.
#[pyfunction]
fn dijkstra(graph: HashMap<usize, Vec<(usize, i32)>>, start: usize, end: usize) -> Option<Vec<usize>> {
    let mut queue = BinaryHeap::new();
    let mut prev = HashMap::new();
    let mut distances = HashMap::new();
    let mut visited = HashSet::new();

    distances.insert(start, 0);
    queue.push(Node::new(start, 0));

    while let Some(Node { id, cost }) = queue.pop() {
        if visited.contains(&id) {
            continue;
        }
        visited.insert(id);

        if id == end {
            // Reconstruct the path from 'prev'
            let mut path = vec![end];
            let mut current = end;
            while let Some(&prev_node) = prev.get(&current) {
                path.push(prev_node);
                current = prev_node;
            }
            path.reverse();
            return Some(path);
        }

        for &(neighbor, weight) in &graph[&id] {
            let alt = cost + weight;
            if alt < *distances.get(&neighbor).unwrap_or(&i32::max_value()) {
                distances.insert(neighbor, alt);
                prev.insert(neighbor, id);
                queue.push(Node::new(neighbor, alt));
            }
        }
    }

    None
}

/// Performs Depth-First Search (DFS) on a graph.
#[pyfunction]
fn dfs(graph: &HashMap<usize, Vec<usize>>, node: usize, end: usize, visited: &mut HashSet<usize>, path: &mut Vec<usize>) -> Option<Vec<usize>> {
    if node == end {
        // Reached the end node, return the path
        path.push(node);
        return Some(path.clone());
    }

    if visited.contains(&node) {
        return None; // Node has already been visited
    }

    visited.insert(node);
    path.push(node);

    for &neighbor in &graph[&node] {
        if !visited.contains(&neighbor) {
            if let Some(result) = dfs(graph, neighbor, end, visited, path) {
                return Some(result);
            }
        }
    }

    path.pop();
    None
}

#[pyfunction]
fn bfs(graph: &HashMap<usize, Vec<usize>>, node: usize, end: usize) -> Option<Vec<usize>> {
    let mut queue = vec![vec![node]];
    let mut visited = HashSet::new();

    while let Some(path) = queue.pop() {
        let node = *path.last().unwrap();
        if node == end {
            return Some(path);
        }

        if visited.contains(&node) {
            continue;
        }

        visited.insert(node);

        for &neighbor in &graph[&node] {
            if !visited.contains(&neighbor) {
                let mut new_path = path.clone();
                new_path.push(neighbor);
                queue.push(new_path);
            }
        }
    }

    None
}

#[pyfunction]
fn bellman_ford(graph: &HashMap<usize, Vec<(usize, i32)>>, start: usize, end: usize) -> Option<Vec<usize>> {
    let mut distances = HashMap::new();
    let mut prev = HashMap::new();

    distances.insert(start, 0);

    for _ in 0..graph.len() {
        for (&node, neighbors) in graph {
            for &(neighbor, weight) in neighbors {
                let alt = distances[&node] + weight;
                if alt < *distances.get(&neighbor).unwrap_or(&i32::max_value()) {
                    distances.insert(neighbor, alt);
                    prev.insert(neighbor, node);
                }
            }
        }
    }

    // Check for negative cycles
    for (&node, neighbors) in graph {
        for &(neighbor, weight) in neighbors {
            let alt = distances[&node] + weight;
            if alt < *distances.get(&neighbor).unwrap_or(&i32::max_value()) {
                return None;
            }
        }
    }

    // Reconstruct the path from 'prev'
    let mut path = vec![end];
    let mut current = end;
    while let Some(&prev_node) = prev.get(&current) {
        path.push(prev_node);
        current = prev_node;
    }
    path.reverse();
    Some(path)
}

#[pyfunction]
fn floyd_warshall(mut graph: Vec<Vec<Option<i32>>>) -> Vec<Vec<Option<i32>>> {
    let num_vertices = graph.len();

    for k in 0..num_vertices {
        for i in 0..num_vertices {
            for j in 0..num_vertices {
                if let (Some(ik), Some(kj)) = (graph[i][k], graph[k][j]) {
                    if graph[i][j].is_none() || graph[i][j].unwrap() > ik + kj {
                        graph[i][j] = Some(ik + kj);
                    }
                }
            }
        }
    }

    graph
}


/// A Python module implemented in Rust.
#[pymodule]
fn fibbers(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(dijkstra, m)?)?;
    m.add_function(wrap_pyfunction!(dfs, m)?)?;
    m.add_function(wrap_pyfunction!(bfs, m)?)?;
    m.add_function(wrap_pyfunction!(bellman_ford, m)?)?;
    m.add_function(wrap_pyfunction!(floyd_warshall, m)?)?;
    Ok(())
}