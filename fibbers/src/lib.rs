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

/// A Python module implemented in Rust.
#[pymodule]
fn fibbers(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(dijkstra, m)?)?;
    m.add_function(wrap_pyfunction!(dfs, m)?)?;
    Ok(())
}