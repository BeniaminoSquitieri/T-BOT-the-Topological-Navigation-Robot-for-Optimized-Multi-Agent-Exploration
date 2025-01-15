# Graph Traversal Time Calculator

## Overview

The **Graph Traversal Time Calculator** is a Python script designed to compute and compare traversal times across different graph traversal algorithms. Specifically, it implements the **Chinese Postman Problem (CPP)**, an approximation of the **Travelling Salesman Problem (TSP)**, **Depth-First Search (DFS)**, and **Breadth-First Search (BFS)**. The tool is particularly useful for applications involving robotic path planning, where understanding traversal efficiency is crucial.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
  - [Input Format](#input-format)
  - [Running the Script](#running-the-script)
- [Algorithms Implemented](#algorithms-implemented)
  - [Chinese Postman Problem (CPP)](#chinese-postman-problem-cpp)
  - [Travelling Salesman Problem (TSP) Approximation](#travelling-salesman-problem-tsp-approximation)
  - [Depth-First Search (DFS)](#depth-first-search-dfs)
  - [Breadth-First Search (BFS)](#breadth-first-search-bfs)
- [Output](#output)
- [Example](#example)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Features

- **Graph Import**: Reads graphs from JSON files with nodes and edges, including positional data and distances.
- **Traversal Algorithms**: Implements CPP, TSP approximation, DFS, and BFS to compute traversal paths.
- **Distance Calculation**: Accurately computes total traversal distances based on edge attributes.
- **Performance Metrics**: Measures both traversal time and execution time for each algorithm.
- **Verification**: Ensures all edges in the graph have the necessary distance attributes.
- **Extensibility**: Easily extendable to incorporate additional traversal algorithms or features.

