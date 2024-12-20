# fleet_turtlebot4_navigation/master/utils.py

import math
import networkx as nx

def orientation_rad_to_str(orientation_rad: float) -> str:
    """
    Converts orientation from radians to a cardinal direction string.

    Args:
        orientation_rad (float): Orientation in radians.

    Returns:
        str: Cardinal direction ("NORTH", "EAST", "SOUTH", "WEST").
    """
    direction_map = {
        0.0: "NORTH",
        -math.pi / 2: "EAST",
        math.pi: "SOUTH",
        math.pi / 2: "WEST"
    }
    # Find the closest matching orientation
    for rad, dir_str in direction_map.items():
        if math.isclose(orientation_rad, rad, abs_tol=1e-2):
            return dir_str
    return "UNKNOWN"

def orientation_str_to_rad(orientation_str: str) -> float:
    """
    Converts a cardinal direction string to radians.

    Args:
        orientation_str (str): Cardinal direction ("NORTH", "EAST", "SOUTH", "WEST").

    Returns:
        float: Orientation in radians.
    """
    orientation_map = {
        "NORTH": 0.0,
        "EAST": -math.pi / 2,
        "SOUTH": math.pi,
        "WEST": math.pi / 2
    }
    return orientation_map.get(orientation_str.upper(), 0.0)

def find_node_from_position(graph: nx.DiGraph, x: float, y: float, tolerance: float = 1e-4) -> str:
    """
    Finds the graph node corresponding to the given coordinates (x, y).

    Args:
        graph (nx.DiGraph): The navigation graph.
        x (float): X-coordinate of the position.
        y (float): Y-coordinate of the position.
        tolerance (float): Tolerance for matching coordinates.

    Returns:
        str: The label of the node in the graph that matches the given coordinates.
             Returns None if no matching node is found.
    """
    for node, data in graph.nodes(data=True):
        if math.isclose(data['x'], x, abs_tol=tolerance) and math.isclose(data['y'], y, abs_tol=tolerance):
            return node
    return None
