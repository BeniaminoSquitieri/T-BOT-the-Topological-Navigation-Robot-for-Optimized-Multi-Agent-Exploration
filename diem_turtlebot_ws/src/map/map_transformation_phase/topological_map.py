# Import necessary libraries
import os  # Module for interacting with the operating system
import numpy as np  # Library for advanced numerical operations on arrays
import cv2  # OpenCV library for image processing
import networkx as ntx  # Library for creating and manipulating graphs
from scipy.ndimage import distance_transform_edt, convolve, generic_filter  # Functions for image processing
from skimage.morphology import skeletonize  # Function to skeletonize binary images
from PIL import Image  # Library for image manipulation
import yaml  # Library for reading and writing YAML files
import argparse  # Module for handling command-line arguments
from sklearn.cluster import DBSCAN
from scipy.spatial import cKDTree
from skimage.draw import line as draw_line
import json
import shutil

class CoordinateTransformer:
    """
    Class to transform coordinates between the map reference system and image pixels.

    Parameters:
        image_height (int): Image height in pixels.
        resolution (float): Map resolution (meters per pixel).
        origin (tuple): Map origin in the reference system (x, y).
    """
    def __init__(self, image_height, resolution, origin):
        self.image_height = image_height
        self.resolution = resolution
        self.origin = origin  # (x_origin, y_origin)
    
    def pixel_to_map(self, node):
        """
        Converts a node's coordinates from pixel to map coordinates, with y-axis inversion.
        """
        y_pixel, x_pixel = node
        x_map = self.origin[0] + x_pixel * self.resolution
        y_map = self.origin[1] + (self.image_height - y_pixel) * self.resolution
        return x_map, y_map

    def map_to_pixel(self, x_map, y_map):
        """
        Converts map coordinates of a point to pixel coordinates with y-axis inversion.
        """
        x_pixel = int((x_map - self.origin[0]) / self.resolution)
        y_pixel = self.image_height - int((y_map - self.origin[1]) / self.resolution)
        return y_pixel, x_pixel

class Config:
    """
    Configuration class for map and graph parameters.

    Attributes:
        resolution (float): Map resolution in meters per pixel.
        origin (tuple): Map origin coordinates (x, y).
        merge_threshold (int): Distance in pixels for merging close nodes.
        max_connection_distance (int): Maximum distance in pixels to connect nodes in the graph.
    """
    def __init__(self):
        self.resolution = 0.05  # meters per pixel
        self.origin = (-32.507755, -27.073547)  # (x_origin, y_origin)
        self.merge_threshold = 50  # in pixels
        self.max_connection_distance = 100000  # in pixels

# --- Basic functions ---

def load_map(image_path):
    """
    Loads the occupancy map from a grayscale image file.

    Parameters:
        image_path (str): Path to the image file to load.

    Returns:
        numpy.ndarray: The loaded grayscale image.
    """
    # Load the grayscale image
    occupancy_grid = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if occupancy_grid is None:
        # Raise an exception if the image cannot be loaded
        raise FileNotFoundError(f"Error: unable to open file {image_path}")
    return occupancy_grid  # Returns the loaded image

def clean_map(occupancy_grid):
    """
    Applies morphological transformations to represent the map as horizontal and vertical rectangles.

    Parameters:
        occupancy_grid (numpy.ndarray): The grayscale image of the map.

    Returns:
        numpy.ndarray: The transformed map in rectangles.
    """
    # Large rectangular kernel for closing
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (60, 60)) # This initial closure uses a large kernel, ideal for connecting small spaces between rectangles.
    closed_map = cv2.morphologyEx(occupancy_grid, cv2.MORPH_CLOSE, kernel_close)

    # Dilation to expand rectangles, maintaining preferred directions
    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))  # Wider in one direction. Dilation has an oriented kernel that helps expand rectangles in horizontal or vertical directions.
    dilated_map = cv2.dilate(closed_map, kernel_dilate, iterations=1)

    # Opening to remove small irregularities and protrusions
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 65)) # Opening with a smaller kernel removes unwanted details while preserving larger shapes.
    opened_map = cv2.morphologyEx(dilated_map, cv2.MORPH_OPEN, kernel_open)

    # Final erosion to restore the original shape while keeping the rectangles
    kernel_erode = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 30)) # Final erosion helps clean up edges and reduce structure to a more consistent rectangular shape.
    final_map = cv2.erode(opened_map, kernel_erode, iterations=1)

    return final_map

def create_binary_map(occupancy_grid):
    """
    Converts the map image into a binary map.

    Parameters:
        occupancy_grid (numpy.ndarray): The grayscale image of the map.

    Returns:
        numpy.ndarray: The resulting binary map.
    """
    # Uses a threshold to binarize the image
    # Pixels with values above 240 become 1 (white), otherwise 0 (black)
    _, binary_map = cv2.threshold(occupancy_grid, 240, 1, cv2.THRESH_BINARY)
    return binary_map  # Returns the binary map

def compute_distance_map(binary_map):
    """
    Calculates the Euclidean distance map from non-zero pixels.

    Parameters:
        binary_map (numpy.ndarray): The binary map.

    Returns:
        numpy.ndarray: The Euclidean distance map.
    """
    # Calculates the Euclidean distance map
    distance_map = distance_transform_edt(binary_map)
    return distance_map  # Returns the distance map

def create_voronoi_lines(distance_map):
    """
    Creates Voronoi lines by identifying pixels with neighboring values of different distances.

    Parameters:
        distance_map (numpy.ndarray): The Euclidean distance map.

    Returns:
        numpy.ndarray: The resulting Voronoi map.
    """
    # Defines a boolean kernel to exclude the central pixel
    kernel = np.array([[1, 1, 1],
                       [1, 0, 1],
                       [1, 1, 1]], dtype=bool)

    # Defines a function that calculates the difference between the maximum and minimum value in the window
    def local_range(values):
        return values.max() - values.min()

    # Applies the local_range function on each pixel using the kernel
    local_ranges = generic_filter(distance_map, local_range, footprint=kernel)
    # Creates the Voronoi map where the local difference is greater than zero
    voronoi_map = (local_ranges > 0).astype(np.uint8)
    # Sets the edges to zero for consistency with the original method
    voronoi_map[0, :] = 0
    voronoi_map[-1, :] = 0
    voronoi_map[:, 0] = 0
    voronoi_map[:, -1] = 0

    return voronoi_map  # Returns the Voronoi map

def skeletonize_voronoi(voronoi_map):
    """
    Skeletonizes the Voronoi map to obtain thin lines.

    Parameters:
        voronoi_map (numpy.ndarray): The Voronoi map.

    Returns:
        numpy.ndarray: The skeletonized Voronoi map image.
    """
    # Skeletonizes the Voronoi map
    return skeletonize(voronoi_map)

def convert_to_map_coordinates(node, image_height, resolution, origin):
    """
    Converts the node coordinates from pixel to map coordinates with y-axis inversion.

    Parameters:
        node (tuple): Node coordinates in pixels (y, x).
        image_height (int): Image height in pixels.
        resolution (float): Map resolution (meters per pixel).
        origin (tuple): Map origin (x, y).

    Returns:
        tuple: Coordinates (x_map, y_map) in the map reference system.
    """
    y_pixel, x_pixel = node  # Note that the order is (y, x)

    # We invert the Y-axis: use (image_height - y_pixel) for conversion
    x_map = origin[0] + x_pixel * resolution
    y_map = origin[1] + (image_height - y_pixel) * resolution

    return x_map, y_map

# --- Function to check the path through white pixels ---
def create_topological_graph_using_skeleton(voronoi_skeleton, merge_threshold=50, max_connection_distance=100000, resolution=0.05, origin=(-32.507755, -27.073547), image_height=None):
    """
    Creates a topological graph based on the Voronoi skeleton,
    with nodes evenly distributed along both the X and Y axes.
    Nodes close to each other within a certain threshold are merged.
    """
    topo_map = ntx.Graph()
    
    # Define a kernel to count neighbors (excluding the central pixel)
    kernel = np.array([[1, 1, 1],
                       [1, 0, 1],
                       [1, 1, 1]], dtype=int)
    
    # Count the number of neighbors for each pixel in the skeleton
    neighbor_count = convolve(voronoi_skeleton.astype(int), kernel, mode='constant', cval=0)
    
    # Identify nodes at intersections or endpoints (where the number of neighbors is not equal to 2)
    node_positions = np.column_stack(np.where((voronoi_skeleton == 1) & (neighbor_count != 2)))
    
    # Apply DBSCAN to merge close nodes
    clustering = DBSCAN(eps=merge_threshold, min_samples=1).fit(node_positions)
    
    # Calculate the centroid of each cluster to get merged nodes
    fused_nodes = []
    for cluster_id in np.unique(clustering.labels_):
        cluster_points = node_positions[clustering.labels_ == cluster_id]
        centroid = np.mean(cluster_points, axis=0).astype(int)
        fused_nodes.append(tuple(centroid))
    
    print("Node fusion complete. Number of nodes after merging:", len(fused_nodes))

    
    # Add the merged nodes to the graph
    topo_map.add_nodes_from(fused_nodes)
    
    # # Construct a KD-Tree to efficiently search for close nodes
    # node_tree = cKDTree(fused_nodes)
    
    # # Find pairs of nodes within the maximum distance
    # pairs = node_tree.query_pairs(r=max_connection_distance)
    
    # # Procedure for creating edges
    # for i, j in pairs:
    #     node_i = fused_nodes[i]
    #     node_j = fused_nodes[j]
    #     if check_line_passes_through_skeleton(node_i, node_j, voronoi_skeleton):
    #         topo_map.add_edge(node_i, node_j)
    #         # Optional debugging
    #         # print(f"Edge created between {node_i} and {node_j}")
    
    return topo_map


# Function to check path through the skeleton
def check_line_passes_through_skeleton(node1, node2, skeleton):
    """
    Checks if a line between two nodes passes entirely through the skeleton.
    """
    y0, x0 = node1  # First node coordinates
    y1, x1 = node2  # Second node coordinates

    # Use Bresenham's algorithm to get the pixels along the line
    rr, cc = draw_line(y0, x0, y1, x1)

    # Ensure indices are within image boundaries
    valid_indices = (rr >= 0) & (rr < skeleton.shape[0]) & \
                    (cc >= 0) & (cc < skeleton.shape[1])
    rr = rr[valid_indices]
    cc = cc[valid_indices]

    # Check if all points along the line belong to the skeleton
    return np.all(skeleton[rr, cc] == 1)


# --- Function to convert NumPy types to standard Python types ---
def numpy_to_python(obj):
    """
    Converts NumPy types to standard Python types.
    Parameters:
        obj: NumPy object (array or value).
    Returns:
        Object converted to a standard Python type (float or int).
    """
    if isinstance(obj, np.ndarray):
        return obj.tolist()  # If it's a NumPy array, convert to list
    elif isinstance(obj, np.generic):
        return obj.item()  # If it's a generic NumPy object, convert to a Python value
    return obj

# --- Function to save waypoints with labels in a YAML file ---

def save_waypoints_as_yaml(waypoints, filename):
    """
    Saves waypoints with labels in a YAML file.

    Parameters:
        waypoints (list): List of waypoints (x, y) to save.
        filename (str): Path to the YAML file to save the waypoints.
    """
    # Create a dictionary for waypoint data, assigning a label to each node
    waypoints_data = {"waypoints": [{"label": f"node_{i+1}", "x": numpy_to_python(wp[0]), "y": numpy_to_python(wp[1])} 
                                    for i, wp in enumerate(waypoints)]}

    # Save the dictionary in YAML format
    with open(filename, 'w') as yaml_file:
        yaml.dump(waypoints_data, yaml_file, default_flow_style=False)

# --- Function to save an image as PNG ---

def save_as_png(image, filename):
    """
    Saves a NumPy array image as a PNG file.

    Parameters:
        image (numpy.ndarray): The image to save.
        filename (str): The path to the file to save the image.
    """
    # Convert the image to uint8 and scale values from 0 to 255 if necessary
    Image.fromarray((image * 255).astype(np.uint8)).save(filename, format="PNG")



def save_graph_as_json(topo_map, filename, transformer):
    """
    Saves the topological graph with nodes and edges in JSON format.

    Parameters:
        topo_map (networkx.Graph): The topological graph with nodes and edges.
        filename (str): Path to the JSON file to save the graph.
        transformer (CoordinateTransformer): Object to transform pixel coordinates to map coordinates.
    """
    # Convert nodes to map coordinates and add them to a list of nodes
    nodes = []
    for i, node in enumerate(topo_map.nodes()):
        x_map, y_map = transformer.pixel_to_map(node)
        nodes.append({"label": f"node_{i+1}", "x": x_map, "y": y_map})

    # Convert edges to a JSON-compatible format
    edges = [{"source": f"node_{i+1}", "target": f"node_{j+1}"} for i, j in topo_map.edges()]

    # Structure graph data in JSON format
    graph_data = {
        "nodes": nodes,
        "edges": edges
    }

    # Save the data to a JSON file
    with open(filename, 'w') as json_file:
        json.dump(graph_data, json_file, indent=4)
    
    print(f"Topological graph saved in JSON format at {filename}")

# --- Function to save the topological map with nodes in PNG format ---
def save_topological_map_with_nodes(skeleton, topo_map, png_filename, transformer):
    """
    Overlays the graph nodes on the skeleton map, drawing the coordinates of each node on the image,
    and saves the resulting image as a PNG file.
    
    Parameters:
        skeleton (numpy.ndarray): The skeleton map to overlay nodes on.
        topo_map (networkx.Graph): The topological graph with nodes.
        png_filename (str): Path to the PNG file to save the image.
        transformer (CoordinateTransformer): Object to transform node coordinates.
    """
    # Invert the skeleton image for better visibility
    skeleton_with_nodes = 255 - (skeleton * 255).astype(np.uint8)

    # Color and font for circles and text
    node_color = 0  # Black color for nodes
    text_color = 0  # Black color for text
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    thickness = 2

    # Draw nodes using pixel coordinates
    for node in topo_map.nodes():
        y_pixel, x_pixel = node
        x_map, y_map = transformer.pixel_to_map(node)
        coordinate_text = f"({x_map:.2f}, {y_map:.2f})"

        # Draw the node as a circle
        cv2.circle(skeleton_with_nodes, (x_pixel, y_pixel), 6, node_color, -1)

        # Add text with coordinates in meters next to the node
        cv2.putText(skeleton_with_nodes, coordinate_text, (x_pixel + 5, y_pixel - 5), font, font_scale, text_color, thickness)

    # Save the image with nodes and coordinates drawn in PNG format
    Image.fromarray(skeleton_with_nodes).save(png_filename, format="PNG")
    print(f"Map with node coordinates saved at {png_filename}")


# --- Function to save the associated YAML file ---

def save_as_yaml(yaml_filename, png_filename):
    """
    Creates a YAML file containing information required to use the map in ROS2.

    Parameters:
        yaml_filename (str): Path to the YAML file to create.
        png_filename (str): The name of the PNG map file.
    """
    # Create a dictionary with data required for the YAML file
    yaml_data = {
        "image": f"./{png_filename}",  # Relative path to the map image
        "resolution": 0.050000,  # Map resolution (e.g., 5 cm per pixel)
        "origin": [-32.507755, -27.073547, 0.000000],  # Map origin coordinates
        "negate": 0,  # Indicates if image colors should be inverted
        "occupied_thresh": 0.65,  # Threshold for considering a pixel as occupied
        "free_thresh": 0.196  # Threshold for considering a pixel as free
    }
    # Write the dictionary to the YAML file
    with open(yaml_filename, 'w') as file:
        yaml.dump(yaml_data, file, default_flow_style=False)

# --- Function to create a specific directory for the map ---

def create_map_directory(map_name):
    """
    Creates a new folder for the map, deleting any existing folder if present.

    Parameters:
        map_name (str): Name of the map and folder.

    Returns:
        str: Name of the created directory.
    """
    # Delete the existing folder, if present
    if os.path.exists(map_name):
        shutil.rmtree(map_name)
    # Create a new folder
    os.makedirs(map_name)
    return map_name  # Returns the directory name


def save_pixel_to_map_transformations(topo_map, filename, transformer):
    """
    Saves the transformation between pixel coordinates and map coordinates in a text file.

    Parameters:
        topo_map (networkx.Graph): The topological graph with nodes.
        filename (str): Path to the text file to save the transformations.
        transformer (CoordinateTransformer): The object for coordinate transformation.
    """
    with open(filename, 'w') as file:
        file.write("Transformation between pixel and map coordinates:\n")
        file.write("Format: (Pixel X, Pixel Y) -> (Map X, Map Y)\n\n")

        for node in topo_map.nodes():
            # Node pixel coordinates
            y_pixel, x_pixel = node

            # Conversion to map coordinates
            x_map, y_map = transformer.pixel_to_map(node)

            # Write the transformation to the file
            file.write(f"Pixel ({x_pixel}, {y_pixel}) -> Map ({x_map:.2f}, {y_map:.2f})\n")

    print(f"Transformation saved at {filename}")

# --- Function to convert nodes into waypoints ---
def convert_nodes_to_waypoints(topo_map, transformer):
    """
    Converts graph nodes into waypoints in the map reference system.

    Parameters:
        topo_map (networkx.Graph): The topological graph with nodes.
        resolution (float): Map resolution (meters per pixel).
        origin (tuple): Map origin in the reference system (x, y, theta).

    Returns:
        list: A list of waypoints (x, y) in the map reference system.
    """
    waypoints = []
    for node in topo_map.nodes():
        x_map, y_map = transformer.pixel_to_map(node)
        waypoints.append((x_map, y_map))
    return waypoints

def add_waypoints_to_original_map(original_map_path, waypoints, output_map_path_png, output_map_path_pgm, transformer):
    """
    Overlays waypoints on the original map and saves the resulting image in PNG and PGM formats.

    Parameters:
        original_map_path (str): Path to the original map.
        waypoints (list): List of waypoints to add to the map.
        output_map_path_png (str): Path to save the map with waypoints in PNG format.
        output_map_path_pgm (str): Path to save the map with waypoints in PGM format.
        transformer (CoordinateTransformer): Object to transform coordinates.
    """
    # Load the original map
    original_map = cv2.imread(original_map_path, cv2.IMREAD_GRAYSCALE)
    if original_map is None:
        raise FileNotFoundError(f"Error: unable to open file {original_map_path}")
    
    # Convert waypoints to pixel coordinates and draw them on the map
    for waypoint in waypoints:
        x_map, y_map = waypoint
        y_pixel, x_pixel = transformer.map_to_pixel(x_map, y_map)
        
        # Draw each waypoint as a circle on the map
        cv2.circle(original_map, (x_pixel, y_pixel), 6, 0, -1)  # Black circle for waypoint

    # Save the map with waypoints in PNG format
    cv2.imwrite(output_map_path_png, original_map)
    print(f"Map with waypoints saved at {output_map_path_png}")

    # Save the map with waypoints in PGM format
    Image.fromarray(original_map).save(output_map_path_pgm, format="PPM")
    print(f"Map with waypoints saved at {output_map_path_pgm}")


# --- Main function ---
def process_map(image_path):
    """
    Coordinates all steps necessary to process the map and generate final files.

    Parameters:
        image_path (str): Path to the map image to process.
    """
    config = Config()
    # Extracts the map name from the image
    map_name = os.path.splitext(os.path.basename(image_path))[0]

    # Create a folder for this map
    map_directory = create_map_directory(map_name + "_topological")

    # Step 1: Load and clean the occupancy map
    occupancy_grid = load_map(image_path)
    image_height = occupancy_grid.shape[0]  # Calculates the image height in pixels

    # Create a CoordinateTransformer object
    transformer = CoordinateTransformer(image_height, config.resolution, config.origin)

    cleaned_map = clean_map(occupancy_grid)  # Applies map cleaning
    save_as_png(cleaned_map, os.path.join(map_directory, f"{map_name}_cleaned_map.png"))

    # Step 2: Create the binary map
    binary_map = create_binary_map(cleaned_map)  # Uses the cleaned map as input
    save_as_png(binary_map, os.path.join(map_directory, f"{map_name}_binary_map.png"))

    # Step 3: Calculate the Euclidean distance map
    distance_map = compute_distance_map(binary_map)
    # Normalize the distance map for visualization
    distance_map_normalized = (distance_map / np.max(distance_map) * 255).astype(np.uint8)
    # Save the normalized distance map
    save_as_png(distance_map_normalized, os.path.join(map_directory, f"{map_name}_distance_map.png"))

    # Step 4: Create the Voronoi lines
    voronoi_map = create_voronoi_lines(distance_map)
    # Save the Voronoi map
    save_as_png(voronoi_map, os.path.join(map_directory, f"{map_name}_voronoi_map.png"))

    # Step 5: Skeletonize the Voronoi lines
    voronoi_skeleton = skeletonize_voronoi(voronoi_map)
    # Save the Voronoi skeleton
    save_as_png(voronoi_skeleton, os.path.join(map_directory, f"{map_name}_skeleton_voronoi.png"))

    # Step 6: Create the topological graph using the skeleton
    topo_map = create_topological_graph_using_skeleton(
        voronoi_skeleton,
        merge_threshold=config.merge_threshold,
        max_connection_distance=config.max_connection_distance,
        resolution=config.resolution,
        origin=config.origin,
        image_height=image_height
    )

    # Convert nodes to waypoints
    waypoints = convert_nodes_to_waypoints(topo_map, transformer)
    # Save the original map with waypoints in PNG and PGM format
    output_map_with_waypoints_png = os.path.join(map_directory, f"{map_name}_with_waypoints.png")
    output_map_with_waypoints_pgm = os.path.join(map_directory, f"{map_name}_with_waypoints.pgm")
    add_waypoints_to_original_map(image_path, waypoints, output_map_with_waypoints_png, output_map_with_waypoints_pgm, transformer)


    # Step 7: Save the topological graph as JSON
    save_graph_as_json(topo_map, os.path.join(map_directory, f"{map_name}_topological_graph.json"), transformer)

    # Save the pixel-to-map transformation in a text file
    save_pixel_to_map_transformations(
        topo_map,
        os.path.join(map_directory, f"{map_name}_pixel_to_map_transformations.txt"),
        transformer
    )

    # Step 8: Save the skeletonized map with nodes
    save_topological_map_with_nodes(
        voronoi_skeleton,
        topo_map,
        os.path.join(map_directory, f"{map_name}_topological_skeleton_nodes.pgm"),
        transformer
    )

# ### UBUNTU VERSION
# if __name__ == "__main__":

#     # Default path for the map
#     default_image_path = "/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map/diem_map.pgm"

#     # Create argument parser
#     parser = argparse.ArgumentParser(description="Generate a topological map from an occupancy map.")
#     parser.add_argument('image_path', type=str, nargs='?', default=default_image_path,
#                         help="Path to the map image to process (default: diem_turtlebot_ws/src/map/diem_map.pgm)")

#     # Parse arguments
#     args = parser.parse_args()

#     # Process the specified map
#     process_map(args.image_path)

#### WINDOWS  VERSION

if __name__ == "__main__":
    # Default path for the map, with double backslashes for Windows
    default_image_path = os.path.join("..", "diem_map.pgm")

    # Create argument parser
    parser = argparse.ArgumentParser(description="Generate a topological map from an occupancy map.")
    parser.add_argument('image_path', type=str, nargs='?', default=default_image_path,
                        help="Path to the map image to process (default: diem_turtlebot_ws/src/map/diem_map.pgm)")

    # Parse arguments
    args = parser.parse_args()

    # Process the specified map
    process_map(args.image_path)
