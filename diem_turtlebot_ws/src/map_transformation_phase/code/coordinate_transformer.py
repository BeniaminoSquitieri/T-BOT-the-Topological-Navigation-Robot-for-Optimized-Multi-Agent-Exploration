# coordinate_transformer.py

class CoordinateTransformer:
    """
    Class to transform coordinates between the map reference system and image pixels.

    This class facilitates the conversion of coordinates from the map's real-world reference frame
    to the corresponding pixel locations in an image representation of the map, and vice versa.
    It accounts for the map's resolution and origin to ensure accurate and consistent transformations.

    Parameters:
        image_height (int): The height of the map image in pixels. This value is essential for
                            correctly inverting the y-axis during transformations.
        resolution (float): The scale of the map, defined as meters per pixel. It determines how
                            real-world distances are mapped to pixel distances.
        origin (tuple): A tuple representing the (x, y) coordinates of the map's origin in the
                        real-world reference system. This origin corresponds to the bottom-left
                        corner of the map image.
    """
    def __init__(self, image_height, resolution, origin):
        """
        Initializes the CoordinateTransformer with the provided image height, resolution, and origin.

        Args:
            image_height (int): The height of the map image in pixels.
            resolution (float): The map resolution in meters per pixel.
            origin (tuple): The (x, y) coordinates of the map's origin in the real-world reference system.
        """
        # Store the image height for y-axis inversion during transformations
        self.image_height = image_height  # Total number of pixels vertically in the map image

        # Store the resolution to convert between meters and pixels
        self.resolution = resolution      # Map scale: meters represented by each pixel

        # Store the origin coordinates for reference frame alignment
        self.origin = origin              # Origin point (x_origin, y_origin) in meters

    def pixel_to_map(self, node):
        """
        Converts a node's coordinates from pixel space to real-world map coordinates,
        accounting for the inversion of the y-axis inherent in image representations.

        This method translates pixel-based coordinates (as used in image processing) to
        real-world coordinates (as used in navigation and mapping), ensuring that the
        spatial relationships are accurately maintained.

        Args:
            node (tuple): A tuple containing the (y_pixel, x_pixel) coordinates in the image.
                          - y_pixel (int): The vertical pixel index (row) in the image.
                          - x_pixel (int): The horizontal pixel index (column) in the image.

        Returns:
            tuple: A tuple containing the (x_map, y_map) coordinates in the real-world map.
                   - x_map (float): The real-world x-coordinate in meters.
                   - y_map (float): The real-world y-coordinate in meters.
        """
        # Unpack the pixel coordinates from the input tuple
        y_pixel, x_pixel = node  # y_pixel corresponds to the row index, x_pixel to the column index

        # Convert the x_pixel to real-world x_map coordinate
        # Calculation: x_map = origin_x + (x_pixel * resolution)
        x_map = self.origin[0] + x_pixel * self.resolution
        # Explanation:
        # - (x_pixel * resolution) converts pixel distance to meters.
        # - Adding origin_x aligns the pixel coordinate with the real-world origin.

        # Convert the y_pixel to real-world y_map coordinate with y-axis inversion
        # Calculation: y_map = origin_y + ((image_height - y_pixel) * resolution)
        y_map = self.origin[1] + (self.image_height - y_pixel) * self.resolution
        # Explanation:
        # - (self.image_height - y_pixel) inverts the y-axis because image coordinates start from the top-left,
        #   whereas real-world coordinates typically start from the bottom-left.
        # - Multiplying by resolution converts pixel distance to meters.
        # - Adding origin_y aligns the pixel coordinate with the real-world origin.

        # Return the transformed real-world coordinates
        return x_map, y_map  # (x_map, y_map) in meters

    def map_to_pixel(self, x_map, y_map):
        """
        Converts real-world map coordinates to pixel coordinates in the image,
        accounting for the inversion of the y-axis inherent in image representations.

        This method translates real-world coordinates (as used in navigation and mapping)
        to pixel-based coordinates (as used in image processing), ensuring that the
        spatial relationships are accurately maintained.

        Args:
            x_map (float): The real-world x-coordinate in meters.
            y_map (float): The real-world y-coordinate in meters.

        Returns:
            tuple: A tuple containing the (y_pixel, x_pixel) coordinates in the image.
                   - y_pixel (int): The vertical pixel index (row) in the image.
                   - x_pixel (int): The horizontal pixel index (column) in the image.
        """
        # Calculate the horizontal pixel index (x_pixel) from the real-world x_map coordinate
        # Calculation: x_pixel = (x_map - origin_x) / resolution
        x_pixel = int((x_map - self.origin[0]) / self.resolution)
        # Explanation:
        # - (x_map - origin_x) computes the distance from the origin in meters.
        # - Dividing by resolution converts meters to pixels.
        # - int() truncates the result to the nearest lower integer pixel index.

        # Calculate the vertical pixel index (y_pixel) from the real-world y_map coordinate
        # Calculation: y_pixel = image_height - ((y_map - origin_y) / resolution)
        y_pixel = self.image_height - int((y_map - self.origin[1]) / self.resolution)
        # Explanation:
        # - (y_map - origin_y) computes the distance from the origin in meters.
        # - Dividing by resolution converts meters to pixels.
        # - int() truncates the result to the nearest lower integer pixel index.
        # - (self.image_height - ...) inverts the y-axis to match image coordinate system
        #   where y increases downward.

        # Return the transformed pixel coordinates
        return y_pixel, x_pixel  # (y_pixel, x_pixel) in image pixels
