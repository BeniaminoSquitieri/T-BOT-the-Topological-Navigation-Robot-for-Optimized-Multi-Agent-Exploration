# coordinate_transformer.py

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
