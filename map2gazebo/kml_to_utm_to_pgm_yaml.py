import numpy as np
import cv2
from pyproj import Transformer
import xml.etree.ElementTree as ET

class Map:
    def __init__(self,  kml_file, pgm_file, yaml_file, resolution=1, yaml_resolution=1, padding=20):
        self.kml_file = kml_file
        self.pgm_file = pgm_file
        self.yaml_file = yaml_file
        self.resolution = resolution
        self.yaml_resolution = yaml_resolution
        self.padding = padding
        self.polygon = self.parse_kml(self.kml_file)
        self.transformer_to_utm = Transformer.from_crs("epsg:4326", "epsg:32633", always_xy=True)  # WGS84 to UTM Zone 33N
        self.transformer_to_latlon = Transformer.from_crs("epsg:32633", "epsg:4326", always_xy=True)  # UTM Zone 33N to WGS84
        self.min_utm_x, self.max_utm_x, self.min_utm_y, self.max_utm_y = self.get_map_bounds()
        self.width, self.height = self.calculate_dimensions()

    def parse_kml(self, kml_file):
        """
        Parse the KML file to extract the polygon (lat, lon) coordinates.
        """
        tree = ET.parse(kml_file)
        root = tree.getroot()
        namespace = {"kml": "http://www.opengis.net/kml/2.2"}

        # Find all coordinates
        coordinates_tag = root.find('.//kml:coordinates', namespaces=namespace)
        coordinates_text = coordinates_tag.text.strip()
        coordinates = []

        for coord in coordinates_text.split():
            lon, lat, _ = map(float, coord.split(','))
            coordinates.append((lon, lat))

        return coordinates

    def get_map_bounds(self):
        """
        Get UTM bounds of the polygon.
        """
        utm_coords = [self.latlon_to_utm(lon, lat) for lon, lat in self.polygon]
        utm_x, utm_y = zip(*utm_coords)
        return min(utm_x), max(utm_x), min(utm_y), max(utm_y)

    def latlon_to_utm(self, lon, lat):
        """
        Convert latitude and longitude to UTM coordinates using pyproj.
        """
        x, y = self.transformer_to_utm.transform(lon, lat)
        return x, y

    def utm_to_latlon(self, x, y):
        """
        Convert UTM coordinates to latitude and longitude using pyproj.
        """
        lon, lat = self.transformer_to_latlon.transform(x, y)
        return lon, lat

    def calculate_dimensions(self):
        """
        Calculate the width and height of the occupancy grid in pixels.
        """
        width = max(int((self.max_utm_x - self.min_utm_x) / self.resolution), 10)
        height = max(int((self.max_utm_y - self.min_utm_y) / self.resolution), 10)
        print(f"{width} {height}")
        return width, height

    def latlon_to_pixel(self, lat, lon):
        """
        Convert latitude and longitude to pixel coordinates.
        """
        utm_x, utm_y = self.latlon_to_utm(lon, lat)
        x = int((utm_x - self.min_utm_x) / self.resolution)
        y = int((utm_y - self.min_utm_y) / self.resolution)
        return x, y

    def pixel_to_latlon(self, pixel_x, pixel_y):
        """
        Convert pixel coordinates back to latitude and longitude.
        """
        utm_x = self.min_utm_x + (pixel_x * self.resolution)
        utm_y = self.min_utm_y + (pixel_y * self.resolution)
        lon, lat = self.utm_to_latlon(utm_x, utm_y)
        return lon, lat
    
    def utm_to_pixel(self, utm_x, utm_y):
        """
        Convert latitude and longitude to pixel coordinates.
        """
        x = (utm_x - self.min_utm_x) / self.resolution
        y = (utm_y - self.min_utm_y) / self.resolution
        return x, y

    def pixel_to_utm(self, pixel_x, pixel_y):
        """
        Convert pixel coordinates back to latitude and longitude.
        """
        utm_x = self.min_utm_x + (pixel_x * self.resolution)
        utm_y = self.min_utm_y + (pixel_y * self.resolution)
        return utm_x, utm_y

    def calculate_shifted_origin(self, other_map):
        """
        Calculate the shifted origin of another map relative to this one.
        """
        origin_x2, origin_y2 = 0, 0
        utm_x2, utm_y2 = self.pixel_to_utm(origin_x2, origin_y2)
        origin_x2, origin_y2 = other_map.utm_to_pixel(utm_x2, utm_y2)
        origin_x2 *= self.yaml_resolution
        origin_y2 *= self.yaml_resolution
        return origin_x2, origin_y2

    def create_occupancy_grid(self):
        """
        Create the occupancy grid for the map by filling the polygon.
        """
        grid = np.ones((self.height, self.width), dtype=np.uint8) * 0
        poly_pixels = [self.latlon_to_pixel(lat, lon) for lon, lat in self.polygon]
        poly_pixels = np.array(poly_pixels, dtype=np.int32)
        cv2.fillPoly(grid, [poly_pixels], color=255)
        grid = cv2.flip(grid, 0)
        return grid

    def add_padding(self, grid):
        """
        Add padding to the grid and adjust UTM coordinates.
        """
        height, width = grid.shape
        diff_in_size = abs(height - width)
        if height > width:
            padded_grid = np.ones((height + 2 * self.padding, width + diff_in_size + 2 * self.padding), dtype=np.uint8) * 0
            padded_grid[self.padding:self.padding + height, self.padding + int(diff_in_size / 2):self.padding + width + int(diff_in_size / 2)] = grid
            new_min_utm_x, new_min_utm_y = self.pixel_to_utm(-(self.padding + int(diff_in_size / 2)), -self.padding)
        elif width > height:
            padded_grid = np.ones((height + diff_in_size + 2 * self.padding, width + 2 * self.padding), dtype=np.uint8) * 0
            padded_grid[self.padding + int(diff_in_size / 2):self.padding + height + int(diff_in_size / 2), self.padding:self.padding + width] = grid
            new_min_utm_x, new_min_utm_y = self.pixel_to_utm(-self.padding, -(self.padding + int(diff_in_size / 2)))
        else:
            padded_grid = np.ones((height + 2 * self.padding, width + 2 * self.padding), dtype=np.uint8) * 0
            padded_grid[self.padding:self.padding + height, self.padding:self.padding + width] = grid
            new_min_utm_x, new_min_utm_y = self.pixel_to_utm(-self.padding, -self.padding)
        return padded_grid, new_min_utm_x, new_min_utm_y

    def save_pgm(self, grid):
        """
        Save the occupancy grid as a PGM file.
        """
        cv2.imwrite(self.pgm_file, grid)

    def save_yaml(self, origin_x, origin_y):
        """
        Save the metadata (origin, resolution, etc.) in a YAML file.
        """
        origin = [origin_x, origin_y, 0.0]
        yaml_data = f"""
image: {self.pgm_file}
resolution: {self.yaml_resolution}
origin: {origin}
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
        with open(self.yaml_file, 'w') as file:
            file.write(yaml_data)

    def process_map(self):
        """
        High-level method to process a KML file and generate a PGM and YAML file.
        """
        grid = self.create_occupancy_grid()
        padded_grid, self.min_utm_x, self.min_utm_y = self.add_padding(grid)
        self.save_pgm(padded_grid)
        # self.save_pgm(grid)
        self.save_yaml(0, 0)  # Origin at (0, 0)


# Main process example usage
kml_file1 = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon.kml'
pgm_file1 = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon_00.pgm'
yaml_file1 = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon_00.yaml'

kml_file2 = 'square_near_unisee.kml'
pgm_file2 = 'square_near_unisee_00.pgm'
yaml_file2 = 'square_near_unisee_00.yaml'

# Process map1
map2 = Map(kml_file1, pgm_file1, yaml_file1)
map2.process_map()

# Process map2
map1 = Map(kml_file2, pgm_file2, yaml_file2)
map1.process_map()


# Calculate shifted origin for map2 relative to map1
origin_x2, origin_y2 = map2.calculate_shifted_origin(map1)
map2.save_yaml(origin_x2, origin_y2)

kml_file3 = 'small_lake_near_unisee.kml'
pgm_file3 = 'small_lake_near_unisee_00.pgm'
yaml_file3 = 'small_lake_near_unisee_00.yaml'

map3 = Map(kml_file3, pgm_file3, yaml_file3)
map3.process_map()

# Calculate shifted origin for map2 relative to map1
origin_x3, origin_y3 = map3.calculate_shifted_origin(map1)
map3.save_yaml(origin_x3, origin_y3)

kml_file4 = 'test_area_port_of_bremen.kml'
pgm_file4 = 'test_area_port_of_bremen_00.pgm'
yaml_file4 = 'test_area_port_of_bremen_00.yaml'

map4 = Map(kml_file4, pgm_file4, yaml_file4)
map4.process_map()

# Calculate shifted origin for map2 relative to map1
origin_x4, origin_y4 = map4.calculate_shifted_origin(map1)
map4.save_yaml(origin_x4, origin_y4)


print("PGM and YAML files created successfully.")
