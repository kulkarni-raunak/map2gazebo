import xml.etree.ElementTree as ET
import numpy as np
import cv2
import math
import pyproj

class Map:
    def __init__(self, kml_file, pgm_file, yaml_file, resolution=0.00001, yaml_resolution=0.01, padding=20):
        self.kml_file = kml_file
        self.pgm_file = pgm_file
        self.yaml_file = yaml_file
        self.resolution = resolution
        self.yaml_resolution = yaml_resolution
        self.padding = padding
        self.polygon = self.parse_kml(self.kml_file)
        self.min_lon, self.max_lon, self.min_lat, self.max_lat = self.get_map_bounds(self.polygon)
        self.lat_scaling_factor = self.scaling_factor(self.min_lat, self.max_lat)
        self.lon_scaling_factor = self.scaling_factor(self.min_lon, self.max_lon)
        self.width = max(int((self.max_lon - self.min_lon) / self.resolution / self.lon_scaling_factor), 10)
        self.height = max(int((self.max_lat - self.min_lat) / self.resolution / self.lat_scaling_factor), 10)

    def parse_kml(self, kml_file):
        tree = ET.parse(kml_file)
        root = tree.getroot()
        namespaces = {'kml': 'http://www.opengis.net/kml/2.2'}
        coordinates = root.find('.//kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates', namespaces)
        coord_list = coordinates.text.strip().split()
        polygon = [(float(coord.split(',')[0]), float(coord.split(',')[1])) for coord in coord_list]
        return polygon

    def get_map_bounds(self, polygon):
        lons, lats = zip(*polygon)
        return min(lons), max(lons), min(lats), max(lats)

    def scaling_factor(self, min_val, max_val):
        avg = (min_val + max_val) / 2
        return math.cos(math.radians(avg))

    def latlon_to_pixel(self, lat, lon):
        x = int((lon - self.min_lon) / self.resolution / self.lon_scaling_factor)
        y = int((lat - self.min_lat) / self.resolution / self.lat_scaling_factor)
        return x, y

    def pixel_to_latlon(self, pixel_x, pixel_y):
        lon = self.min_lon + (pixel_x * self.resolution * self.lon_scaling_factor)
        lat = self.min_lat + (pixel_y * self.resolution * self.lat_scaling_factor)
        return lon, lat

    def create_occupancy_grid(self):
        grid = np.ones((self.height, self.width), dtype=np.uint8) * 0
        poly_pixels = [self.latlon_to_pixel(lat, lon) for lon, lat in self.polygon]
        poly_pixels = np.array(poly_pixels, dtype=np.int32)
        cv2.fillPoly(grid, [poly_pixels], color=255)
        grid = cv2.flip(grid, 0)
        return grid

    def add_padding(self, grid):
        height, width = grid.shape
        if height > width:
            diff_in_size = height - width
            padded_grid = np.ones((height + 2 * self.padding, width + diff_in_size + 2 * self.padding), dtype=np.uint8) * 0
            padded_grid[self.padding:self.padding + height, self.padding + int(diff_in_size / 2):self.padding + width + int(diff_in_size / 2)] = grid
            new_min_lon, new_min_lat = self.pixel_to_latlon(-(self.padding + int(diff_in_size / 2)), -self.padding)
        elif height < width:
            diff_in_size = width - height
            padded_grid = np.ones((height + diff_in_size + 2 * self.padding, width + 2 * self.padding), dtype=np.uint8) * 0
            padded_grid[self.padding + int(diff_in_size / 2):self.padding + height + int(diff_in_size / 2), self.padding:self.padding + width] = grid
            new_min_lon, new_min_lat = self.pixel_to_latlon(-self.padding, -(self.padding + int(diff_in_size / 2)))
        else:
            padded_grid = np.ones((height + 2 * self.padding, width + 2 * self.padding), dtype=np.uint8) * 0
            padded_grid[self.padding:self.padding + height, self.padding:self.padding + width] = grid
            new_min_lon, new_min_lat = self.pixel_to_latlon(-self.padding, -self.padding)
        return padded_grid, new_min_lon, new_min_lat

    def save_pgm(self, grid):
        cv2.imwrite(self.pgm_file, grid)

    def save_yaml(self, origin_x, origin_y):
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

    def calculate_shifted_origin(self, other_map):
        origin_x2, origin_y2 = 0, 0
        lon2, lat2 = self.pixel_to_latlon(origin_x2, origin_y2)
        origin_x2, origin_y2 = other_map.latlon_to_pixel(lat2, lon2)
        origin_x2 *= self.yaml_resolution
        origin_y2 *= self.yaml_resolution
        return origin_x2, origin_y2

    def process_map(self):
        grid = self.create_occupancy_grid()
        padded_grid, self.min_lon, self.min_lat = self.add_padding(grid)
        self.save_pgm(padded_grid)
        origin_x, origin_y = 0, 0  # This can be modified as needed
        self.save_yaml(origin_x, origin_y)


# Example usage:
kml_file1 = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon.kml'
pgm_file1 = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon_00.pgm'
yaml_file1 = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon_00.yaml'

map1 = Map(kml_file1, pgm_file1, yaml_file1)
map1.process_map()

kml_file2 = 'square_near_unisee.kml'
pgm_file2 = 'square_near_unisee_00.pgm'
yaml_file2 = 'square_near_unisee_00.yaml'

map2 = Map(kml_file2, pgm_file2, yaml_file2)
map2.process_map()

# Calculate shifted origin for map2 relative to map1
origin_x2, origin_y2 = map2.calculate_shifted_origin(map1)
map2.save_yaml(origin_x2, origin_y2)

print("PGM and YAML files created successfully.")
