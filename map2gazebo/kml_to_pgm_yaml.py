import xml.etree.ElementTree as ET
import numpy as np
import cv2
import math

# Step 1: Parse KML file and extract polygon coordinates
def parse_kml(kml_file):
    tree = ET.parse(kml_file)
    root = tree.getroot()

    # Namespace handling for KML
    namespaces = {'kml': 'http://www.opengis.net/kml/2.2'}
    coordinates = root.find('.//kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates', namespaces)
    coord_list = coordinates.text.strip().split()

    polygon = []
    for coord in coord_list:
        lon, lat, _ = map(float, coord.split(','))
        polygon.append((lon, lat))

    return polygon

# Step 2: Define the map dimensions and resolution
def get_map_bounds(polygon):
    lons, lats = zip(*polygon)
    min_lon, max_lon = min(lons), max(lons)
    min_lat, max_lat = min(lats), max(lats)
    return min_lon, max_lon, min_lat, max_lat

def latlon_to_pixel(lat, lon, min_lat, min_lon, resolution, lat_scaling_factor):
    x = int((lon - min_lon) / resolution)
    y = int((lat - min_lat) / resolution / lat_scaling_factor)
    # print(f"x: {x}, y: {y}")
    return x, y

# Step 3: Create the occupancy grid
def create_occupancy_grid(polygon, width, height, resolution, min_lat, min_lon, lat_scaling_factor):
    # Create a white image (occupied space) with size (width, height)
    grid = np.ones((height, width), dtype=np.uint8) * 0  # 100 represents occupied space

    print(f"min_lat: {min_lat}, min_lon: {min_lon}")

    # Convert polygon points to pixel coordinates
    poly_pixels = [latlon_to_pixel(lat, lon, min_lat, min_lon, resolution, lat_scaling_factor) for lon, lat in polygon]
    
    # # Ensure all coordinates are within bounds
    # poly_pixels = [(min(max(0, x), width-1), min(max(0, y), height-1)) for x, y in poly_pixels]

    # Draw the polygon on the grid using OpenCV
    poly_pixels = np.array(poly_pixels, dtype=np.int32)
    cv2.fillPoly(grid, [poly_pixels], color=255)  # 0 represents free space

    return grid

def add_padding(grid, padding):
    """Add padding to the grid."""
    height, width = grid.shape
    if height > width:
        diff_in_size = height - width
        padded_grid = np.ones((height + 2*padding, width + diff_in_size + 2*padding), dtype=np.uint8) * 0  # 100 represents occupied space
        # Place the original grid in the center of the padded grid
        padded_grid[padding:padding+height, padding + + int(diff_in_size/2):padding+width+int(diff_in_size/2)] = grid        
    elif height < width:
        diff_in_size = width - height
        padded_grid = np.ones((height + diff_in_size + 2*padding, width + 2*padding), dtype=np.uint8) * 0  # 100 represents occupied space
        print(f"size of padded_grid {np.shape(padded_grid)}")
        # Place the original grid in the center of the padded grid
        padded_grid[padding + int(diff_in_size/2):padding+height + int(diff_in_size/2), padding:padding+width] = grid        
    else:
        padded_grid = np.ones((height + 2*padding, width + 2*padding), dtype=np.uint8) * 0  # 100 represents occupied space
        # Place the original grid in the center of the padded grid
        padded_grid[padding:padding+height, padding:padding+width] = grid

    return padded_grid

# Step 4: Save as PGM file
def save_pgm(grid, pgm_file):
    cv2.imwrite(pgm_file, grid)

# Step 5: Create and save the YAML file
def save_yaml(yaml_file, resolution, min_lat, min_lon, pgm_file, padding):
    """Save the YAML configuration file."""
    origin = [min_lon - padding * resolution, min_lat - padding * resolution, 0.0]  # [x, y, theta]
    yaml_data = f"""
image: {pgm_file}
resolution: {resolution}
origin: {origin}
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
    """
    with open(yaml_file, 'w') as file:
        file.write(yaml_data)

# Main process
kml_file = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon.kml'
pgm_file = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon.pgm'
yaml_file = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon.yaml'
resolution = 0.00001  # meters per pixel
padding = 20

polygon = parse_kml(kml_file)
# print(f"polygon: {polygon}")
min_lon, max_lon, min_lat, max_lat = get_map_bounds(polygon)

# Calculate latitude scaling factor based on the average latitude
avg_lat = (min_lat + max_lat) / 2
lat_scaling_factor = math.cos(math.radians(avg_lat))

# Calculate width and height with a minimum value to ensure the image is not too small
width = max(int((max_lon - min_lon) / resolution), 10)
height = max(int((max_lat - min_lat) / resolution / lat_scaling_factor), 10)

# # Calculate width and height with a minimum value to ensure the image is not too small
# width = max(int((max_lon - min_lon) / resolution), 10)
# height = max(int((max_lat - min_lat) / resolution), 10)
print(f"width: {width} , height: {height}")

grid = create_occupancy_grid(polygon, width, height, resolution, min_lat, min_lon, lat_scaling_factor)
padded_grid = add_padding(grid, padding)
save_pgm(padded_grid, pgm_file)
save_yaml(yaml_file, resolution, min_lat, min_lon, pgm_file, padding)

print("PGM and YAML files created successfully.")
