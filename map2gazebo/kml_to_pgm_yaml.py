import xml.etree.ElementTree as ET
import numpy as np
import cv2
import math
import pyproj

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

def compute_centroid(polygon):
    """Compute the centroid of the polygon using the average of the latitudes and longitudes."""
    lons, lats = zip(*polygon)
    centroid_lon = sum(lons) / len(lons)
    centroid_lat = sum(lats) / len(lats)
    return centroid_lat, centroid_lon

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

    # Flip the grid vertically to correct for inversion
    grid = cv2.flip(grid, 0)

    return grid

def add_padding(grid, padding):
    """Add padding to the grid."""
    height, width = grid.shape
    if height > width:
        diff_in_size = height - width
        padded_grid = np.ones((height + 2*padding, width + diff_in_size + 2*padding), dtype=np.uint8) * 0  # 100 represents occupied space
        print(f"size of padded_grid {np.shape(padded_grid)}")
        # Place the original grid in the center of the padded grid
        padded_grid[padding:padding+height, padding + int(diff_in_size/2):padding+width+int(diff_in_size/2)] = grid        
    elif height < width:
        diff_in_size = width - height
        padded_grid = np.ones((height + diff_in_size + 2*padding, width + 2*padding), dtype=np.uint8) * 0  # 100 represents occupied space
        print(f"size of padded_grid {np.shape(padded_grid)}")
        # Place the original grid in the center of the padded grid
        padded_grid[padding + int(diff_in_size/2):padding+height + int(diff_in_size/2), padding:padding+width] = grid        
    else:
        padded_grid = np.ones((height + 2*padding, width + 2*padding), dtype=np.uint8) * 0  # 100 represents occupied space
        print(f"size of padded_grid {np.shape(padded_grid)}")
        # Place the original grid in the center of the padded grid
        padded_grid[padding:padding+height, padding:padding+width] = grid

    return padded_grid


# def pixel_to_meters(x, y, resolution):
#     """Convert pixel coordinates to meters."""
#     return x * resolution, y * resolution

# def calculate_origin(polygon, min_lat, min_lon, resolution, lat_scaling_factor, width, height, padding):
#     """Calculate the grid origin based on the centroid, and adjust for resolution and padding."""
#     # Compute the centroid of the polygon in lat/lon
#     centroid_lat, centroid_lon = compute_centroid(polygon)

#     # Convert the centroid to pixel coordinates
#     centroid_x, centroid_y = latlon_to_pixel(centroid_lat, centroid_lon, min_lat, min_lon, resolution, lat_scaling_factor)

#     # Adjust for padding (add padding to the pixel coordinates)
#     centroid_x += padding
#     centroid_y += padding

#     # Convert the centroid from pixel coordinates to meters
#     origin_x_meters, origin_y_meters = pixel_to_meters(centroid_x, centroid_y, resolution)

#     # Flip the y-axis for ROS, as pixel origin is at the top-left corner of the image
#     origin_y_meters = -(origin_y_meters)

#     return origin_x_meters, origin_y_meters

def latlon_to_utm(lat, lon):
    """Convert latitude and longitude to UTM coordinates using pyproj."""
    proj = pyproj.Proj(proj="utm", zone=32, ellps="WGS84")  # Example for UTM Zone 32 (adjust as needed)
    return proj(lon, lat)

def calculate_origin(polygon, min_lat, min_lon, resolution, lat_scaling_factor, padding):
    """Calculate the grid origin based on UTM coordinates of the bottom-left corner."""
    # Convert bottom-left corner (min_lat, min_lon) to UTM coordinates
    min_x_utm, min_y_utm = latlon_to_utm(min_lat, min_lon)

    # Convert pixel coordinates (bottom-left corner) to meters using resolution and padding
    origin_x_meters = min_x_utm - padding * resolution
    origin_y_meters = min_y_utm - padding * resolution

    return origin_x_meters, origin_y_meters

# Step 4: Save as PGM file
def save_pgm(grid, pgm_file):
    cv2.imwrite(pgm_file, grid)

# Step 5: Create and save the YAML file
def save_yaml(yaml_file, resolution, min_lat, min_lon, pgm_file, padding,  origin_x, origin_y, centroid_lat, centroid_lon):
    """Save the YAML configuration file."""
    # origin = [min_lon - padding * resolution, min_lat - padding * resolution, 0.0]  # [x, y, theta]
    origin = [origin_x, origin_y, 0.0]  # [x, y, theta]
    yaml_data = f"""
image: {pgm_file}
resolution: {resolution}
origin: {origin}
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
# centroid: [{centroid_lon}, {centroid_lat}, 0.0]
"""
    with open(yaml_file, 'w') as file:
        file.write(yaml_data)

# Main process
kml_file = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon.kml'
pgm_file = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon_00.pgm'
yaml_file = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon_00.yaml'
# kml_file = 'square_near_unisee.kml'
# pgm_file = 'square_near_unisee_00.pgm'
# yaml_file = 'square_near_unisee_00.yaml'
# resolution = 0.00001  # meters per pixel
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

# Calculate the origin in pixel coordinates (in meters, considering resolution and padding)
# origin_x, origin_y = calculate_origin(polygon, min_lat, min_lon, resolution, lat_scaling_factor, width, height, padding)
origin_x, origin_y = calculate_origin(polygon, min_lat, min_lon, resolution, lat_scaling_factor, padding)

# Compute the centroid of the polygon for setting the origin in the YAML
centroid_lat, centroid_lon = compute_centroid(polygon)

# Save PGM and YAML files
save_pgm(padded_grid, pgm_file)
save_yaml(yaml_file, resolution, min_lat, min_lon, pgm_file, padding, origin_x, origin_y, centroid_lat, centroid_lon)

print("PGM and YAML files created successfully.")
