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

def latlon_to_utm(lon, lat):
    """Convert latitude and longitude to UTM coordinates using pyproj."""
    proj = pyproj.Proj(proj="utm", zone=32, ellps="WGS84")  # Adjust UTM zone as needed
    return proj(lon, lat)

def calculate_centroid_utm(polygon):
    """Calculate the centroid of the polygon in UTM coordinates."""
    utm_coords = [latlon_to_utm(lon, lat) for lon, lat in polygon]
    centroid_x = sum([p[0] for p in utm_coords]) / len(utm_coords)
    centroid_y = sum([p[1] for p in utm_coords]) / len(utm_coords)
    return centroid_x, centroid_y

# def calculate_shifted_origin(centroid1, centroid2, resolution, padding):
def calculate_shifted_origin(centroid1, centroid2, resolution, lat_scaling_factor, lon_scaling_factor):
    """Calculate the shifted origin for the second polygon relative to the first."""
    # Compute the shift between the centroids (in meters)
    shift_x_lon = centroid2[0] - centroid1[0]
    shift_y_lat = centroid2[1] - centroid1[1]

    # Convert the shift to pixel units based on the resolution
    # shift_x_pixels = shift_x_lon / resolution / lon_scaling_factor
    # shift_y_pixels = shift_y_lat / resolution / lat_scaling_factor
    shift_x_pixels = shift_x_lon * resolution / lat_scaling_factor
    shift_y_pixels = shift_y_lat * resolution / lon_scaling_factor

    # # Adjust for padding (shift origin by padding pixels)
    # origin_x_shifted = -shift_x_pixels + padding
    # origin_y_shifted = -shift_y_pixels + padding

    # return origin_x_shifted * resolution, origin_y_shifted * resolution
    return shift_x_pixels, shift_y_pixels

def scaling_factor(min, max):
    """Calculate the scaling factor based on the latitude or longitude."""
    avg = (min + max) / 2
    return math.cos(math.radians(avg))


def latlon_to_pixel(lat, lon, min_lat, min_lon, resolution, lat_scaling_factor, lon_scaling_factor):
    x = int((lon - min_lon) / resolution / lon_scaling_factor)
    y = int((lat - min_lat) / resolution / lat_scaling_factor)
    # print(f"x: {x}, y: {y}")
    return x, y

# Step 3: Create the occupancy grid
def create_occupancy_grid(polygon, width, height, resolution, min_lat, min_lon, lat_scaling_factor, lon_scaling_factor):
    # Create a white image (occupied space) with size (width, height)
    grid = np.ones((height, width), dtype=np.uint8) * 0  # 100 represents occupied space

    print(f"min_lat: {min_lat}, min_lon: {min_lon}")

    # Convert polygon points to pixel coordinates
    poly_pixels = [latlon_to_pixel(lat, lon, min_lat, min_lon, resolution, lat_scaling_factor, lon_scaling_factor) for lon, lat in polygon]
    
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

# Step 4: Save as PGM file
def save_pgm(grid, pgm_file):
    cv2.imwrite(pgm_file, grid)

# Step 5: Create and save the YAML file
# def save_yaml(yaml_file, resolution, min_lat, min_lon, pgm_file, padding,  origin_x, origin_y, centroid_lat, centroid_lon):
def save_yaml(yaml_file, resolution, origin_x, origin_y, pgm_file):
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
"""
    with open(yaml_file, 'w') as file:
        file.write(yaml_data)

# Main process
kml_file = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon.kml'
pgm_file = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon_00.pgm'
yaml_file = 'src/submodules/katamaran_nav2_bt/maps/unisee_bremen_polygon_00.yaml'
kml_file_2 = 'square_near_unisee.kml'
pgm_file_2 = 'square_near_unisee_00.pgm'
yaml_file_2 = 'square_near_unisee_00.yaml'
# resolution = 0.00001  # meters per pixel
# Define two different resolutions
calculation_resolution = 0.00001  # meters per pixel for map generation (finer)
yaml_resolution = 0.01  # meters per pixel for ROS2 usage (coarser)
padding = 20

# Load polygons from KML files (two different KML files)
polygon1 = parse_kml(kml_file)
polygon2 = parse_kml(kml_file_2)

# Calculate the centroids of both polygons in UTM coordinates
centroid1_utm = calculate_centroid_utm(polygon1)
centroid2_utm = calculate_centroid_utm(polygon2)

# # Set the origin of the first map such that its centroid is at (0, 0) in RViz
min_lon, max_lon, min_lat, max_lat = get_map_bounds(polygon1)

# Calculate latitude & Longitude scaling factor based on the average latitude and average longitude
lat_scaling_factor = scaling_factor(min_lat, max_lat)
lon_scaling_factor = scaling_factor(min_lon, max_lon)

# Calculate width and height with a minimum value to ensure the image is not too small
width = max(int((max_lon - min_lon) / calculation_resolution  / lon_scaling_factor), 10)
height = max(int((max_lat - min_lat) / calculation_resolution / lat_scaling_factor), 10)

# Create occupancy grid for the first map and add padding
grid1 = create_occupancy_grid(polygon1, width, height, calculation_resolution, min_lat, min_lon, lat_scaling_factor, lon_scaling_factor)
padded_grid1 = add_padding(grid1, padding)

# Set the origin of the first map such that its centroid is at (0, 0) in RViz
origin_x1 = -np.shape(padded_grid1)[0] * yaml_resolution / 2  # Origin at (0, 0) in RViz grid
origin_y1 = -np.shape(padded_grid1)[1] * yaml_resolution / 2

# Save the first PGM and YAML files
save_pgm(padded_grid1, pgm_file)
save_yaml(yaml_file, yaml_resolution, origin_x1, origin_y1, pgm_file)

# Create occupancy grid for the second map and add padding
min_lon2, max_lon2, min_lat2, max_lat2 = get_map_bounds(polygon2)

# Calculate latitude scaling factor based on the average latitude
lat_scaling_factor2 = scaling_factor(min_lat2, max_lat2)
lon_scaling_factor2 = scaling_factor(min_lon2, max_lon2)

# Calculate the shifted origin for the second polygon based on the first polygon's centroid
# origin_x2, origin_y2 = calculate_shifted_origin(centroid1_utm, centroid2_utm, yaml_resolution, padding)
origin_x2, origin_y2 = calculate_shifted_origin(centroid1_utm, centroid2_utm, yaml_resolution, lat_scaling_factor2, lon_scaling_factor2)

# Calculate width and height with a minimum value to ensure the image is not too small
width2 = max(int((max_lon2 - min_lon2) / calculation_resolution  / lon_scaling_factor2), 10)
height2 = max(int((max_lat2 - min_lat2) / calculation_resolution / lat_scaling_factor2), 10)

grid2 = create_occupancy_grid(polygon2, width2, height2, calculation_resolution, min_lat2, min_lon2, lat_scaling_factor2, lon_scaling_factor2)
padded_grid2 = add_padding(grid2, padding)

# Save the second PGM and YAML files with the adjusted origin
save_pgm(padded_grid2, pgm_file_2)
save_yaml(yaml_file_2, yaml_resolution, origin_x2, origin_y2, pgm_file_2)

print("PGM and YAML files created successfully.")
