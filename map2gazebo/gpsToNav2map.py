import pandas as pd
import numpy as np
from pyproj import CRS, Transformer
from PIL import Image, ImageDraw
import math
import argparse


def generate_map(
    csv_file="gps_data_unisee.csv",
    resolution=0.5,
    map_image_name="map_fixed",
    map_yaml_name="map.yaml",
):

    # Load GPS data from CSV
    data = pd.read_csv(csv_file)

    # Assuming the CSV has columns 'latitude' and 'longitude'
    gps_points = list(zip(data["latitude"], data["longitude"]))

    # Convert GPS to UTM using modern pyproj API
    wgs84 = CRS.from_epsg(4326)
    utm = CRS.from_epsg(32632)  # Bremen's UTM zone
    transformer = Transformer.from_crs(wgs84, utm, always_xy=True)
    utm_points = [transformer.transform(lon, lat) for lat, lon in gps_points]

    # Calculate the bounds and resolution
    min_x = min([p[0] for p in utm_points])
    min_y = min([p[1] for p in utm_points])
    max_x = max([p[0] for p in utm_points])
    max_y = max([p[1] for p in utm_points])
    # resolution = 0.50  # 50 cm per pixel

    # Calculate centroid
    centroid_x = sum([p[0] for p in utm_points]) / len(utm_points)
    centroid_y = sum([p[1] for p in utm_points]) / len(utm_points)

    # Sort points based on angle with centroid
    def angle_from_centroid(point):
        return math.atan2(point[1] - centroid_y, point[0] - centroid_x)

    utm_points_sorted = sorted(utm_points, key=angle_from_centroid)

    # Add padding to prevent out-of-bounds errors
    padding = 10  # You can adjust this value
    width = int((max_x - min_x) / resolution) + 2 * padding
    height = int((max_y - min_y) / resolution) + 2 * padding
    image = Image.new("L", (width, height), 0)  # black background (obstacle)
    draw = ImageDraw.Draw(image)

    # Convert UTM points to pixel coordinates for the polygon
    # Note: No need to flip y-axis as the top-left is the origin (consistent with image orientation)
    polygon_points = [
        (
            int((x - min_x) / resolution) + padding,
            int((y - min_y) / resolution) + padding,
        )
        for x, y in utm_points_sorted
    ]

    # Draw and fill the polygon as unexplored region (mid-gray)
    draw.polygon(polygon_points, fill=205)  # 205 for unexplored

    # Save the map
    image.save(f"{map_image_name}.pgm")
    image.save(f"{map_image_name}.png", "png")

    # Create the map.yaml file
    with open(map_yaml_name, "w") as f:
        f.write(f"image: {map_image_name}.pgm\n")
        f.write(f"resolution: {resolution}\n")
        f.write(
            f"origin: [{min_x - padding * resolution}, {min_y - padding * resolution}, 0.0]\n"
        )
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
        f.write("negate: 0\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a map from GPS points.")
    parser.add_argument(
        "--csv_file",
        type=str,
        default="gps_data_unisee.csv",
        help="Input CSV file containing latitude and longitude.",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.05,
        help="Resolution of the map in meters per pixel.",
    )
    parser.add_argument(
        "--map_image_name",
        type=str,
        default="map_fixed.pgm",
        help="Output map image file name.",
    )
    parser.add_argument(
        "--map_yaml_name",
        type=str,
        default="map.yaml",
        help="Output map YAML file name.",
    )

    args = parser.parse_args()

    generate_map(
        args.csv_file, args.resolution, args.map_image_name, args.map_yaml_name
    )

