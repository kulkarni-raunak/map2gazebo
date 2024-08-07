#!/usr/bin/env python3

import cv2
import numpy as np
import trimesh
from matplotlib.tri import Triangulation
from rclpy.node import Node
from rclpy import logging
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter

class MapConverter(Node):
    def __init__(self):
        super().__init__('map2gazebo')

        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('occupied_thresh', 1)
        self.declare_parameter('box_height', 2.0)
        self.declare_parameter('mesh_type', 'stl')
        self.declare_parameter('export_dir', '/path/to/export/dir')

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.threshold = self.get_parameter('occupied_thresh').get_parameter_value().integer_value
        self.height = self.get_parameter('box_height').get_parameter_value().double_value
        self.mesh_type = self.get_parameter('mesh_type').get_parameter_value().string_value
        self.export_dir = self.get_parameter('export_dir').get_parameter_value().string_value

        self.test_map_pub = self.create_publisher(OccupancyGrid, 'test_map', 10)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            10
        )
        self.get_logger().info('map2gazebo node started')

    def map_callback(self, map_msg):
        self.get_logger().info('Received map')
        map_dims = (map_msg.info.height, map_msg.info.width)
        map_array = np.array(map_msg.data).reshape(map_dims)

        # Set all -1 (unknown) values to 0 (unoccupied)
        map_array[map_array < 0] = 0
        contours = self.get_occupied_regions(map_array)
        meshes = [self.contour_to_mesh(c, map_msg.info) for c in contours]

        corners = list(np.vstack(contours))
        corners = [c[0] for c in corners]
        self.publish_test_map(corners, map_msg.info, map_msg.header)
        mesh = trimesh.util.concatenate(meshes)

        # Export as STL or DAE
        if self.mesh_type == 'stl':
            with open(f"{self.export_dir}/map.stl", 'wb') as f:
                mesh.export(f, 'stl')
            self.get_logger().info('Exported STL. You can shut down this node now')
        elif self.mesh_type == 'dae':
            with open(f"{self.export_dir}/map.dae", 'wb') as f:
                f.write(trimesh.exchange.dae.export_collada(mesh))
            self.get_logger().info('Exported DAE. You can shut down this node now')

    def publish_test_map(self, points, metadata, map_header):
        """
        For testing purposes, publishes a map highlighting certain points.
        points is a list of tuples (x, y) in the map's coordinate system.
        """
        test_map = np.zeros((metadata.height, metadata.width))
        for x, y in points:
            test_map[y, x] = 100
        test_map_msg = OccupancyGrid()
        test_map_msg.header = map_header
        test_map_msg.header.stamp = self.get_clock().now().to_msg()
        test_map_msg.info = metadata
        test_map_msg.data = list(np.ravel(test_map))
        self.test_map_pub.publish(test_map_msg)

    def get_occupied_regions(self, map_array):
        """
        Get occupied regions of map
        """
        map_array = map_array.astype(np.uint8)
        _, thresh_map = cv2.threshold(
                map_array, self.threshold, 100, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(
                thresh_map, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        hierarchy = hierarchy[0]
        corner_idxs = [i for i in range(len(contours)) if hierarchy[i][3] == -1]
        return [contours[i] for i in corner_idxs]

    def contour_to_mesh(self, contour, metadata):
        height = np.array([0, 0, self.height])
        s3 = 3**0.5 / 3.
        meshes = []
        for point in contour:
            x, y = point[0]
            vertices = []
            new_vertices = [
                    coords_to_loc((x, y), metadata),
                    coords_to_loc((x, y+1), metadata),
                    coords_to_loc((x+1, y), metadata),
                    coords_to_loc((x+1, y+1), metadata)]
            vertices.extend(new_vertices)
            vertices.extend([v + height for v in new_vertices])
            faces = [[0, 2, 4],
                     [4, 2, 6],
                     [1, 2, 0],
                     [3, 2, 1],
                     [5, 0, 4],
                     [1, 0, 5],
                     [3, 7, 2],
                     [7, 6, 2],
                     [7, 4, 6],
                     [5, 4, 7],
                     [1, 5, 3],
                     [7, 3, 5]]
            mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
            if not mesh.is_volume:
                self.get_logger().debug('Fixing mesh normals')
                mesh.fix_normals()
            meshes.append(mesh)
        mesh = trimesh.util.concatenate(meshes)
        mesh.update_faces(mesh.unique_faces())
        return mesh

def coords_to_loc(coords, metadata):
    x, y = coords
    loc_x = x * metadata.resolution + metadata.origin.position.x
    loc_y = y * metadata.resolution + metadata.origin.position.y
    return np.array([loc_x, loc_y, 0.0])

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = MapConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()