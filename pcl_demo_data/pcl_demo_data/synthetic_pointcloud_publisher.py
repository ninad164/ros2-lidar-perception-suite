"""Publish a synthetic PointCloud2 scene for the perception pipeline."""

import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class SyntheticPointCloudPublisher(Node):
    """Publishes ground, car-like clusters, and sparse noise as XYZ points."""

    def __init__(self):
        super().__init__("synthetic_pointcloud_publisher")
        self.publisher = self.create_publisher(
            PointCloud2,
            "/kitti/point_cloud",
            10,
        )
        self.timer = self.create_timer(0.2, self.publish_cloud)
        self.tick = 0

        self.get_logger().info("Publishing synthetic PointCloud2 on /kitti/point_cloud")

    def publish_cloud(self):
        header = self.get_clock().now().to_msg()
        points = self.generate_scene()

        cloud_msg = point_cloud2.create_cloud_xyz32(
            self._make_header(header),
            points,
        )
        self.publisher.publish(cloud_msg)
        self.tick += 1

    def _make_header(self, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = "map"
        return header

    def generate_scene(self):
        points = []
        points.extend(self.generate_ground_plane())

        car_centers = [
            (8.0, -2.5, 0.8),
            (14.0, 2.0, 0.8),
            (20.0, -1.0, 0.8),
        ]
        for index, center in enumerate(car_centers):
            points.extend(self.generate_car_cluster(center, index))

        points.extend(self.generate_noise())
        return points

    def generate_ground_plane(self):
        points = []
        for x_index in range(-20, 61):
            x = x_index * 0.5
            for y_index in range(-18, 19):
                y = y_index * 0.5
                z = random.uniform(-0.015, 0.015)
                points.append((x, y, z))
        return points

    def generate_car_cluster(self, center, index):
        length = 4.4
        width = 1.9
        height = 1.6
        cx, cy, cz = center
        wobble = 0.08 * math.sin((self.tick * 0.15) + index)
        cx += wobble

        points = []
        samples_per_face = 180
        for _ in range(samples_per_face):
            local_x = random.uniform(-length / 2.0, length / 2.0)
            local_y = random.uniform(-width / 2.0, width / 2.0)
            local_z = random.uniform(-height / 2.0, height / 2.0)

            face = random.choice(("front", "rear", "left", "right", "top"))
            if face == "front":
                local_x = length / 2.0
            elif face == "rear":
                local_x = -length / 2.0
            elif face == "left":
                local_y = width / 2.0
            elif face == "right":
                local_y = -width / 2.0
            else:
                local_z = height / 2.0

            points.append(
                (
                    cx + local_x + random.uniform(-0.025, 0.025),
                    cy + local_y + random.uniform(-0.025, 0.025),
                    cz + local_z + random.uniform(-0.025, 0.025),
                )
            )

        return points

    def generate_noise(self):
        return [
            (
                random.uniform(-10.0, 35.0),
                random.uniform(-10.0, 10.0),
                random.uniform(0.0, 3.0),
            )
            for _ in range(120)
        ]


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticPointCloudPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
