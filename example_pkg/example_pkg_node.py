#! /usr/bin/env python3

import os
import django
import rclpy

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "example_django_project.settings")
django.setup()

from ros2_django.ros_node import ROS2DjangoNode  # noqa


def main(args=None):
    rclpy.init(args=args)
    node = ROS2DjangoNode(django_app="example_app", node_name="example_pkg_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Fail silently when interrupted, shutdown will show a message anyway
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
