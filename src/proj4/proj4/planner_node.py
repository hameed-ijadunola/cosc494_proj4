#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from proj4.planner_ros import PlannerROS


def main(args=None):
    rclpy.init(args=args)

    node = Node("planner")

    node.declare_parameter("num_vertices", 100)
    node.declare_parameter("connection_radius", 2.0)
    node.declare_parameter("curvature", 1.0)
    node.declare_parameter("tf_prefix", "")
    node.declare_parameter("cache_roadmap", False)
    node.declare_parameter("num_goals", 1)

    PlannerROS(
        node=node,
        num_vertices=node.get_parameter("num_vertices").value,
        connection_radius=float(node.get_parameter("connection_radius").value),
        curvature=float(node.get_parameter("curvature").value),
        tf_prefix=node.get_parameter("tf_prefix").value,
        cache_roadmap=bool(node.get_parameter("cache_roadmap").value),
        num_goals=node.get_parameter("num_goals").value,
    )

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
