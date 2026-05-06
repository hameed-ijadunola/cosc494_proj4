#!/usr/bin/env python3

from __future__ import print_function

import errno

import networkx as nx
import numpy as np
import os
import rclpy
import tf2_ros
import time

from geometry_msgs.msg import PoseStamped, PoseArray, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from std_msgs.msg import Header

from proj4 import utils
from control_interfaces.srv import FollowPath
from proj4 import search
from proj4.problems import SE2Problem
from proj4.roadmap import Roadmap
from proj4.samplers import samplers


class PlannerROS:
    def __init__(
        self,
        node,
        num_vertices,
        connection_radius,
        curvature,
        do_shortcut=True,
        sampler_type="halton",
        cache_roadmap=False,
        tf_prefix="",
        num_goals=1,
        tf_listener=None,
    ):
        """Motion planning ROS wrapper.

        Args:
            num_vertices: number of vertices in the graph
            connection_radius: radius for connecting vertices
            curvature: curvature for Dubins paths
            do_shortcut: whether to shortcut the planned path
            sampler_type: name of the sampler to construct
            cache_roadmap: whether to cache the constructed roadmap
        """
        self.node = node
        self.tf_prefix = tf_prefix

        if tf_listener:
            self.tf_buffer = tf_listener
        else:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer,
                self.node,
            )

        self.num_vertices = num_vertices
        self.connection_radius = connection_radius
        self.do_shortcut = do_shortcut

        self.multi_goals = num_goals > 1
        self.num_goals = num_goals

        if self.multi_goals:
            print("Accept multiple goals")
            self.goals = []
            self.route_sent = False

        self.permissible_region, self.map_info = utils.get_map(self.node, "/map_server/map")

        self.problem = SE2Problem(
            self.permissible_region,
            map_info=self.map_info,
            check_resolution=0.1,
            curvature=curvature,
        )

        self.rm = None

        self.goal_sub = self.node.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.get_goal,
            num_goals,
        )

        self.nodes_viz = self.node.create_publisher(
            PoseArray,
            "~/vertices",
            1,
        )

        self.edges_viz = self.node.create_publisher(
            Marker,
            "~/edges",
            1,
        )

        self.path_viz = self.node.create_publisher(
            Path,
            "/planned_path",
            1,
        )

        self.controller = self.node.create_client(
            FollowPath,
            "/control_node/follow_path",
        )

        # Load or construct a roadmap
        sampler = samplers[sampler_type](self.problem.extents)

        self.node.get_logger().info("Constructing roadmap...")

        saveto = None

        if cache_roadmap:
            saveto = graph_location(
                node,
                "se2",
                sampler_type,
                num_vertices,
                connection_radius,
                curvature,
            )

            self.node.get_logger().info("Cached at: {}".format(saveto))

        start_stamp = time.time()

        self.rm = Roadmap(
            self.problem,
            sampler,
            num_vertices,
            connection_radius,
            lazy=True,
            saveto=saveto,
        )

        load_time = time.time() - start_stamp

        self.node.get_logger().info("Roadmap constructed in {:2.2f}s".format(load_time))

        self.visualize()

    def plan_to_goal(self, start, goal):
        """Return a planned path from start to goal."""
        # Add current pose and goal to the planning env
        self.node.get_logger().info("Adding start and goal node")

        try:
            start_id = self.rm.add_node(start, is_start=True)
            goal_id = self.rm.add_node(goal, is_start=False)

        except ValueError:
            self.node.get_logger().info("Either start or goal was in collision")
            return None

        # Call the Lazy A* planner
        try:
            self.node.get_logger().info("Planning...")

            start_edges_evaluated = self.rm.edges_evaluated
            start_time = time.time()

            path, _ = search.astar(self.rm, start_id, goal_id)

            end_time = time.time()

            edges_evaluated = self.rm.edges_evaluated - start_edges_evaluated

            self.node.get_logger().info("Path length: {}".format(self.rm.compute_path_length(path)))

            self.node.get_logger().info("Planning time: {}".format(end_time - start_time))

            self.node.get_logger().info("Edges evaluated: {}".format(edges_evaluated))

        except nx.NetworkXNoPath:
            self.node.get_logger().info("Failed to find a plan")
            return None

        # Shortcut if necessary
        if self.do_shortcut:
            self.node.get_logger().info("Shortcutting path...")

            start_time = time.time()

            path = search.shortcut(self.rm, path)

            end_time = time.time()

            self.node.get_logger().info("Shortcut length: {}".format(self.rm.compute_path_length(path)))

            self.node.get_logger().info("Shortcut time: {}".format(end_time - start_time))

        # Return interpolated path
        return self.rm.compute_qpath(path)

    def plan_multi_goals(self, start):
        world_points = []

        for i, goal in enumerate(self.goals):
            print("Plan from {} to {}".format(start, goal))

            path = self.plan_to_goal(start, goal)

            if path is None:
                print("Failed to find a plan")
                return None

            world_points += [path]

            start = goal

        print("got a plan")

        world_points = np.concatenate(world_points, axis=0)

        return world_points

    def get_goal(self, msg):
        """Goal callback function."""
        if self.rm is None:
            return False

        self.goal = np.array(utils.pose_to_particle(msg.pose))

        start = self._get_car_pose()

        path_states = None

        if self.multi_goals:
            if self.route_sent:
                self.route_sent = False
                self.goals = []

            self.goals += [self.goal.copy()]

            if len(self.goals) == self.num_goals:
                print("Got the final goal for mutli goal. Planning for multiple goals")

                path_states = self.plan_multi_goals(start)

            else:
                print(
                    "Got {}/{} goals.".format(
                        len(self.goals),
                        self.num_goals,
                    )
                )

        else:
            path_states = self.plan_to_goal(start, self.goal)

        if path_states is None:
            return False

        return self.send_path(path_states)

    def _get_car_pose(self):
        """Return the current vehicle state."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                self.tf_prefix + "base_footprint",
                rclpy.time.Time(),
            )

            # Drop stamp header
            transform = transform.transform

            return np.array(
                [
                    transform.translation.x,
                    transform.translation.y,
                    utils.quaternion_to_angle(transform.rotation),
                ]
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:

            self.node.get_logger().warn(str(e))

            return None

    def send_path(self, path_states):
        """Send a planned path to the controller."""
        h = Header()
        h.stamp = rclpy.time.Time().to_msg()  # important: use latest TF
        h.frame_id = "map"

        desired_speed = 1.0

        path = Path()
        path.header = h
        path.poses = [
            PoseStamped(
                header=h,
                pose=utils.particle_to_pose(state),
            )
            for state in path_states
        ]

        self.controller.wait_for_service()

        self.node.get_logger().info(f"Publishing planned path with {len(path.poses)} poses")
        self.path_viz.publish(path)

        req = FollowPath.Request()
        req.path = path
        req.speed = float(desired_speed)

        future = self.controller.call_async(req)

        future.add_done_callback(lambda f: self.node.get_logger().info(f"ContPath setroller response: {f.result()}"))

        return future

    def visualize(self):
        """Visualize the nodes and edges of the roadmap."""
        vertices = self.rm.vertices.copy()

        poses = list(map(utils.particle_to_pose, vertices))

        msg = PoseArray(
            header=Header(frame_id="map"),
            poses=poses,
        )

        self.nodes_viz.publish(msg)

        # There are lots of edges, so we'll shuffle and incrementally visualize
        # them. This is more work overall, but gives more immediate feedback
        # about the graph.
        all_edges = np.empty((0, 2), dtype=int)

        edges = np.array(self.rm.graph.edges(), dtype=int)

        np.random.shuffle(edges)

        split_indices = np.array(
            [500, 1000, 2000, 5000] + [10000, 20000, 50000] + list(range(100000, edges.shape[0], 100000)),
            dtype=int,
        )

        for batch in np.split(edges, split_indices, axis=0):
            batch_edges = []

            for u, v in batch:
                q1 = self.rm.vertices[u, :]
                q2 = self.rm.vertices[v, :]

                # Check edge validity on the problem rather than roadmap to
                # circumvent edge collision-checking count
                if not self.rm.problem.check_edge_validity(q1, q2):
                    continue

                edge, _ = self.problem.steer(
                    q1,
                    q2,
                    resolution=0.25,
                    interpolate_line=False,
                )

                with_repeats = np.repeat(
                    edge[:, :2],
                    2,
                    0,
                ).reshape(
                    -1, 2
                )[1:-1]

                batch_edges.append(with_repeats)

            if not batch_edges:
                continue

            batch_edges = np.vstack(batch_edges)

            all_edges = np.vstack((all_edges, batch_edges))

            points = list(
                map(
                    lambda x: Point(
                        x=x[0],
                        y=x[1],
                        z=-1.0,
                    ),
                    all_edges,
                )
            )

            msg = Marker(
                header=Header(frame_id="map"),
                type=Marker.LINE_LIST,
                points=points,
            )

            msg.scale.x = 0.01
            msg.pose.orientation.w = 1.0
            msg.color.a = 0.1

            self.edges_viz.publish(msg)


def mkdir_p(path):
    """Equivalent to mkdir -p path."""
    try:
        os.makedirs(path)

    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise


def graph_location(
    node,
    problem_name,
    sampler_name,
    num_vertices,
    connection_radius,
    curvature=None,
    map_name=None,
):
    """Return the name of this graph in the cache."""
    cache_dir = os.path.expanduser("~/.ros/graph_cache/")

    mkdir_p(cache_dir)

    if map_name is None:
        map_file = node.get_parameter("map_file").value
        map_name, _ = os.path.splitext(os.path.basename(map_file))

    params = [
        map_name,
        problem_name,
        sampler_name,
        num_vertices,
        connection_radius,
    ]

    if problem_name == "se2":
        params.append(curvature)

    name = "_".join(str(p) for p in params)

    return os.path.join(cache_dir, name + ".pkl")
