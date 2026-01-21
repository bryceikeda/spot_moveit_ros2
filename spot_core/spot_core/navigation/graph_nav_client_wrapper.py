from __future__ import annotations

from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from spot_core.spot_core.spot_commander import (
        SpotCommander,
    )

from google.protobuf import wrappers_pb2
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.api.graph_nav import map_pb2, nav_pb2
from bosdyn.client.exceptions import ResponseError
import time
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2


class GraphNavClientWrapper:
    """A ROS 2 node providing navigation for Spot."""

    def __init__(
        self, graph_nav_client, state_client, filepath="../../downloaded_graph"
    ):
        self._graph_nav_client = graph_nav_client
        self._state_client = state_client
        self._upload_filepath = filepath

    def navigate_to_waypoint_blocking(self, waypoint: str):
        """Navigate to a specific waypoint."""
        destination_waypoint = self.find_unique_waypoint_id(
            waypoint, self._current_graph, self._current_annotation_name_to_wp_id
        )
        if not destination_waypoint:
            # Failed to find the appropriate unique waypoint id for the navigation command.
            return
        nav_to_cmd_id = None
        # Navigate to the destination waypoint.
        is_finished = False
        while not is_finished:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            try:
                nav_to_cmd_id = self._graph_nav_client.navigate_to(
                    destination_waypoint, 1.0, command_id=nav_to_cmd_id
                )
            except ResponseError as e:
                print(f"Error while navigating {e}")
                break
            time.sleep(0.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            is_finished = self._check_success(nav_to_cmd_id)

    def _check_success(self, command_id=-1):
        """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
        if command_id == -1:
            # No command, so we have no status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL
        ):
            # Successfully completed the navigation commands!
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            print(
                "Robot got lost when navigating the route, the robot will now sit down."
            )
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            print(
                "Robot got stuck when navigating the route, the robot will now sit down."
            )
            return True
        elif (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED
        ):
            print("Robot is impaired.")
            return True
        else:
            # Navigation command is not complete yet.
            return False

    def _list_graph_waypoint_and_edge_ids(self, *args):
        """List the waypoint ids and edge ids of the graph currently on the robot."""

        # Download current graph
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            print("Empty graph.")
            return
        self._current_graph = graph

        localization_id = (
            self._graph_nav_client.get_localization_state().localization.waypoint_id
        )

        # Update and print waypoints and edges
        self._current_annotation_name_to_wp_id, self._current_edges = (
            self.update_waypoints_and_edges(graph, localization_id)
        )

    def _set_initial_localization_fiducial(self):
        """Trigger localization when near a fiducial."""
        robot_state = self._state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot
        ).to_proto()
        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            ko_tform_body=current_odom_tform_body,
        )

    def _upload_graph_and_snapshots(self):
        """Upload the graph and snapshots to the robot."""
        print("Loading the graph from disk into local storage...")
        with open(self._upload_filepath + "/graph", "rb") as graph_file:
            # Load the graph from disk.
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            # print(
            #    f'Loaded graph has {len(self._current_graph.waypoints)} waypoints and {self._current_graph.edges} edges'
            # )
        for waypoint in self._current_graph.waypoints:
            # Load the waypoint snapshots from disk.
            with open(
                f"{self._upload_filepath}/waypoint_snapshots/{waypoint.snapshot_id}",
                "rb",
            ) as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[waypoint_snapshot.id] = (
                    waypoint_snapshot
                )
        for edge in self._current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            # Load the edge snapshots from disk.
            with open(
                f"{self._upload_filepath}/edge_snapshots/{edge.snapshot_id}", "rb"
            ) as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        # Upload the graph to the robot.
        print("Uploading the graph and snapshots to the robot...")
        true_if_empty = not len(self._current_graph.anchoring.anchors)
        response = self._graph_nav_client.upload_graph(
            graph=self._current_graph, generate_new_anchoring=true_if_empty
        )
        # Upload the snapshots to the robot.
        for snapshot_id in response.unknown_waypoint_snapshot_ids:
            waypoint_snapshot = self._current_waypoint_snapshots[snapshot_id]
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            # print(f'Uploaded {waypoint_snapshot.id}')
        for snapshot_id in response.unknown_edge_snapshot_ids:
            edge_snapshot = self._current_edge_snapshots[snapshot_id]
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            # print(f'Uploaded {edge_snapshot.id}')

        # The upload is complete! Check that the robot is localized to the graph,
        # and if it is not, prompt the user to localize the robot before attempting
        # any navigation commands.
        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            # The robot is not localized to the newly uploaded graph.
            print("Upload complete!")

    def id_to_short_code(self, id):
        """Convert a unique id to a 2 letter short code."""
        tokens = id.split("-")
        if len(tokens) > 2:
            return f"{tokens[0][0]}{tokens[1][0]}"
        return None

    def pretty_print_waypoints(
        self, waypoint_id, waypoint_name, short_code_to_count, localization_id
    ):
        short_code = self.id_to_short_code(waypoint_id)
        if short_code is None or short_code_to_count[short_code] != 1:
            short_code = "  "  # If the short code is not valid/unique, don't show it.

        waypoint_symbol = "->" if localization_id == waypoint_id else "  "
        print(
            f"{waypoint_symbol} Waypoint name: {waypoint_name} id: {waypoint_id} short code: {short_code}"
        )

    def find_unique_waypoint_id(self, short_code, graph, name_to_id):
        """Convert either a 2 letter short code or an annotation name into the associated unique id."""
        if graph is None:
            print(
                "Please list the waypoints in the map before trying to navigate to a specific one (Option #4)."
            )
            return

        if len(short_code) != 2:
            # Not a short code, check if it is an annotation name (instead of the waypoint id).
            if short_code in name_to_id:
                # Short code is a waypoint's annotation name. Check if it is paired with a unique waypoint id.
                if name_to_id[short_code] is not None:
                    # Has an associated waypoint id!
                    return name_to_id[short_code]
                else:
                    print(
                        f"The waypoint name {short_code} is used for multiple different unique waypoints. Please use "
                        f"the waypoint id."
                    )
                    return None
            # Also not a waypoint annotation name, so we will operate under the assumption that it is a
            # unique waypoint id.
            return short_code

        ret = short_code
        for waypoint in graph.waypoints:
            if short_code == self.id_to_short_code(waypoint.id):
                if ret != short_code:
                    return short_code  # Multiple waypoints with same short code.
                ret = waypoint.id
        return ret

    def update_waypoints_and_edges(self, graph, localization_id, do_print=True):
        """Update and print waypoint ids and edge ids."""
        name_to_id = dict()
        edges = dict()

        short_code_to_count = {}
        waypoint_to_timestamp = []
        for waypoint in graph.waypoints:
            # Determine the timestamp that this waypoint was created at.
            timestamp = -1.0
            try:
                timestamp = (
                    waypoint.annotations.creation_time.seconds
                    + waypoint.annotations.creation_time.nanos / 1e9
                )
            except:
                # Must be operating on an older graph nav map, since the creation_time is not
                # available within the waypoint annotations message.
                pass
            waypoint_to_timestamp.append(
                (waypoint.id, timestamp, waypoint.annotations.name)
            )

            # Determine how many waypoints have the same short code.
            short_code = self.id_to_short_code(waypoint.id)
            if short_code not in short_code_to_count:
                short_code_to_count[short_code] = 0
            short_code_to_count[short_code] += 1

            # Add the annotation name/id into the current dictionary.
            waypoint_name = waypoint.annotations.name
            if waypoint_name:
                if waypoint_name in name_to_id:
                    # Waypoint name is used for multiple different waypoints, so set the waypoint id
                    # in this dictionary to None to avoid confusion between two different waypoints.
                    name_to_id[waypoint_name] = None
                else:
                    # First time we have seen this waypoint annotation name. Add it into the dictionary
                    # with the respective waypoint unique id.
                    name_to_id[waypoint_name] = waypoint.id

        # Sort the set of waypoints by their creation timestamp. If the creation timestamp is unavailable,
        # fallback to sorting by annotation name.
        waypoint_to_timestamp = sorted(
            waypoint_to_timestamp, key=lambda x: (x[1], x[2])
        )

        # Print out the waypoints name, id, and short code in an ordered sorted by the timestamp from
        # when the waypoint was created.
        if do_print:
            print(f"{len(graph.waypoints):d} waypoints:")
            for waypoint in waypoint_to_timestamp:
                self.pretty_print_waypoints(
                    waypoint[0], waypoint[2], short_code_to_count, localization_id
                )

        for edge in graph.edges:
            if edge.id.to_waypoint in edges:
                if edge.id.from_waypoint not in edges[edge.id.to_waypoint]:
                    edges[edge.id.to_waypoint].append(edge.id.from_waypoint)
            else:
                edges[edge.id.to_waypoint] = [edge.id.from_waypoint]
            if do_print:
                print(
                    f"(Edge) from waypoint {edge.id.from_waypoint} to waypoint {edge.id.to_waypoint} "
                    f"(cost {edge.annotations.cost.value})"
                )

        return name_to_id, edges

    def sort_waypoints_chrono(self, graph):
        """Sort waypoints by time created."""
        waypoint_to_timestamp = []
        for waypoint in graph.waypoints:
            # Determine the timestamp that this waypoint was created at.
            timestamp = -1.0
            try:
                timestamp = (
                    waypoint.annotations.creation_time.seconds
                    + waypoint.annotations.creation_time.nanos / 1e9
                )
            except:
                # Must be operating on an older graph nav map, since the creation_time is not
                # available within the waypoint annotations message.
                pass
            waypoint_to_timestamp.append(
                (waypoint.id, timestamp, waypoint.annotations.name)
            )

        # Sort the set of waypoints by their creation timestamp. If the creation timestamp is unavailable,
        # fallback to sorting by annotation name.
        waypoint_to_timestamp = sorted(
            waypoint_to_timestamp, key=lambda x: (x[1], x[2])
        )

        return waypoint_to_timestamp
