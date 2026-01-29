


class UploadGraph(SpotServiceBehaviour):
    def __init__(self, name : str, robot, filepath: str) -> None:
        super().__init__(name, client_type=GraphNavClient, robot=robot)
        self._filepath = filepath

    def initialise(self):
        self._upload_graph_and_snapshots()

    def _upload_graph_and_snapshots(self):
        """Upload the graph and snapshots to the robot."""
        current_waypoint_snapshots = {}
        current_edge_snapshots = {}
        print('Loading the graph from disk into local storage...')
        with open(self._filepath + '/graph', 'rb') as graph_file:
            # Load the graph from disk.
            data = graph_file.read()
            current_graph = map_pb2.Graph()
            current_graph.ParseFromString(data)
            print(
                f'Loaded graph has {len(current_graph.waypoints)} waypoints and {len(current_graph.edges)} edges'
            )
        for waypoint in current_graph.waypoints:
            # Load the waypoint snapshots from disk.
            with open(f'{self._filepath}/waypoint_snapshots/{waypoint.snapshot_id}',
                      'rb') as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot
        for edge in current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            # Load the edge snapshots from disk.
            with open(f'{self._filepath}/edge_snapshots/{edge.snapshot_id}',
                      'rb') as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        # Upload the graph to the robot.
        print('Uploading the graph and snapshots to the robot...')
        true_if_empty = not len(current_graph.anchoring.anchors)
        response = self.client.upload_graph(graph=current_graph,
                                                       generate_new_anchoring=true_if_empty)
        # Upload the snapshots to the robot.
        for snapshot_id in response.unknown_waypoint_snapshot_ids:
            waypoint_snapshot = current_waypoint_snapshots[snapshot_id]
            self.client.upload_waypoint_snapshot(waypoint_snapshot)
            print(f'Uploaded {waypoint_snapshot.id}')
        for snapshot_id in response.unknown_edge_snapshot_ids:
            edge_snapshot = current_edge_snapshots[snapshot_id]
            self.client.upload_edge_snapshot(edge_snapshot) 
            print(f'Uploaded {edge_snapshot.id}')
        
        print("Upload complete!")



class ListGraphService(SpotServiceBehaviour):
    def __init__(self, name: str, robot, filepath):
        super().__init__(name, robot=robot, client_type=GraphNavClient)
        self._filepath = filepath

    def initialise(self) -> None:
        self._list_graph_waypoint_and_edge_ids()

    def _list_graph_waypoint_and_edge_ids(self, *args):
        """List the waypoint ids and edge ids of the graph currently on the robot."""

        # Download current graph
        graph = self.client.download_graph()
        print(graph)
        if graph is None:
            print('Empty graph.')
            return

        print("Printing waypoints:")
        for waypoint in graph.waypoints:
            print(f"{waypoint.annotations.name}: {waypoint.id}")
        
        # Update and print waypoints and edges
        #self._current_annotation_name_to_wp_id, self._current_edges = graph_nav_util.update_waypoints_and_edges(
        #    graph, localization_id)
        

class Localize(SpotServiceBehaviour):
    def __init__(self, name : str, robot) -> None:
        super().__init__(name, robot=robot, client_type=GraphNavClient)
        self._robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)

    def initialise(self) -> None:
        self._localize_by_fiducial()
        print("localized!")

    def _localize_by_fiducial(self):
        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.    
        localization = nav_pb2.Localization()
        print(self.client.set_localization(initial_guess_localization=localization,
                                                ko_tform_body=current_odom_tform_body))

class NavigateToWaypoint(SpotServiceBehaviour):
    def __init__(self, name : str, robot, waypoint_id) -> None:
        super().__init__(name, robot=robot, client_type=GraphNavClient)
        self.waypoint_id = waypoint_id

    def initialise(self) -> None:
        self.cmd_id = self.client.navigate_to(self.waypoint_id, 30)

    def update(self):
        if self.cmd_id == -1:
            # No command, so we have no status to check.
            print("No command found")
            return py_trees.common.Status.FAILURE
        status = self.client.navigation_feedback(self.cmd_id)
        if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            # Successfully completed the navigation commands!
            return py_trees.common.Status.SUCCESS
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            print('Robot got lost when navigating the route, the robot will now sit down.')
            return py_trees.common.Status.FAILURE
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            print('Robot got stuck when navigating the route, the robot will now sit down.')
            return py_trees.common.Status.FAILURE
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            print('Robot is impaired.')
            return py_trees.common.Status.FAILURE
        else:
            # Navigation command is not complete yet.
            return py_trees.common.Status.RUNNING


class RemoveMap(SpotServiceBehaviour):
    def __init__(self, name: str, robot):
        super().__init__(name, client_type=GraphNavClient, robot=robot)

    def initialise(self) -> None:
        self.client.clear_graph()
        

        
upload = UploadGraph("upload :)", robot=robot, filepath="./downloaded_graph/")
lg = ListGraphService("List :)", robot=robot, filepath="./downloaded_graph/")
ng = NavigateToWaypoint("Navigate :)", robot=robot, waypoint_id="craggy-ewe-nU3UrDLpr9.2G5QnGWT7.w==")
ng2 = NavigateToWaypoint("Navigate :)", robot=robot, waypoint_id="sewed-corgi-LVzjYYf3CN6ZNiBQr16TCQ==")
loc = Localize("Localize :)", robot=robot)