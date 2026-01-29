import sys
import time

import numpy as np
import py_trees

from service import SpotServiceBehaviour
sys.path.append('../spot-sdk/python/examples')

#import graph_nav_command_line.graph_nav_util
import grpc

from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.client.channel
from bosdyn.api import world_object_pb2
from bosdyn.client.math_helpers import Quat
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder

class NavigateToFiducical(SpotServiceBehaviour):
    def __init__(self, name: str, robot, fiducial_name: str):
        super().__init__(name=name, client_type=WorldObjectClient, robot=robot)
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        BODY_LENGTH = 1.1
        self._tag_offset = BODY_LENGTH / 1.5  # meters
        self._current_tag_world_pose = np.array([])
        self._x_eps = .1
        self._y_eps = .1
        self._angle_eps = .1
        self.key = fiducial_name

    def get_fiducial_objects(self):
        """Get all fiducials that Spot detects with its perception system."""
        # Get all fiducial objects (an object of a specific type).
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = self.client.list_world_objects(
            object_type=request_fiducials).world_objects
        if len(fiducial_objects) > 0:
            # Return the first detected fiducial.
            for fiducial in fiducial_objects:
                if fiducial.apriltag_properties.frame_name_fiducial == self.key:
                    return fiducial
        # Return none if no fiducials are found.
        return None
    
    def get_desired_angle(self, xhat):
        """Compute heading based on the vector from robot to object."""
        zhat = [0.0, 0.0, 1.0]
        yhat = np.cross(zhat, xhat)
        mat = np.array([xhat, yhat, zhat]).transpose()
        return Quat.from_matrix(mat).to_yaw()
    
    def offset_tag_pose(self, object_rt_world, dist_margin=1.0):
        """Offset the go-to location of the fiducial and compute the desired heading."""
        robot_rt_world = get_vision_tform_body(self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
        robot_to_object_ewrt_world = np.array(
            [object_rt_world.x - robot_rt_world.x, object_rt_world.y - robot_rt_world.y, 0])
        robot_to_object_ewrt_world_norm = robot_to_object_ewrt_world / np.linalg.norm(
            robot_to_object_ewrt_world)
        heading = self.get_desired_angle(robot_to_object_ewrt_world_norm)
        goto_rt_world = np.array([
            object_rt_world.x - robot_to_object_ewrt_world_norm[0] * dist_margin,
            object_rt_world.y - robot_to_object_ewrt_world_norm[1] * dist_margin
        ])
        return goto_rt_world, heading
    
    def initialise(self) -> None:
        fiducial = self.get_fiducial_objects()
        if fiducial is not None:
            vision_tform_fiducial = get_a_tform_b(
                fiducial.transforms_snapshot, VISION_FRAME_NAME,
                fiducial.apriltag_properties.frame_name_fiducial).to_proto()
            if vision_tform_fiducial is not None:
                detected_fiducial = True
                fiducial_rt_world = vision_tform_fiducial.position
        
        # Compute the go-to point (offset by .5m from the fiducial position) and the heading at
        # this point.
        self._current_tag_world_pose, self._angle_desired = self.offset_tag_pose(
            fiducial_rt_world, self._tag_offset)

        #Command the robot to go to the tag in kinematic odometry frame
        #mobility_params = self.set_mobility_params()
        tag_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=self._current_tag_world_pose[0], goal_y=self._current_tag_world_pose[1],
            goal_heading=self._angle_desired, frame_name=VISION_FRAME_NAME,
            body_height=0.0, locomotion_hint=spot_command_pb2.HINT_AUTO)
        end_time = 5.0
        self._robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                end_time_secs=time.time() + end_time)
    
    def update(self):
        robot_state = get_vision_tform_body(self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
        robot_angle = robot_state.rot.to_yaw()
        if self._current_tag_world_pose.size != 0:
            x_dist = abs(self._current_tag_world_pose[0] - robot_state.x)
            y_dist = abs(self._current_tag_world_pose[1] - robot_state.y)
            angle = abs(self._angle_desired - robot_angle)
            if ((x_dist < self._x_eps) and (y_dist < self._y_eps) and (angle < self._angle_eps)):
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING