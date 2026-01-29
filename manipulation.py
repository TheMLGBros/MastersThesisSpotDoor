from enum import Enum
import math
import sys
from dataclasses import dataclass
import time
import cv2
import numpy as np
import py_trees

from service import SpotServiceBehaviour
sys.path.append('../spot-sdk/python/examples')
import bosdyn.client.util
import bosdyn.client.channel
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.api import geometry_pb2, arm_command_pb2, gripper_command_pb2, manipulation_api_pb2, image_pb2
from bosdyn.client.math_helpers import Quat
from bosdyn.client.frame_helpers import BODY_FRAME_NAME
from bosdyn.client.image import ImageClient
from bosdyn.api import world_object_pb2 as wo
from bosdyn.client.world_object import WorldObjectClient, make_add_world_object_req
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand, block_until_arm_arrives
from yolo import ImgDetector



@dataclass
class ArmJointAngles:
    """Arm joint angles in degrees"""
    sh0: float
    sh1: float
    el0: float
    el1: float
    wr0: float
    wr1: float
    
    def to_radians(self):
        """Convert all angles to radians"""
        return ArmJointAngles(
            sh0=math.radians(self.sh0),
            sh1=math.radians(self.sh1),
            el0=math.radians(self.el0),
            el1=math.radians(self.el1),
            wr0=math.radians(self.wr0),
            wr1=math.radians(self.wr1)
        )


class LookDirection(Enum):
    """Predefined look directions"""
    FORWARD_DOWN = ArmJointAngles(0.0, -90.0, 90.0, 0.0, 90.0, 0.0)
    LEFT_DOWN = ArmJointAngles(120.0, -90.0, 90.0, 0.0, 90.0, 0.0)
    RIGHT_DOWN = ArmJointAngles(-120.0, -90.0, 90.0, 0.0, 90.0, 0.0)
    LEFT_DIAGONAL_DOWN = ArmJointAngles(60.0, -90.0, 90.0, 0.0, 90.0, 0.0)
    RIGHT_DIAGONAL_DOWN = ArmJointAngles(-60.0, -90.0, 90.0, 0.0, 90.0, 0.0)
    FORWARD_UP = ArmJointAngles(0.0, -90.0, 90.0, 0.0, 50.0, 0.0)
    LEFT_UP = ArmJointAngles(120.0, -90.0, 90.0, 0.0, 50.0, 0.0)
    RIGHT_UP = ArmJointAngles(-120.0, -90.0, 90.0, 0.0, 50.0, 0.0)
    LEFT_DIAGONAL_UP = ArmJointAngles(60.0, -90.0, 90.0, 0.0, 50.0, 0.0)
    RIGHT_DIAGONAL_UP = ArmJointAngles(-60.0, -90.0, 90.0, 0.0, 50.0, 0.0)

class Look(SpotServiceBehaviour):
    def __init__(self, name: str, robot, joint_angles: ArmJointAngles) -> None:
        super().__init__(name, robot=robot, client_type=RobotCommandClient)
        self.joint_angles = joint_angles

    def initialise(self):
        joint_angles_radians = self.joint_angles.value.to_radians()
        joint_move = RobotCommandBuilder.arm_joint_command(
            sh0 = joint_angles_radians.sh0,
            sh1 = joint_angles_radians.sh1,
            el0 = joint_angles_radians.el0,
            el1 = joint_angles_radians.el1,
            wr0 = joint_angles_radians.wr0,
            wr1 = joint_angles_radians.wr1
        )
        # Synchro command
        cmd = RobotCommandBuilder.build_synchro_command(joint_move)

        # Send the request to the robot
        self.arm_move_cmd_id = self.client.robot_command(cmd)

    def update(self):
        move_response = self.client.robot_command_feedback(self.arm_move_cmd_id)
        f = move_response.feedback.synchronized_feedback.arm_command_feedback

        status = f.arm_joint_move_feedback.status

        if status == arm_command_pb2.ArmJointMoveCommand.Feedback.STATUS_COMPLETE:
            return py_trees.common.Status.SUCCESS
        elif status == arm_command_pb2.ArmJointMoveCommand.Feedback.STATUS_IN_PROGRESS:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.FAILURE

class MoveToCarryState(SpotServiceBehaviour):
    def __init__(self, name: str, robot):
        super().__init__(name, client_type=RobotCommandClient, robot=robot)

    def initialise(self):
        carry = RobotCommandBuilder.arm_carry_command()
        self.carry_command_id = self.client.robot_command(carry)
    
    def update(self):
        move_response = self.client.robot_command_feedback(self.carry_command_id)
        m_status = move_response.feedback.synchronized_feedback.arm_command_feedback.named_arm_position_feedback.status
        if (m_status == arm_command_pb2.NamedArmPositionsCommand.Feedback.STATUS_COMPLETE):
            return py_trees.common.Status.SUCCESS 
        elif (m_status == arm_command_pb2.NamedArmPositionsCommand.Feedback.STATUS_IN_PROGRESS):
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.FAILURE


class PlacePosition(SpotServiceBehaviour):
    def __init__(self, name : str, robot, x, y, z) -> None:
        super().__init__(name, robot=robot, client_type=RobotCommandClient)
        self.x = x
        self.y = y
        self.z = z

    def initialise(self):
        x = self.x
        y = self.y
        z = self.z
        hand_ewrt_flat_body = geometry_pb2.Vec3(x=x, y=y, z=z)

        pitch = np.pi/4

        rot = Quat.from_pitch(pitch)

        q_rot = geometry_pb2.Quaternion(w=rot.w, x=rot.x, y=rot.y, z=rot.z)

        pose = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body, rotation=q_rot)
        
        print("Running initialize")
        command = RobotCommandBuilder.arm_pose_command_from_pose(pose, BODY_FRAME_NAME, seconds=1)
        cmd = RobotCommandBuilder.build_synchro_command(command)

        # Send the request to the robot.
        self.arm_move_cmd_id = self.client.robot_command(cmd)

    def update(self):
        move_response = self.client.robot_command_feedback(self.arm_move_cmd_id)
        m_status = move_response.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.status
        if (m_status == arm_command_pb2.ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_COMPLETE):
            return py_trees.common.Status.SUCCESS 
        elif (m_status == arm_command_pb2.ArmCartesianCommand.Feedback.STATUS_IN_PROGRESS):
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.FAILURE
    
class GripperCommand(SpotServiceBehaviour):
    def __init__(self, name : str, robot, open_gripper: bool) -> None:
        super().__init__(name, robot=robot, client_type=RobotCommandClient)
        self.open_gripper = open_gripper

    def initialise(self):
        command = RobotCommandBuilder.claw_gripper_open_command() if self.open_gripper else RobotCommandBuilder.claw_gripper_close_command()
        self.cmd_id = self.client.robot_command(command)

    def update(self):
        open_response = self.client.robot_command_feedback(self.cmd_id)
        o_status = open_response.feedback.synchronized_feedback.gripper_command_feedback.claw_gripper_feedback.status
        if (o_status == gripper_command_pb2.ClawGripperCommand.Feedback.STATUS_AT_GOAL):
            return py_trees.common.Status.SUCCESS 
        elif (o_status == gripper_command_pb2.ClawGripperCommand.Feedback.STATUS_IN_PROGRESS):
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.FAILURE
    
class DropObject(GripperCommand):
    def __init__(self, name : str, robot) -> None:
        super().__init__(name, robot=robot, open_gripper=True)

class CloseMouth(GripperCommand):
    def __init__(self, name : str, robot) -> None:
        super().__init__(name, robot=robot, open_gripper=False)


        
def convert_image(img_raw):
    if len(img_raw) != 1:
        print(f'Got invalid number of images: {len(img_raw)}')
        print(img_raw)
        assert False

    image = img_raw[0]
    if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
        dtype = np.uint16
    else:
        dtype = np.uint8
    img = np.fromstring(image.shot.image.data, dtype=dtype)
    if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
        img = img.reshape(image.shot.image.rows, image.shot.image.cols)
    else:
        img = cv2.imdecode(img, -1)

    return img, image

class GraspObject(SpotServiceBehaviour):
    def __init__(self, name: str, robot) -> None:
        super().__init__(name, robot=robot, client_type=ManipulationApiClient)
        self.img_client = self.robot.ensure_client(ImageClient.default_service_name)
        self._robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.detector = ImgDetector()

    def initialise(self) -> None:
        self.robot.time_sync.wait_for_sync()
        self._helper()

    def _helper(self):
        image_responses = self.img_client.get_image_from_sources(["hand_color_image"])
        img, image = convert_image(image_responses)
        start = time.time()
        grab_point = self.detector.get_grab_point(img, confidence=0.8)
        print(f"Object detection ran in: {time.time() - start} seconds" )
        # Set default failing value
        self.cmd_id = -1
        if grab_point:
            x_grab, y_grab = grab_point[0]

            pick_vec = geometry_pb2.Vec2(x=x_grab, y=y_grab)

            # Build the proto
            grasp = manipulation_api_pb2.PickObjectInImage(
                pixel_xy=pick_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
                frame_name_image_sensor=image.shot.frame_name_image_sensor,
                camera_model=image.source.pinhole, walk_gaze_mode=manipulation_api_pb2.PICK_AUTO_GAZE)

            # Ask the robot to pick up the object
            grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)

            # Send the request
            self.cmd_id = self.client.manipulation_api_command(
                manipulation_api_request=grasp_request).manipulation_cmd_id

    def _check_grasp_succeeded(self) -> bool:
        """If the gripper is not fully closed, the grasp likely succeeded"""
        return self._robot_state_client.get_robot_state().manipulator_state.gripper_open_percentage > 5
       
    def update(self):
        if self.cmd_id == -1:
            return py_trees.common.Status.FAILURE
        feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
            manipulation_cmd_id=self.cmd_id)

        # Send the request
        response = self.client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=feedback_request)

        RUNNING_STATES = [
            manipulation_api_pb2.MANIP_STATE_SEARCHING_FOR_GRASP,
            manipulation_api_pb2.MANIP_STATE_MOVING_TO_GRASP,
            manipulation_api_pb2.MANIP_STATE_ATTEMPTING_RAYCASTING,
            manipulation_api_pb2.MANIP_STATE_GRASPING_OBJECT,
            manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_SUCCEEDED,
            manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_WAITING_DATA_AT_EDGE
        ]

        if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
            # If the gripper is fully closed, an object (a ball) was not picked up
            if not self._check_grasp_succeeded():
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.SUCCESS 
        elif response.current_state in RUNNING_STATES:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.FAILURE
