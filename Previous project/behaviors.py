import sys
import time
import cv2
import numpy as np
from factory import BehaviorFactory
import py_trees
sys.path.append('../spot-sdk/python/examples')
import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.client.channel
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from manipulation import GraspObject, DropObject, CloseMouth, MoveToCarryState, PlacePosition, LookDirection, ArmJointAngles
from movement import WalkService, RotateService
from navigation import NavigateToFiducical


def robot_startup_sequence(robot: bosdyn.client.robot.Robot):
    try:
        bosdyn.client.util.authenticate(robot)
        robot.time_sync.wait_for_sync()
    except Exception as err:
        print('Failed to communicate with robot: %s', err)
        return False
    robot.power_on()
    assert robot.is_powered_on(), "Robot failed to power on"
    robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    robot_command_client.robot_command(RobotCommandBuilder.synchro_stand_command())
    print("Robot powered on and standing.")


def main():
    print("Starting...")
    sdk = bosdyn.client.create_standard_sdk('Behaviours')
    print("Created sdk")
    robot = sdk.create_robot("192.168.80.3")
    print("Created robot")
    bosdyn.client.util.authenticate(robot)
    print("Authenticated robot")
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    
    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        robot_startup_sequence(robot)

        close_looks = [
            LookDirection.LEFT_DOWN,
            LookDirection.LEFT_DIAGONAL_DOWN,
            LookDirection.FORWARD_DOWN,
            LookDirection.RIGHT_DIAGONAL_DOWN,
            LookDirection.RIGHT_DOWN,
            LookDirection.RIGHT_DIAGONAL_DOWN,
            LookDirection.FORWARD_DOWN,
            LookDirection.LEFT_DIAGONAL_DOWN
        ]

        distant_looks = [
            LookDirection.LEFT_UP,
            LookDirection.LEFT_DIAGONAL_UP,
            LookDirection.FORWARD_UP,
            LookDirection.RIGHT_DIAGONAL_UP,
            LookDirection.RIGHT_UP,
            LookDirection.RIGHT_DIAGONAL_UP,
            LookDirection.FORWARD_UP,
            LookDirection.LEFT_DIAGONAL_UP
        ]

        look_order = [look for pair in zip(close_looks, distant_looks) for look in pair]
        # 16 unique looks, the last 6 become unique due to turning the body
        look_grasp_sequences = [BehaviorFactory.create_look_grasp_sequence(
            name=f"Look and grasp sequence {i+1}",
            robot=robot,
            joint_angles_look=look_direction
        ) for i, look_direction in enumerate(look_order)]

        rotate = RotateService("Rotate 180 degrees clockwise", robot=robot, direction=-1)

        # Inject 180 degree rotate after doing the first 10 look and grasps to cover all directions
        big_grasp = py_trees.composites.Selector(
            name="Big ass tree",
            memory=True,
            children=[
                *look_grasp_sequences[:10],
                py_trees.decorators.Inverter(
                    name="Rotating",
                    child=rotate
                ),
                *look_grasp_sequences[10:]
            ]
        )

        move_arm_to_carry_state = MoveToCarryState("Predefined carry state", robot=robot)

        place_object = BehaviorFactory.create_place_object_sequence(
            robot=robot
        ) 
        navigate_to_fiducial_1 = NavigateToFiducical("Navigate to fiducial 1", robot=robot, fiducial_name="fiducial_1")
        walk_back = WalkService("Walk back 1 meter", robot=robot)

        place_and_walk_back = py_trees.decorators.FailureIsSuccess(
            name="Always succeed",
            child= py_trees.composites.Sequence(
            name="Place object and walk backwards",
            memory=True,
            children=[
                    place_object,
                    walk_back
                ]
        ))

        root = py_trees.decorators.Retry(
            name="Retry once if fails",
            num_failures=2,
            child=
                py_trees.decorators.Repeat(
                child=
                    py_trees.composites.Sequence(
                    name="I am Root",
                    memory=True,
                    children=[
                        big_grasp,
                        move_arm_to_carry_state,
                        navigate_to_fiducial_1,
                        place_and_walk_back,
                        ]
                    ),
                    num_success=-1,
                    name="Repeat forever"
                )
        )
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(timeout=15)
        tree.tick()

        try:
            rate_hz = 10
            print(root.status)
            while root.status == py_trees.common.Status.RUNNING:
                tree.tick()
                time.sleep(1.0/rate_hz)

            if root.status == py_trees.common.Status.SUCCESS:
                print("Mission completed successfully!")
            else:
                print("Mission failed!")
                
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            tree.shutdown()

if __name__ == '__main__':
    main()
