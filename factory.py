import bosdyn
from navigation import NavigateToFiducical
from manipulation import GraspObject, DropObject, CloseMouth, MoveToCarryState, PlacePosition, Look, ArmJointAngles
import py_trees
import bosdyn

class BehaviorFactory:
    @staticmethod
    def create_look_behavior(robot: bosdyn.client.robot.Robot, joint_angles: ArmJointAngles, name: str = "look behaviour") -> Look:
        return Look(name=name, robot=robot, joint_angles=joint_angles)
    

    @staticmethod
    def create_look_grasp_sequence(name: str, robot: bosdyn.client.robot.Robot, joint_angles_look: ArmJointAngles) -> py_trees.composites.Sequence:
        return py_trees.composites.Sequence(
            name=name,
            memory=True,
            children= [
                BehaviorFactory.create_look_behavior(robot, joint_angles_look),
                GraspObject(name="Grasping behavior", robot=robot),
            ]
        )
    
    @staticmethod
    def create_place_object_sequence(robot: bosdyn.client.robot.Robot) -> py_trees.composites.Sequence:
        return py_trees.composites.Sequence(
            name="Place object",
            memory=True,
            children=
            [
                PlacePosition("Move to dropping position", robot=robot, x = 0.90, y = 0, z = 0.50),
                DropObject("Open mouth", robot=robot),
                CloseMouth("Close the mouth", robot=robot)
            ]
        )
    
