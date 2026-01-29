import sys
import time
import numpy as np
import py_trees
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b, GROUND_PLANE_FRAME_NAME
sys.path.append('../spot-sdk/python/examples')
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.client.channel
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from service import SpotServiceBehaviour

class VelocityCommandService(SpotServiceBehaviour):
    def __init__(self, name, robot, v_x=0.0, v_y=0.0, v_rot=0.0, duration=0):
        super().__init__(name, client_type=RobotCommandClient, robot=robot)
        self.v_x = v_x
        self.v_y = v_y
        self.v_rot = v_rot
        self.duration = duration

    def initialise(self) -> None:
        self._velocity_cmd_helper()
    
    def _velocity_cmd_helper(self):
        self.client.robot_command(
            RobotCommandBuilder.synchro_velocity_command(v_x=self.v_x, v_y=self.v_y, v_rot=self.v_rot),
            end_time_secs=time.time() + self.duration)
        
        # Blocking sleep for command duration
        time.sleep(self.duration)

class WalkService(VelocityCommandService):
    """Walks 1 meter backwards"""
    def __init__(self, name: str, robot):
        super().__init__(name, robot=robot, v_x=-1, duration=1)

    

class RotateService(VelocityCommandService):
    """Rotates 180 degrees in given direction"""
    def __init__(self, name: str, robot, direction: int):
        super().__init__(name, robot=robot,
                          v_rot=np.pi/2*direction,
                          duration=2.2
                        )