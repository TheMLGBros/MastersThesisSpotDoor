

import py_trees


class SpotServiceBehaviour(py_trees.behaviour.Behaviour):
    """Base class for Spot service behaviors"""
    
    def __init__(self, name, client_type, robot):
        super().__init__(name)
        self.robot = robot
        self.response_future = None
        self.client_type = client_type
        self.client = None
        
    def setup(self):
        self.client = self.robot.ensure_client(self.client_type.default_service_name)
        if self.client is None:
            self.logger.error(f"Service not available: {self.name}")
            return False
        print("service available")
        return True
        
    def update(self):
        if self.response_future is None:
            print("Sync behaviour, should be done already")
            return py_trees.common.Status.SUCCESS
        if self.response_future.done():
            try:
                result = self.response_future.result()
                self.logger.info(f"{self.name} completed successfully")
                return py_trees.common.Status.SUCCESS
            except Exception as e:
                self.logger.error(f"{self.name} failed: {e}")
                return py_trees.common.Status.FAILURE
    
        return py_trees.common.Status.RUNNING
            
    def terminate(self, new_status):
        # if future is none the call was sync and is already completed
        if self.response_future is not None:
            self.response_future.cancel()
            self.response_future = None