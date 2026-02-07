import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.sensors.physics import IMUSensor
from isaacsim.storage.native import get_assets_root_path
from isaacsim.sensors.physics import IMUSensor
import omni.graph.core as og 

"""
Oceansim IMU

Class wraps around ImuSensor and adds methos for publishing to ROS topic
"""

class ImuSensor_ROS(IMUSensor):

    def __init__(self, 
        prim_path, 
        name = "Imu", 
        frequency = None,
        translation = None, og_node = None):
        self._og_node = None
    
        super().__init__(prim_path=prim_path, name=name, frequency=frequency,translation=translation)
    
    def initialize(self,og_node=None):
        self._og_node = og_node


    def read(self):
        # imu api https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.physics/docs/index.html#isaacsim.sensors.physics.IMUSensor
        #see graph node attributes: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2PublishImu.html
        imu_data = self.get_current_frame()
        og.Controller.attribute(self._og_node.get_attribute("inputs:angularVelocity")).set(imu_data["ang_vel"])
        og.Controller.attribute(self._og_node.get_attribute("inputs:linearAcceleration")).set(imu_data["lin_acc"])
        og.Controller.attribute(self._og_node.get_attribute("inputs:orientation")).set(imu_data["orientation"])


