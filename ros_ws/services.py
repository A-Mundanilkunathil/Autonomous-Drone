from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode 

class MavrosServiceClient:
    def __init__(self, node: Node):
        self.node = node
        self.arm_cli = node.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = node.create_client(SetMode, "/mavros/set_mode")
    
    def arm(self, value: bool = True):
        req = CommandBool.Request()
        req.value = value
        self.arm_cli.wait_for_service(timeout_sec=3.0)
        self.arm_cli.call_async(req)

    def set_mode(self, mode: str):
        req = SetMode.Request("")
        req.custom_mode = mode
        self.mode_cli.wait_for_service(timeout_sec=3.0)
        self.mode_cli.call_async(req)