import time
import logging
from pymavlink import mavutil
from session import MavSession

log = logging.getLogger("VehicleBase")

class VehicleBase:
    """Mode & arming primitives over a MavSession."""

    def __init__(self, session: MavSession):
        self.sess = session
        self.conn = session.conn

    def recv_msg(self, msg_type=None, timeout=1.0):
        return self.sess.recv(msg_type, timeout=timeout)

    def wait_mode(self, name: str, timeout: float = 3.0) -> bool:
        mapping = self.conn.mode_mapping()
        want = mapping[name]
        end = time.time() + timeout
        while time.time() < end:
            hb = self.recv_msg('HEARTBEAT', timeout=0.3)
            if hb and getattr(hb, 'custom_mode', None) == want:
                return True
        raise RuntimeError(f"Mode change to {name} timed out")

    def set_mode(self, name: str) -> None:
        mapping = self.conn.mode_mapping()
        if name not in mapping:
            raise RuntimeError(f"Mode {name} not available. Got: {list(mapping.keys())}")
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[name]
        )
        self.wait_mode(name, timeout=3.0)

    def wait_command_ack(self, cmd: int, timeout: float = 3.0) -> None:
        end = time.time() + timeout
        while time.time() < end:
            ack = self.recv_msg('COMMAND_ACK', timeout=0.3)
            if ack and ack.command == cmd:
                if ack.result not in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                      mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                    raise RuntimeError(f"Command {cmd} failed: {ack.result}")
                return
        raise RuntimeError(f"No COMMAND_ACK for {cmd}")

    def arm(self, wait: bool = True, timeout: float = 5.0) -> None:
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0,0,0,0,0,0
        )
        self.wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 3.0)
        if not wait:
            return
        end = time.time() + timeout
        while time.time() < end:
            hb = self.recv_msg('HEARTBEAT', timeout=0.3)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                log.info("Vehicle armed")
                return
        raise RuntimeError("Arm timeout")

    def disarm(self, wait: bool = True, timeout: float = 5.0) -> None:
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0,0,0,0,0,0
        )
        self.wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 3.0)
        if not wait:
            return
        end = time.time() + timeout
        while time.time() < end:
            hb = self.recv_msg('HEARTBEAT', timeout=0.3)
            if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                log.info("Vehicle disarmed")
                return
        raise RuntimeError("Disarm timeout")
