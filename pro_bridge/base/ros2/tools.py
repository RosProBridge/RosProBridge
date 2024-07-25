import rclpy
import rclpy.qos
from typing import Union
from pydoc import locate


def get_qos(qos=10) -> Union[rclpy.qos.QoSProfile, int]:
    if isinstance(qos, str):
        return locate("rclpy.qos." + qos)  # type: ignore
    elif isinstance(qos, dict):
        return rclpy.qos.QoSProfile(
            reliability=locate("rclpy.qos.ReliabilityPolicy." + qos.get("reliability", "RELIABLE")),
            history=locate("rclpy.qos.HistoryPolicy." + qos.get("history", "KEEP_LAST")),
            depth=qos.get("depth", 10),
            durability=locate("rclpy.qos.DurabilityPolicy." + qos.get("durability", "VOLATILE")),
            liveliness=locate("rclpy.qos.LivelinessPolicy." + qos.get("liveliness", "AUTOMATIC")),
        )
    elif isinstance(qos, int):
        return qos

    return rclpy.qos.qos_profile_system_default
