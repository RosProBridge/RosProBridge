from rclpy.publisher import Publisher
from rclpy.serialization import deserialize_message
from pydoc import locate
from base.publisher import BridgePublisher
from base.ros2.tools import get_qos
from typing import TYPE_CHECKING
from rosidl_parser.definition import NamespacedType, BasicType, UnboundedString, UnboundedSequence, Array
from std_msgs.msg import Header

if TYPE_CHECKING:
    from base.ros2.node import ProBridgeRos2

FIXED_TYPES = {
    'int8': 1,
    'int16': 2,
    'int32': 4,
    'int64': 8,
    'uint8': 1,
    'uint16': 2,
    'uint32': 4,
    'uint64': 8,
    'float': 4,
    'float64': 8,
    'boolean': 1,
    'double': 8
}

NOT_FIXED_SIZE = ['string']


class BridgePublisherRos2(BridgePublisher):
    bridge: "ProBridgeRos2"

    def __init__(self, bridge: "ProBridgeRos2") -> None:
        super().__init__(bridge)

    def create_pub(self, b_msg: dict) -> Publisher:
        qos_profile = get_qos(b_msg.get("q", 0))
        publisher = self.bridge.create_publisher(msg_type=locate(b_msg["t"]), topic=b_msg["n"], qos_profile=qos_profile)
        self.publishers.update({b_msg["n"]: publisher})
        self.bridge.loginfo('Subscribed on topic "' + b_msg["n"] + '".')
        return publisher

    def deserialize_ros_message(self, binary_msg, msg_type):
        ros_msg = locate(msg_type)
        return deserialize_message(binary_msg, ros_msg)

    def override_msg(self, msg_type: str, msg_packet: bytes):
        def apply_padding(size, msg_packet, byte_idx):
            padding_bytes = (size - (byte_idx % size)) % size
            if padding_bytes != 0:
                msg_packet[byte_idx:byte_idx] =bytearray([0x00] * padding_bytes)
                byte_idx += padding_bytes
            return byte_idx

        def override(msg_type: str, msg_packet, byte_idx):
            message = locate(msg_type)

            if (type(message) == type(Header)):
                del msg_packet[byte_idx:byte_idx + 4]

            for field in message.SLOT_TYPES:
                if isinstance(field, NamespacedType):
                    field_type = ".".join(field.namespaced_name())
                    byte_idx = override(field_type, msg_packet, byte_idx)
                elif isinstance(field, BasicType):
                    if field.typename not in FIXED_TYPES:
                        raise Exception
                    fixed_size = FIXED_TYPES[field.typename]
                    if fixed_size != 1:
                        byte_idx = apply_padding(fixed_size, msg_packet, byte_idx)
                    byte_idx += fixed_size
                elif isinstance(field, UnboundedString):
                    length_of_string = int.from_bytes(msg_packet[byte_idx:byte_idx + 4], byteorder='little')
                    msg_packet[byte_idx:byte_idx + 4] = int.to_bytes(length_of_string + 1, length=4, byteorder='little')
                    byte_idx += 4 +length_of_string
                    msg_packet.insert(byte_idx, 0x00) # insert empty byte. ROS2 string always ends with 0x00
                    byte_idx += 1
                elif isinstance(field, UnboundedSequence):
                    byte_idx = apply_padding(4, msg_packet, byte_idx)
                    number_of_sequences = int.from_bytes(msg_packet[byte_idx:byte_idx + 4], byteorder='little')
                    byte_idx += 4

                    if hasattr(field.value_type, 'namespaced_name'):
                        ctype = ".".join(field.value_type.namespaced_name()) #complex type
                        for i in range(number_of_sequences):
                            byte_idx = override(ctype, msg_packet, byte_idx)
                    else:
                        if field.value_type.typename in FIXED_TYPES: #simple type
                            value_size = FIXED_TYPES[field.value_type.typename]
                            byte_idx = apply_padding(value_size, msg_packet, byte_idx)

                            # for i in range(number_of_sequences):
                            #     byte_idx = apply_padding(value_size, msg_packet, byte_idx)
                            byte_idx += value_size * number_of_sequences

            return byte_idx

        msg_packet = bytearray(msg_packet)

        byte_idx = 0
        byte_idx = override(msg_type, msg_packet, byte_idx)

        msg_packet[0:0] = bytearray([0x00, 0x01, 0x00, 0x00]) # add RMW Header

        return bytes(msg_packet)