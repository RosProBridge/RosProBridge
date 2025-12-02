import rospy

from pydoc import locate
from base.publisher import BridgePublisher
from typing import TYPE_CHECKING
from std_msgs.msg import Header

if TYPE_CHECKING:
    from base.ros1.node import ProBridgeRos1

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
    'bool': 1,
    'double': 8,
}

class BridgePublisherRos1(BridgePublisher):
    bridge: "ProBridgeRos1"

    def __init__(self, bridge: "ProBridgeRos1") -> None:
        super().__init__(bridge)

    def create_pub(self, b_msg: dict) -> rospy.Publisher:
        publisher = rospy.Publisher(
            name=b_msg["n"],
            data_class=locate(b_msg["t"]),
            queue_size=1,
            latch=bool(b_msg.get("l", False)),
        )
        self.publishers.update({b_msg["n"]: publisher})
        rospy.loginfo('Subscribed on topic "' + b_msg["n"] + '".')
        return publisher

    def deserialize_ros_message(self, binary_msg: bytes, msg_type: str):
        ros_msg = locate(msg_type)()
        return ros_msg.deserialize(binary_msg)

    def override_msg(self, msg_type: str, msg_packet: bytes):
        def remove_padding(size, msg_packet, byte_idx, bytes_removed):
            if size == 1:
                return bytes_removed
            padding_bytes = (size - (byte_idx+bytes_removed % size)) % size
            if padding_bytes != 0:
                del msg_packet[byte_idx:byte_idx+padding_bytes]
                bytes_removed += padding_bytes
            return bytes_removed

        def override(msg_type: str, msg_packet, byte_idx, bytes_removed):
            message = locate(msg_type)

            if (message._type == 'std_msgs/Header'):
                msg_packet[byte_idx:byte_idx] = bytearray([0x00, 0x00, 0x00, 0x00]) # add seq field
                bytes_removed -= 4
            for field in message._slot_types:
                if field in FIXED_TYPES:
                    field_size = FIXED_TYPES[field]
                    bytes_removed = remove_padding(field_size, msg_packet, byte_idx, bytes_removed)
                    byte_idx += FIXED_TYPES[field]
                elif 'time' in field:
                    bytes_removed = remove_padding(4, msg_packet, byte_idx, bytes_removed)
                    byte_idx+=4
                    bytes_removed = remove_padding(4, msg_packet, byte_idx, bytes_removed)
                    byte_idx+=4
                elif field == 'string':
                    bytes_removed = remove_padding(4, msg_packet, byte_idx, bytes_removed) # remove padding for length string
                    length_of_string = int.from_bytes(msg_packet[byte_idx:byte_idx + 4], byteorder='little') -1
                    msg_packet[byte_idx:byte_idx + 4] = int.to_bytes(length_of_string, length=4, byteorder='little')
                    byte_idx += 4 +length_of_string
                    del msg_packet[byte_idx] # remove empty byte from ros2 string
                    bytes_removed += 1
                elif '[' in field:
                    array_field_type = field.split('[')[0]
                    number_of_sequences = field.split('[')[1][:-1]
                    if number_of_sequences == '': #complex type received
                        bytes_removed = remove_padding(4, msg_packet, byte_idx, bytes_removed)
                        array_field_type = array_field_type.replace('/', '.msg.')
                        number_of_sequences = int.from_bytes(msg_packet[byte_idx:byte_idx + 4], byteorder='little')
                    else:
                        number_of_sequences = int(number_of_sequences)
                        if array_field_type in FIXED_TYPES:
                            value_size = FIXED_TYPES[array_field_type]
                            byte_idx += value_size * number_of_sequences
                            return byte_idx, bytes_removed
                        else:
                            raise NotImplementedError

                    byte_idx+=4
                    if array_field_type in FIXED_TYPES:
                        value_size = FIXED_TYPES[array_field_type]
                        for i in range(number_of_sequences):
                            bytes_removed = remove_padding(value_size, msg_packet, byte_idx, bytes_removed)
                    else:
                        for i in range(number_of_sequences):
                            byte_idx, bytes_removed = override(array_field_type, msg_packet, byte_idx, bytes_removed) 

                elif '/' in field:
                    q = field.replace('/', '.msg.')
                    byte_idx, bytes_removed = override(q, msg_packet,byte_idx, bytes_removed) #complex type
                else:
                    raise Exception
            return byte_idx, bytes_removed

        bytes_removed = 0
        msg_packet = bytearray(msg_packet)
        del msg_packet[0:4] #remove RMW header
        byte_idx = 0
        byte_idx = override(msg_type, msg_packet, byte_idx, bytes_removed)
        return bytes(msg_packet)
