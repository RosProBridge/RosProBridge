import os

ROS_VERSION = os.environ['ROS_VERSION']

if ROS_VERSION == '1':
    from base.ros1.node import ProBridgeRos1 as ProBridge
elif ROS_VERSION == '2':
    from base.ros2.node import ProBridgeRos2 as ProBridge