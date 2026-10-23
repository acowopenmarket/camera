"""Read actual latched TF with explicit reliable/transient-local QoS."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init()
node = Node('camera_tf_capture')
received = []
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
sub = node.create_subscription(TFMessage, '/tf_static',
    lambda msg: received.append(message_to_ordereddict(msg)), qos)
end = time.monotonic()+12
while time.monotonic() < end and not received:
    rclpy.spin_once(node, timeout_sec=.1)
path = Path.home()/'orbbec_ws/results/tf_static.json'
path.write_text(json.dumps(received, indent=2))
print(json.dumps(received, indent=2))
node.destroy_node()
rclpy.shutdown()
