import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image

rclpy.init(); node = Node('camera_header_capture'); found = {}
def record(name, msg):
    found[name] = dict(encoding=msg.encoding, width=msg.width, height=msg.height,
                       step=msg.step, is_bigendian=msg.is_bigendian,
                       data_bytes=len(msg.data), frame_id=msg.header.frame_id,
                       stamp_sec=msg.header.stamp.sec, stamp_nanosec=msg.header.stamp.nanosec)
subs = [node.create_subscription(Image, name, lambda m, n=name: record(n,m), QoSProfile(depth=10))
        for name in ('/camera/color/image_raw', '/camera/depth/image_raw')]
end = time.monotonic()+10
while len(found) < 2 and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.1)
(Path.home()/'orbbec_ws/results/image_headers.json').write_text(json.dumps(found, indent=2))
print(json.dumps(found, indent=2)); node.destroy_node(); rclpy.shutdown()
