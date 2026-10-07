"""Stream raw PhaseSpace LEDs (and rigid bodies) to RViz.

Publishes:
  /phasespace_leds  visualization_msgs/MarkerArray, one sphere + ID label per visible LED
  /tf               map -> phasespace_<id> for every rigid tracker the server reports

PhaseSpace is millimetres with Y up; RViz expects metres with Z up, so
positions are converted (x, y, z)_ps -> (x, -z, y) / 1000.
"""

import argparse
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Ros_2"))
from phasespace_node_ros2 import Context, Type  # noqa: E402


def to_ros(x, y, z):
    return x / 1000.0, -z / 1000.0, y / 1000.0


class LedPublisher(Node):
    def __init__(self):
        super().__init__("phasespace_rviz")
        self.led_pub = self.create_publisher(MarkerArray, "/phasespace_leds", 10)
        self.tf = TransformBroadcaster(self)

    def publish_frame(self, markers, rigids):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for m in markers:
            if m.cond <= 0:
                continue
            x, y, z = to_ros(m.x, m.y, m.z)
            for kind in (Marker.SPHERE, Marker.TEXT_VIEW_FACING):
                mk = Marker()
                mk.header.frame_id = "map"
                mk.header.stamp = stamp
                mk.ns = "leds" if kind == Marker.SPHERE else "ids"
                mk.id = m.id
                mk.type = kind
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = x, y, z
                mk.pose.orientation.w = 1.0
                if kind == Marker.SPHERE:
                    mk.scale.x = mk.scale.y = mk.scale.z = 0.03
                    mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 1.0, 0.2, 1.0
                else:
                    mk.pose.position.z += 0.04
                    mk.scale.z = 0.04
                    mk.text = str(m.id)
                    mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
                array.markers.append(mk)
        self.led_pub.publish(array)

        for r in rigids:
            if r.cond <= 0:
                continue
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "map"
            t.child_frame_id = f"phasespace_{r.id}"
            x, y, z = to_ros(*r.pose[:3])
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.translation.z = z
            # quaternion (w, a, b, c) in PhaseSpace axes -> same axis swap as positions
            w, qx, qy, qz = r.pose[3:7]
            t.transform.rotation.w = w
            t.transform.rotation.x, t.transform.rotation.y = qx, -qz
            t.transform.rotation.z = qy
            self.tf.sendTransform(t)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="cs-phasespace.cs.umn.edu")
    parser.add_argument("--freq", type=float, default=60.0)
    parser.add_argument("--stream", choices=("tcp", "udp"), default="udp")
    parser.add_argument("--timeout", type=int, default=5000000)
    args = parser.parse_args()

    rclpy.init()
    node = LedPublisher()
    owl = Context()
    owl.open(args.device, f"timeout={args.timeout}")
    owl.initialize(f"timeout={args.timeout} event.markers=1 event.rigids=1")
    owl.frequency(args.freq)
    owl.streaming(2 if args.stream == "udp" else 1)
    node.get_logger().info(f"Streaming LEDs from {args.device}")

    try:
        while rclpy.ok() and owl.isOpen() and owl.property("initialized"):
            event = owl.nextEvent(args.timeout)
            if not event:
                continue
            if event.type_id == Type.FRAME:
                markers = event.markers if "markers" in event else []
                rigids = event.rigids if "rigids" in event else []
                node.publish_frame(markers, rigids)
            elif event.type_id == Type.ERROR:
                node.get_logger().error(f"PhaseSpace error: {event.data}")
                break
    except KeyboardInterrupt:
        pass
    finally:
        owl.done()
        owl.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
