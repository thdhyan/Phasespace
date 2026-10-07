"""Stream PhaseSpace data to RViz and to the G1 stack.

Publishes (all in frame `phasespace`, the PhaseSpace world: origin = alignment
origin on the floor tape, Z up, metres):
  /phasespace_leds     MarkerArray  one sphere + ID per visible LED, coloured per microdriver
  /phasespace_world    MarkerArray  floor plane over the camera footprint, camera view cones,
                                    SMPL-X body per LED group (latched)
  /tf_static           phasespace -> ps_cam_<id> for every calibrated camera
  /tf                  phasespace -> <device> at the LED-group centroid (head centre),
                       phasespace -> phasespace_<id> for rigid trackers
  /phasespace/humans   std_msgs/String JSON, same schema as G1_sim's /sim/humans
                       ({"frame", "humans": [{"name", "x", "y", "heading_deg"}]})

LED groups come from the active session profile: every microdriver in it is
one group named after the device (e.g. "dhyan-hat" -> LEDs 0-7).

PhaseSpace is millimetres with Y up; ROS is metres with Z up, so positions map
(x, y, z)_ps -> (x, -z, y) / 1000. Camera optical axis is the camera's local -Z.
"""

import argparse
import json
import math
import os
import sys
import time
from urllib.parse import unquote

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Ros_2"))
from phasespace_node_ros2 import Context, Type  # noqa: E402

FRAME = "phasespace"
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
GROUP_COLORS = [(0.1, 1.0, 0.2), (1.0, 0.5, 0.1), (0.2, 0.6, 1.0), (1.0, 0.2, 0.8), (1.0, 1.0, 0.2)]


def to_ros(x, y, z):
    return np.array([x, -z, y]) / 1000.0


def quat_to_ros(w, a, b, c):
    """PhaseSpace (w, x, y, z) quaternion -> ROS (x, y, z, w) under the same axis swap."""
    return a, -c, b, w


def quat_matrix(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def norm_name(name):
    return unquote(name).replace(" ", "").lower()


def led_groups(owl, profile):
    """{device name: [led ids]} for the named (or default) session profile, {} until known."""
    raw = owl.property("profiles.json") or ""
    if "=" not in raw:
        return {}
    wanted = norm_name(profile or owl.property("defaultprofile") or "")
    for p in json.loads(raw.split("=", 1)[1]):
        if norm_name(p["name"]) == wanted:
            return {
                d["name"]: [led["id"] for s in d["strings"] for led in s["leds"]]
                for d in p["devices"]
            }
    return {}


def load_smplx(path):
    """SMPL-X neutral template as (vertices, faces) in ROS axes, head centre at the origin."""
    d = np.load(path, allow_pickle=True)
    v, f = d["v_template"], d["f"]
    neck_y = (d["J_regressor"] @ v)[12, 1]  # joint 12 = neck
    head_center = v[v[:, 1] > neck_y].mean(0)
    v = v - head_center
    # SMPL-X: Y up, facing +Z, +X = body's left  ->  ROS: Z up, facing +X, +Y = left
    return v[:, [2, 0, 1]], f


class PhaseSpaceRviz(Node):
    def __init__(self, args):
        super().__init__("phasespace_rviz")
        self.args = args
        self.led_pub = self.create_publisher(MarkerArray, "/phasespace_leds", 10)
        self.world_pub = self.create_publisher(MarkerArray, "/phasespace_world", LATCHED)
        self.humans_pub = self.create_publisher(String, args.humans_topic, LATCHED)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.groups = {}
        self.world = {}  # ns -> [Marker], republished whole on every change
        self.last_humans = 0.0
        self.seen_groups = set()
        self.bodies = {}  # group -> SMPL-X marker, sent once the group's TF exists
        self.smpl = None
        if args.smplx and os.path.exists(args.smplx):
            self.smpl = load_smplx(args.smplx)
            self.get_logger().info(f"SMPL-X body from {args.smplx}")
        else:
            self.get_logger().warn(f"No SMPL-X model at {args.smplx}; bodies disabled")

    # ---------- static world: cameras, floor, bodies ----------
    def set_cameras(self, cameras):
        cams = [c for c in cameras if c.cond > 0]
        if not cams:
            return
        transforms, cones, labels = [], [], []
        half = math.tan(math.radians(self.args.cam_fov) / 2.0)
        cone = Marker(ns="camera_cones", id=0, type=Marker.LINE_LIST, action=Marker.ADD)
        cone.scale.x = 0.01
        cone.color.r, cone.color.g, cone.color.b, cone.color.a = 0.3, 0.8, 1.0, 0.35
        for c in cams:
            pos = to_ros(*c.pose[:3])
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = FRAME
            t.child_frame_id = f"ps_cam_{c.id}"
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pos
            r = t.transform.rotation
            r.x, r.y, r.z, r.w = quat_to_ros(*c.pose[3:7])
            transforms.append(t)

            rot = quat_matrix(*c.pose[3:7])
            corners = [
                to_ros(*(np.array(c.pose[:3]) + 1000.0 * self.args.cone_length * rot @ np.array([sx * half, sy * half, -1.0])))
                for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))
            ]
            for i, k in enumerate(corners):
                cone.points += [Point(x=pos[0], y=pos[1], z=pos[2]), Point(x=k[0], y=k[1], z=k[2])]
                n = corners[(i + 1) % 4]
                cone.points += [Point(x=k[0], y=k[1], z=k[2]), Point(x=n[0], y=n[1], z=n[2])]
            labels.append(self._text("camera_ids", c.id, pos + [0, 0, 0.12], str(c.id), 0.12))
        self.static_tf.sendTransform(transforms)

        # floor plane (PhaseSpace y = 0) over the camera footprint
        xy = np.array([to_ros(*c.pose[:3])[:2] for c in cams])
        lo, hi = xy.min(0), xy.max(0)
        floor = Marker(ns="floor", id=0, type=Marker.CUBE, action=Marker.ADD)
        floor.pose.position.x, floor.pose.position.y = (lo + hi) / 2.0
        floor.pose.position.z = -0.005
        floor.pose.orientation.w = 1.0
        floor.scale.x, floor.scale.y, floor.scale.z = hi[0] - lo[0], hi[1] - lo[1], 0.01
        floor.color.r = floor.color.g = floor.color.b = 0.55
        floor.color.a = 0.35
        origin = self._text("origin", 0, np.array([0.0, 0.0, 0.15]), "PhaseSpace origin", 0.15)

        self.world.update(camera_cones=[cone], camera_ids=labels, floor=[floor], origin=[origin])
        self._publish_world()
        self.get_logger().info(
            f"{len(cams)} cameras; floor x [{lo[0]:.2f}, {hi[0]:.2f}] y [{lo[1]:.2f}, {hi[1]:.2f}] m"
        )

    def set_groups(self, groups):
        self.groups = groups
        self.get_logger().info(f"LED groups: {groups}")
        if self.smpl is None:
            return
        v, f = self.smpl
        bodies = []
        for i, name in enumerate(groups):
            mk = Marker(ns="smplx", id=i, type=Marker.TRIANGLE_LIST, action=Marker.ADD)
            mk.header.frame_id = name
            mk.frame_locked = True
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 1.0
            r, g, b = GROUP_COLORS[i % len(GROUP_COLORS)]
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, 0.6
            mk.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in v[f.reshape(-1)]]
            bodies.append(mk)
        self.bodies = dict(zip(groups, bodies))

    def _publish_world(self):
        # zero stamp: RViz uses the latest TF, which these latched markers need
        array = MarkerArray()
        bodies = [mk for name, mk in self.bodies.items() if name in self.seen_groups]
        for markers in [*self.world.values(), bodies]:
            for mk in markers:
                if not mk.header.frame_id:
                    mk.header.frame_id = FRAME
                array.markers.append(mk)
        self.world_pub.publish(array)

    def _text(self, ns, id, pos, text, size):
        mk = Marker(ns=ns, id=id, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, text=text)
        mk.header.frame_id = FRAME
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = (float(p) for p in pos)
        mk.pose.orientation.w = 1.0
        mk.scale.z = size
        mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
        return mk

    # ---------- per frame: LEDs, group centroids, rigids ----------
    def publish_frame(self, markers, rigids):
        stamp = self.get_clock().now().to_msg()
        group_of = {led: (i, name) for i, (name, leds) in enumerate(self.groups.items()) for led in leds}
        array = MarkerArray()
        array.markers.append(Marker(action=Marker.DELETEALL))
        centroids = {}
        for m in markers:
            if m.cond <= 0:
                continue
            pos = to_ros(m.x, m.y, m.z)
            gi, gname = group_of.get(m.id, (None, None))
            if gname:
                centroids.setdefault(gname, []).append(pos)
            sphere = Marker(ns="leds", id=m.id, type=Marker.SPHERE, action=Marker.ADD)
            sphere.header.frame_id = FRAME
            sphere.header.stamp = stamp
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = pos
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.03
            r, g, b = GROUP_COLORS[gi % len(GROUP_COLORS)] if gi is not None else (0.7, 0.7, 0.7)
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, 1.0
            label = self._text("ids", m.id, pos + [0, 0, 0.04], str(m.id), 0.04)
            label.header.stamp = stamp
            array.markers += [sphere, label]
        self.led_pub.publish(array)

        transforms, humans = [], []
        for name, pts in centroids.items():
            c = np.mean(pts, axis=0)
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = FRAME
            t.child_frame_id = name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = c
            t.transform.rotation.w = 1.0  # heading unknown from loose LEDs; body faces +x
            transforms.append(t)
            humans.append({"name": name, "x": round(float(c[0]), 3), "y": round(float(c[1]), 3),
                           "z": round(float(c[2]), 3), "heading_deg": None, "n_leds": len(pts)})
        for r in rigids:
            if r.cond <= 0:
                continue
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = FRAME
            t.child_frame_id = f"phasespace_{r.id}"
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = to_ros(*r.pose[:3])
            q = t.transform.rotation
            q.x, q.y, q.z, q.w = quat_to_ros(*r.pose[3:7])
            transforms.append(t)
        if transforms:
            self.tf.sendTransform(transforms)
        if not set(centroids) <= self.seen_groups:
            self.seen_groups |= set(centroids)
            self._publish_world()

        now = time.monotonic()
        if now - self.last_humans >= 1.0 / self.args.humans_rate:
            self.last_humans = now
            self.humans_pub.publish(String(data=json.dumps({"frame": FRAME, "humans": humans})))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="cs-phasespace.cs.umn.edu")
    parser.add_argument("--freq", type=float, default=60.0)
    parser.add_argument("--stream", choices=("tcp", "udp"), default="udp")
    parser.add_argument("--timeout", type=int, default=5000000)
    parser.add_argument("--profile", default=None, help="session profile name (default: server default)")
    parser.add_argument("--smplx", default=os.environ.get(
        "SMPLX_MODEL", os.path.expanduser("~/Projects/HOVER/body_models/smplx/SMPLX_NEUTRAL.npz")))
    parser.add_argument("--cam-fov", type=float, default=60.0, help="camera view-cone full angle, deg")
    parser.add_argument("--cone-length", type=float, default=3.0, help="camera view-cone length, m")
    parser.add_argument("--humans-topic", default="/phasespace/humans")
    parser.add_argument("--humans-rate", type=float, default=10.0, help="Hz")
    args = parser.parse_args()

    rclpy.init()
    node = PhaseSpaceRviz(args)
    owl = Context()
    owl.open(args.device, f"timeout={args.timeout}")
    owl.initialize(f"timeout={args.timeout} event.cameras=1 event.markers=1 event.rigids=1")
    owl.frequency(args.freq)
    owl.streaming(2 if args.stream == "udp" else 1)
    node.get_logger().info(f"Streaming from {args.device}")

    have_cameras = False
    try:
        while rclpy.ok() and owl.isOpen() and owl.property("initialized"):
            event = owl.nextEvent(args.timeout)
            if not event:
                continue
            if not node.groups:
                groups = led_groups(owl, args.profile)
                if groups:
                    node.set_groups(groups)
            if event.type_id == Type.CAMERA and not have_cameras:
                node.set_cameras(event.data)
                have_cameras = True
            elif event.type_id == Type.FRAME:
                if "cameras" in event and not have_cameras:
                    node.set_cameras(event.cameras)
                    have_cameras = True
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
