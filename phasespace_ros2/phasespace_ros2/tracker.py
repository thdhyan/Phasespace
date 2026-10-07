"""Stream PhaseSpace LEDs, LED groups, cameras and rigid trackers to ROS 2.

Publishes (all in frame `phasespace`, the PhaseSpace world: origin = alignment
origin on the floor tape, Z up, metres):
  /phasespace_leds     MarkerArray  one sphere + ID per visible LED, coloured per LED group
  /phasespace_world    MarkerArray  floor plane over the camera footprint, camera view cones (latched)
  /phasespace/groups   std_msgs/String JSON (latched):
                       {"frame", "groups": {name: {"leds": [ids], "oriented": bool}}}
  /tf_static           phasespace -> ps_cam_<id> for every calibrated camera
  /tf                  phasespace -> <group>            group pose (see Orientation)
                       phasespace -> <group>/led_<id>   every visible LED of a group (position only)
                       phasespace -> led_<id>           visible LEDs in no group
                       phasespace -> phasespace_<id>    rigid trackers

LED groups come from the active session profile: every microdriver in it is
one group named after the device (e.g. "dhyan-hat" -> LEDs 0-7).

Orientation: `--capture NAME` records group NAME's LED layout (hold it still,
facing +x / upright) to <ref-dir>/NAME.json. While running, each group with a
reference is fitted to it (Kabsch on matched LED IDs, >= 3 visible), giving the
group's full pose; without one the group TF is the LED centroid with no rotation.

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
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from phasespace_ros2.owl import Context, Type

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


def matrix_to_quat(m):
    """Rotation matrix -> (x, y, z, w)."""
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2.0, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2.0, m[1, 0] - m[0, 1])
    return x, y, z, w


def fit_pose(ref, seen):
    """Pose (R, t) mapping reference LED positions onto the seen ones (Kabsch), or None.

    ref: {led id: local xyz}, origin = reference centroid; seen: {led id: world xyz}.
    """
    ids = [i for i in seen if i in ref]
    if len(ids) < 3:
        return None
    a = np.array([ref[i] for i in ids])
    b = np.array([seen[i] for i in ids])
    ca, cb = a.mean(0), b.mean(0)
    u, sv, vt = np.linalg.svd((a - ca).T @ (b - cb))
    if sv[1] < 1e-6:  # collinear: rotation about that line is undefined
        return None
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return rot, cb - rot @ ca


def load_refs(ref_dir):
    refs = {}
    if os.path.isdir(ref_dir):
        for fn in sorted(os.listdir(ref_dir)):
            if fn.endswith(".json"):
                d = json.load(open(os.path.join(ref_dir, fn)))
                refs[d["name"]] = {int(k): np.array(v) for k, v in d["leds"].items()}
    return refs


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


class PhaseSpaceTracker(Node):
    def __init__(self, args):
        super().__init__("phasespace_tracker")
        self.args = args
        self.led_pub = self.create_publisher(MarkerArray, "/phasespace_leds", 10)
        self.world_pub = self.create_publisher(MarkerArray, "/phasespace_world", LATCHED)
        self.groups_pub = self.create_publisher(String, "/phasespace/groups", LATCHED)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.groups = {}
        self.refs = load_refs(args.ref_dir)
        self.get_logger().info(f"Orientation references in {args.ref_dir}: {sorted(self.refs) or 'none'}")

    # ---------- static world: cameras, floor ----------
    def set_cameras(self, cameras):
        cams = [c for c in cameras if c.cond > 0]
        if not cams:
            return
        transforms, labels = [], []
        half = math.tan(math.radians(self.args.cam_fov) / 2.0)
        cone = Marker(ns="camera_cones", id=0, type=Marker.LINE_LIST, action=Marker.ADD)
        cone.header.frame_id = FRAME
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
        floor.header.frame_id = FRAME
        floor.pose.position.x, floor.pose.position.y = (lo + hi) / 2.0
        floor.pose.position.z = -0.005
        floor.pose.orientation.w = 1.0
        floor.scale.x, floor.scale.y, floor.scale.z = hi[0] - lo[0], hi[1] - lo[1], 0.01
        floor.color.r = floor.color.g = floor.color.b = 0.55
        floor.color.a = 0.35
        origin = self._text("origin", 0, np.array([0.0, 0.0, 0.15]), "PhaseSpace origin", 0.15)

        self.world_pub.publish(MarkerArray(markers=[cone, *labels, floor, origin]))
        self.get_logger().info(
            f"{len(cams)} cameras; floor x [{lo[0]:.2f}, {hi[0]:.2f}] y [{lo[1]:.2f}, {hi[1]:.2f}] m"
        )

    def set_groups(self, groups):
        self.groups = groups
        self.get_logger().info(f"LED groups: {groups}")
        self.groups_pub.publish(String(data=json.dumps({
            "frame": FRAME,
            "groups": {name: {"leds": leds, "oriented": name in self.refs} for name, leds in groups.items()},
        })))

    def _text(self, ns, id, pos, text, size):
        mk = Marker(ns=ns, id=id, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, text=text)
        mk.header.frame_id = FRAME
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = (float(p) for p in pos)
        mk.pose.orientation.w = 1.0
        mk.scale.z = size
        mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
        return mk

    def _transform(self, stamp, child, pos, quat=(0.0, 0.0, 0.0, 1.0)):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = FRAME
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(p) for p in pos)
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = quat
        return t

    # ---------- per frame: LEDs, groups, rigids ----------
    def publish_frame(self, markers, rigids):
        stamp = self.get_clock().now().to_msg()
        group_of = {led: (i, name) for i, (name, leds) in enumerate(self.groups.items()) for led in leds}
        array = MarkerArray()
        array.markers.append(Marker(action=Marker.DELETEALL))
        transforms, seen_by_group = [], {}
        for m in markers:
            if m.cond <= 0:
                continue
            pos = to_ros(m.x, m.y, m.z)
            gi, gname = group_of.get(m.id, (None, None))
            if gname:
                seen_by_group.setdefault(gname, {})[m.id] = pos
            transforms.append(self._transform(stamp, f"{gname}/led_{m.id}" if gname else f"led_{m.id}", pos))
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

        for name, seen in seen_by_group.items():
            pose = fit_pose(self.refs[name], seen) if name in self.refs else None
            if pose:
                rot, c = pose
                transforms.append(self._transform(stamp, name, c, matrix_to_quat(rot)))
            else:  # no reference / too few LEDs: centroid, unrotated
                transforms.append(self._transform(stamp, name, np.mean(list(seen.values()), axis=0)))
        for r in rigids:
            if r.cond > 0:
                transforms.append(self._transform(stamp, f"phasespace_{r.id}", to_ros(*r.pose[:3]),
                                                  quat_to_ros(*r.pose[3:7])))
        if transforms:
            self.tf.sendTransform(transforms)


def capture(owl, args):
    """Average group LED positions for a few seconds; save them relative to their centroid."""
    leds, samples, t_end = None, {}, time.monotonic() + args.capture_secs
    try:
        while time.monotonic() < t_end:
            event = owl.nextEvent(args.timeout)
            if leds is None:
                leds = set(led_groups(owl, args.profile).get(args.capture, [])) or None
            if leds and event and event.type_id == Type.FRAME and "markers" in event:
                for m in event.markers:
                    if m.cond > 0 and m.id in leds:
                        samples.setdefault(m.id, []).append(to_ros(m.x, m.y, m.z))
    finally:
        owl.done()
        owl.close()
    if leds is None:
        sys.exit(f"No LED group named {args.capture!r} in the session profile")
    if len(samples) < 3:
        sys.exit(f"Only LEDs {sorted(samples)} visible; need at least 3")
    mean = {i: np.mean(p, axis=0) for i, p in samples.items()}
    center = np.mean(list(mean.values()), axis=0)
    os.makedirs(args.ref_dir, exist_ok=True)
    path = os.path.join(args.ref_dir, f"{args.capture}.json")
    json.dump({
        "name": args.capture,
        "captured": time.strftime("%Y-%m-%d %H:%M:%S"),
        "note": "LED positions (m) relative to their centroid; this layout = identity orientation (facing +x, Z up)",
        "centroid_at_capture": [round(float(v), 4) for v in center],
        "leds": {str(i): [round(float(v), 5) for v in p - center] for i, p in sorted(mean.items())},
    }, open(path, "w"), indent=2)
    print(f"Saved {path}: LEDs {sorted(mean)}, {sum(map(len, samples.values()))} samples")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="cs-phasespace.cs.umn.edu")
    parser.add_argument("--freq", type=float, default=60.0)
    parser.add_argument("--stream", choices=("tcp", "udp"), default="udp")
    parser.add_argument("--timeout", type=int, default=5000000)
    parser.add_argument("--profile", default=None, help="session profile name (default: server default)")
    parser.add_argument("--cam-fov", type=float, default=60.0, help="camera view-cone full angle, deg")
    parser.add_argument("--cone-length", type=float, default=3.0, help="camera view-cone length, m")
    parser.add_argument("--ref-dir", default=os.path.join(get_package_share_directory("phasespace_ros2"), "trackers"),
                        help="orientation references, one JSON per LED group (default: installed trackers/)")
    parser.add_argument("--capture", metavar="NAME", help="record group NAME's LED layout as its reference and exit")
    parser.add_argument("--capture-secs", type=float, default=3.0)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = PhaseSpaceTracker(args)
    owl = Context()
    owl.open(args.device, f"timeout={args.timeout}")
    owl.initialize(f"timeout={args.timeout} event.cameras=1 event.markers=1 event.rigids=1")
    owl.frequency(args.freq)
    owl.streaming(2 if args.stream == "udp" else 1)
    node.get_logger().info(f"Streaming from {args.device}")

    if args.capture:
        capture(owl, args)
        return

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
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
