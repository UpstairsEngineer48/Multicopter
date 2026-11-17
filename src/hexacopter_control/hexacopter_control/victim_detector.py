#!/usr/bin/env python3
"""victim_detector.py - finds the coloured victim plates (HSV blobs), projects each centroid onto the
ground in the map frame, merges repeated sightings and publishes the confirmed victims.

  in : /camera/image_raw, /camera/camera_info, /model/X4/odometry, /sweep/done
  out: /victims (PoseArray, map)  /victims_markers  /victims/final  /debug/victims
"""
import json
import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from hexacopter_control import hexa_common as hc

LATCH = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
RGBA = {'red': (1.0, 0.0, 0.0), 'green': (0.0, 1.0, 0.0), 'blue': (0.1, 0.3, 1.0)}


class VictimDetector(Node):
    def __init__(self):
        super().__init__('victim_detector')
        d = lambda n, v: self.declare_parameter(n, v).value
        self.min_area = d('min_area', 250.0)
        self.min_obs = int(d('min_observations', 3))
        self.min_alt = d('min_altitude', 1.5)
        self.max_tilt = d('max_tilt_deg', 12.0)
        self.save_path = os.path.join(os.path.expanduser(d('save_dir', '~')), 'victims.json')
        self.debug = d('debug', True)
        self.tracker = hc.VictimTracker(d('merge_radius', 0.35))
        self.buf, self.cam, self.final, self.sig = hc.PoseBuffer(), None, False, None

        self.create_subscription(Odometry, '/model/X4/odometry', self.on_odom, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Bool, '/sweep/done', self.on_done, LATCH)
        self.pose_pub = self.create_publisher(PoseArray, '/victims', LATCH)
        self.mark_pub = self.create_publisher(MarkerArray, '/victims_markers', LATCH)
        self.fin_pub = self.create_publisher(Bool, '/victims/final', LATCH)
        self.dbg_pub = self.create_publisher(Image, '/debug/victims', 1)
        self.create_timer(1.0, self.publish)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.buf.add(hc.stamp_sec(m.header), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def on_info(self, m):
        if self.cam is None:
            self.cam = hc.Camera(m.k, m.width, m.height, 8)

    def on_image(self, m):
        if self.final or self.cam is None:
            return
        pose = self.buf.at(hc.stamp_sec(m.header))
        if pose is None:
            return
        p, q = pose
        R = hc.quat_to_R(*q)
        if p[2] < self.min_alt or hc.tilt_deg(R) > self.max_tilt:
            return
        img = hc.image_to_rgb(m)
        blobs = hc.detect_blobs(img, self.min_area)
        for name, u, v, _ in blobs:
            x, y, _, ok = hc.rays_to_ground(self.cam.pixel_ray(u, v), p, R, hc.VICTIM_Z)
            if ok[0]:
                self.tracker.update(name, float(x[0]), float(y[0]))
        if self.debug:
            vis = img.copy()
            for name, u, v, _ in blobs:
                cv2.circle(vis, (int(u), int(v)), 14, hc.DRAW_RGB[name], 3)
                cv2.putText(vis, name, (int(u) + 16, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            self.dbg_pub.publish(hc.to_image_msg(Image(), vis, 'rgb8', m.header.stamp))

    def publish(self):
        vs = self.tracker.confirmed(self.min_obs)
        sig = tuple((n, round(x, 2), round(y, 2)) for n, x, y, _ in vs)
        if sig == self.sig:
            return
        self.sig = sig
        stamp = self.get_clock().now().to_msg()
        pa = PoseArray()
        pa.header.frame_id, pa.header.stamp = 'map', stamp
        poses, marks = [], []
        clear = Marker()
        clear.action = Marker.DELETEALL
        marks.append(clear)
        for i, (name, x, y, n) in enumerate(vs):
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, 0.05
            pose.orientation.w = 1.0
            poses.append(pose)
            for kind, ns, scale, z in ((Marker.SPHERE, 'victim', 0.35, 0.2), (Marker.TEXT_VIEW_FACING, 'label', 0.3, 0.7)):
                mk = Marker()
                mk.header.frame_id, mk.header.stamp = 'map', stamp
                mk.ns, mk.id, mk.type, mk.action = ns, i, kind, Marker.ADD
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = x, y, z
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = mk.scale.z = scale
                r, g, b = RGBA[name] if kind == Marker.SPHERE else (1.0, 1.0, 1.0)
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, 0.9
                if kind == Marker.TEXT_VIEW_FACING:
                    mk.text = f'{i + 1}:{name}'
                marks.append(mk)
        pa.poses = poses
        ma = MarkerArray()
        ma.markers = marks
        self.pose_pub.publish(pa)
        self.mark_pub.publish(ma)

    def on_done(self, m):
        if not m.data or self.final:
            return
        self.final = True
        self.sig = None
        self.publish()
        vs = self.tracker.confirmed(self.min_obs)
        with open(self.save_path, 'w') as f:
            json.dump([{'id': i + 1, 'color': n, 'x': round(x, 3), 'y': round(y, 3), 'sightings': c}
                       for i, (n, x, y, c) in enumerate(vs)], f, indent=1)
        self.fin_pub.publish(Bool(data=True))
        cnt = {c: sum(1 for v in vs if v[0] == c) for c in hc.VICTIM_COLOURS}
        self.get_logger().info(f'victims final: {len(vs)} confirmed {cnt}, saved {self.save_path}')


def main():
    rclpy.init()
    node = VictimDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
