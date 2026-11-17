#!/usr/bin/env python3
"""mapper.py - accumulates classified overhead frames (OpenCV HSV masking) into a world-frame
nav_msgs/OccupancyGrid while the drone flies the lawnmower sweep.

  in : /camera/image_raw, /camera/camera_info, /model/X4/odometry, /sweep/done
  out: /map (raw)  /map_inflated (obstacles grown by inflate_radius)  /mapping/final  /debug/obstacle_mask
"""
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from hexacopter_control import hexa_common as hc

LATCH = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class Mapper(Node):
    def __init__(self):
        super().__init__('mapper')
        d = lambda n, v: self.declare_parameter(n, v).value
        res = d('resolution', 0.05)
        x0, y0, x1, y1 = d('x_min', -1.0), d('y_min', -7.0), d('x_max', 17.0), d('y_max', 7.0)
        self.spec = hc.GridSpec(res, x0, y0, round((x1 - x0) / res), round((y1 - y0) / res))
        self.stride = int(d('stride', 3))
        self.inflate_r = d('inflate_radius', 0.35)
        self.min_alt = d('min_altitude', 1.5)
        self.max_tilt = d('max_tilt_deg', 12.0)
        self.save_prefix = os.path.join(os.path.expanduser(d('save_dir', '~')), 'hexacopter_map')
        self.debug = d('debug', True)
        self.acc = hc.MapAccumulator(self.spec, d('weight_sigma', 0.0), int(d('min_hits', 3)), d('occupied_threshold', 0.8))
        self.buf, self.cam, self.final, self.frames = hc.PoseBuffer(), None, False, 0

        self.create_subscription(Odometry, '/model/X4/odometry', self.on_odom, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Bool, '/sweep/done', self.on_done, LATCH)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', LATCH)
        self.inf_pub = self.create_publisher(OccupancyGrid, '/map_inflated', LATCH)
        self.fin_pub = self.create_publisher(Bool, '/mapping/final', LATCH)
        self.dbg_pub = self.create_publisher(Image, '/debug/obstacle_mask', 1)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f'grid {self.spec.w}x{self.spec.h} @ {res} m, origin ({x0}, {y0})')

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.buf.add(hc.stamp_sec(m.header), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def on_info(self, m):
        if self.cam is None:
            self.cam = hc.Camera(m.k, m.width, m.height, self.stride)
            self.get_logger().info(f'camera fx={self.cam.fx:.1f} {self.cam.w}x{self.cam.h}')

    def on_image(self, m):
        if self.final or self.cam is None:
            return
        pose = self.buf.at(hc.stamp_sec(m.header))
        if pose is None:
            return
        p, q = pose
        R = hc.quat_to_R(*q)
        if p[2] < self.min_alt or hc.tilt_deg(R) > self.max_tilt:      # skip take-off / aggressive attitude
            return
        mask = hc.obstacle_mask(hc.image_to_rgb(m))
        is_obs = (mask[np.ix_(self.cam.vi, self.cam.ui)] > 0).ravel()
        x, y, tan_t, ok = hc.rays_to_ground(self.cam.rays, p, R)
        self.acc.add(x, y, tan_t, is_obs, ok)
        self.frames += 1
        if self.debug:
            self.dbg_pub.publish(hc.to_image_msg(Image(), mask, 'mono8', m.header.stamp))

    def grids(self):
        raw = hc.clean_grid(self.acc.grid())
        return raw, hc.inflate_grid(raw, self.inflate_r, self.spec.res)

    def to_msg(self, data):
        o = OccupancyGrid()
        o.header.frame_id = 'map'
        o.header.stamp = self.get_clock().now().to_msg()
        o.info.resolution = float(self.spec.res)
        o.info.width, o.info.height = self.spec.w, self.spec.h
        o.info.origin.position.x, o.info.origin.position.y = float(self.spec.ox), float(self.spec.oy)
        o.info.origin.orientation.w = 1.0
        o.data = data.astype(np.int8).ravel().tolist()
        return o

    def publish(self):
        if self.frames == 0:
            return
        raw, inf = self.grids()
        self.map_pub.publish(self.to_msg(raw))
        self.inf_pub.publish(self.to_msg(inf))

    def on_done(self, m):
        if not m.data or self.final:
            return
        self.final = True
        raw, inf = self.grids()
        self.map_pub.publish(self.to_msg(raw))
        self.inf_pub.publish(self.to_msg(inf))
        hc.save_map(raw, self.spec, self.save_prefix)
        self.fin_pub.publish(Bool(data=True))
        known = float((raw >= 0).mean()) * 100
        self.get_logger().info(f'map final: {self.frames} frames, {known:.0f}% of grid observed, '
                               f'{int((raw >= 50).sum())} occupied cells, saved {self.save_prefix}.pgm/.yaml')


def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
