#!/usr/bin/env python3
"""planner.py - A* over the inflated occupancy grid from the start to every victim.

Victims are visited nearest-first (nearest-neighbour tour, straight-line distance); every leg is planned
with 8-connected A*, line-of-sight smoothed, resampled, and the whole tour is published as one nav_msgs/Path.
Planning runs once, when the sweep is done and both the final map and the final victim list have arrived
(publish std_msgs/Empty on /replan to run it again).

  in : /map_inflated  /victims  /mapping/final  /victims/final  /sweep/done
  out: /plan/path (Path, map)  /plan/markers
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, Empty
from visualization_msgs.msg import Marker, MarkerArray
from hexacopter_control import hexa_common as hc

LATCH = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        d = lambda n, v: self.declare_parameter(n, v).value
        self.start = (d('start_x', 0.0), d('start_y', 0.0))
        self.unknown_blocked = d('unknown_blocked', True)
        self.start_clear = d('start_clear_radius', 1.0)    # take-off pad was never mapped well: force it free
        self.spacing = d('waypoint_spacing', 0.15)
        self.smooth = d('smooth', True)
        self.go_home = d('return_home', True)
        self.grid, self.victims, self.flags = None, None, set()
        self.planned = False

        self.create_subscription(OccupancyGrid, '/map_inflated', lambda m: self.set('grid', m), LATCH)
        self.create_subscription(PoseArray, '/victims', lambda m: self.set('victims', m), LATCH)
        for t in ('/mapping/final', '/victims/final', '/sweep/done'):
            self.create_subscription(Bool, t, lambda m, t=t: self.flag(t, m), LATCH)
        self.create_subscription(Empty, '/replan', lambda m: self.plan(), 10)
        self.path_pub = self.create_publisher(Path, '/plan/path', LATCH)
        self.mark_pub = self.create_publisher(MarkerArray, '/plan/markers', LATCH)

    def set(self, name, m):
        setattr(self, name, m)
        self.try_plan()

    def flag(self, t, m):
        if m.data:
            self.flags.add(t)
        self.try_plan()

    def try_plan(self):
        if (not self.planned and self.grid is not None and self.victims is not None
                and {'/mapping/final', '/victims/final', '/sweep/done'} <= self.flags):
            self.plan()

    def plan(self):
        if self.grid is None or self.victims is None:
            return
        self.planned = True
        spec = hc.GridSpec.from_info(self.grid.info)
        data = np.array(self.grid.data, np.int8).reshape(spec.h, spec.w)
        blocked = data >= 50
        if self.unknown_blocked:
            blocked |= data < 0
        sc = spec.cell(*self.start)
        yy, xx = np.ogrid[:spec.h, :spec.w]
        blocked[((xx - sc[0]) ** 2 + (yy - sc[1]) ** 2) * spec.res ** 2 <= self.start_clear ** 2] = False
        snap_r = int(round(1.0 / spec.res))

        todo = [(p.position.x, p.position.y) for p in self.victims.poses]
        cur_w, cur_c = self.start, hc.snap_free(blocked, sc, snap_r)
        legs, order, lost = [], [], []
        if cur_c is None:
            self.get_logger().error('start cell is blocked and no free cell nearby')
            return
        while todo:
            v = min(todo, key=lambda a: math.hypot(a[0] - cur_w[0], a[1] - cur_w[1]))
            todo.remove(v)
            gc = hc.snap_free(blocked, spec.cell(*v), snap_r)
            cells = hc.astar(blocked, cur_c, gc) if gc else None
            if cells is None:
                lost.append(v)
                self.get_logger().warn(f'no path to victim at ({v[0]:.2f}, {v[1]:.2f})')
                continue
            legs.append(self.finish(cells, blocked, spec))
            order.append(v)
            cur_w, cur_c = v, gc
        if self.go_home and order:
            sc2 = hc.snap_free(blocked, sc, snap_r)
            cells = hc.astar(blocked, cur_c, sc2)
            if cells:
                legs.append(self.finish(cells, blocked, spec))
        if not legs:
            self.get_logger().error('nothing planned')
            return
        pts = [legs[0]] + [l[1:] for l in legs[1:]]
        xy = np.vstack(pts)
        length = float(np.hypot(*np.diff(xy, axis=0).T).sum())
        self.get_logger().info(f'planned {len(order)}/{len(order) + len(lost)} victims, {len(legs)} legs, '
                               f'{length:.1f} m, {len(xy)} waypoints')
        self.publish(xy, order)

    def finish(self, cells, blocked, spec):
        if self.smooth:
            cells = hc.simplify(cells, blocked)
        xy = np.array([spec.world(ix, iy) for ix, iy in cells])
        return hc.resample(xy, self.spacing)

    def publish(self, xy, order):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id, path.header.stamp = 'map', stamp
        poses = []
        for x, y in xy:
            ps = PoseStamped()
            ps.header.frame_id, ps.header.stamp = 'map', stamp
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            poses.append(ps)
        path.poses = poses
        marks = []
        clear = Marker()
        clear.action = Marker.DELETEALL
        marks.append(clear)
        for i, (x, y) in enumerate(order):
            mk = Marker()
            mk.header.frame_id, mk.header.stamp = 'map', stamp
            mk.ns, mk.id, mk.type, mk.action = 'visit_order', i, Marker.TEXT_VIEW_FACING, Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = x, y + 0.5, 0.3
            mk.pose.orientation.w = 1.0
            mk.scale.z = 0.4
            mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
            mk.text = f'#{i + 1}'
            marks.append(mk)
        ma = MarkerArray()
        ma.markers = marks
        self.path_pub.publish(path)
        self.mark_pub.publish(ma)


def main():
    rclpy.init()
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
