#!/usr/bin/env python3
"""follower.py - flies the drone along the planned A* path (pure-pursuit carrot + P control).

Velocity commands go to the multicopter velocity controller in the body frame, like wp.py. The drone holds
`altitude` while following; it hovers `dwell` seconds whenever it passes within `victim_radius` of a victim.
Starts by itself when /plan/path arrives (wp.py has released the twist topic after /sweep/done).

  in : /plan/path  /victims  /model/X4/odometry
  out: /X4/gazebo/command/twist  /X4/enable  /drone/pose  /drone/trail  /mission/complete
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from hexacopter_control import hexa_common as hc

LATCH = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        d = lambda n, v: self.declare_parameter(n, v).value
        self.alt = d('altitude', 2.0)
        self.kpz, self.vzmax = d('kp_z', 1.0), d('vz_max', 0.6)
        self.vr, self.dwell = d('victim_radius', 0.4), d('dwell', 1.5)
        self.tp = dict(lookahead=d('lookahead', 0.6), vcruise=d('speed', 1.0), kp=d('kp_xy', 1.2), tol=d('goal_tol', 0.15))
        self.p, self.yaw = None, 0.0
        self.state, self.trk, self.victims, self.visited, self.until = 'IDLE', None, [], set(), 0.0
        self.err, self.trail = [], Path()
        self.trail.header.frame_id = 'map'

        self.create_subscription(Odometry, '/model/X4/odometry', self.on_odom, 10)
        self.create_subscription(Path, '/plan/path', self.on_path, LATCH)
        self.create_subscription(PoseArray, '/victims', self.on_victims, LATCH)
        self.cmd_pub = self.create_publisher(Twist, '/X4/gazebo/command/twist', 10)
        self.en_pub = self.create_publisher(Bool, '/X4/enable', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.trail_pub = self.create_publisher(Path, '/drone/trail', 1)
        self.done_pub = self.create_publisher(Bool, '/mission/complete', LATCH)
        self.create_timer(0.05, self.step)
        self.create_timer(0.5, self.draw)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.p = (p.x, p.y, p.z)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_victims(self, m):
        self.victims = [(q.position.x, q.position.y) for q in m.poses]

    def on_path(self, m):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]
        if len(pts) < 2:
            return
        self.trk = hc.PathTracker(pts, **self.tp)
        self.visited, self.err, self.state = set(), [], 'FOLLOW'
        self.get_logger().info(f'following {len(pts)} waypoints at {self.alt} m')

    def send(self, vx, vy):
        vz = max(-self.vzmax, min(self.vzmax, self.kpz * (self.alt - self.p[2])))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = c * vx + s * vy, -s * vx + c * vy, vz     # world -> body
        self.cmd_pub.publish(t)

    def step(self):
        self.en_pub.publish(Bool(data=True))
        if self.p is None or self.state in ('IDLE', 'DONE'):
            return
        x, y = self.p[0], self.p[1]
        if self.state == 'DWELL':
            if time.monotonic() >= self.until:
                self.state = 'FOLLOW'
            else:
                self.send(0.0, 0.0)
                return
        for i, v in enumerate(self.victims):
            if i not in self.visited and math.hypot(v[0] - x, v[1] - y) < self.vr:
                self.visited.add(i)
                self.state, self.until = 'DWELL', time.monotonic() + self.dwell
                self.get_logger().info(f'victim {i + 1} reached at ({v[0]:.2f}, {v[1]:.2f}) '
                                       f'[{len(self.visited)}/{len(self.victims)}]')
                self.send(0.0, 0.0)
                return
        v, done, xt = self.trk.update((x, y))
        self.err.append(xt)
        if done:
            self.state = 'DONE'
            self.send(0.0, 0.0)
            r = math.sqrt(sum(e * e for e in self.err) / len(self.err))
            self.get_logger().info(f'path complete: {len(self.visited)}/{len(self.victims)} victims visited, '
                                   f'cross-track max={max(self.err):.3f} rms={r:.3f}')
            self.done_pub.publish(Bool(data=True))
            return
        self.send(float(v[0]), float(v[1]))

    def draw(self):
        if self.p is None:
            return
        ps = PoseStamped()
        ps.header.frame_id, ps.header.stamp = 'map', self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = self.p
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.pose_pub.publish(ps)
        self.trail.poses = (list(self.trail.poses) + [ps])[-4000:]
        self.trail.header.stamp = ps.header.stamp
        self.trail_pub.publish(self.trail)


def main():
    rclpy.init()
    node = Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
