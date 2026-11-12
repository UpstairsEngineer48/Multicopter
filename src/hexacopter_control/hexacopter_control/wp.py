import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

BODY=True
KP=1.0
VMAX=1.5
TOL=0.2
WPS=[(0,0,2),(3,0,2),(3,3,2),(0,3,2),(0,0,2)]

class Wp(Node):
    def __init__(self):
        super().__init__('wp')
        self.pub=self.create_publisher(Twist,'/X4/gazebo/command/twist',10)
        self.en=self.create_publisher(Bool,'/X4/enable',10)
        self.create_subscription(Odometry,'/model/X4/odometry',self.cb,10)
        self.p=None
        self.yaw=0.0
        self.i=0
        self.a=None
        self.err=[]
        self.done=False
        self.create_timer(0.05,self.step)

    def cb(self,m):
        p=m.pose.pose.position
        q=m.pose.pose.orientation
        self.p=(p.x,p.y,p.z)
        self.yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def xt(self,a,b,p):
        ab=[b[k]-a[k] for k in range(3)]
        ap=[p[k]-a[k] for k in range(3)]
        L=sum(v*v for v in ab)
        t=max(0.0,min(1.0,sum(ab[k]*ap[k] for k in range(3))/L)) if L>0 else 0.0
        return math.sqrt(sum((a[k]+t*ab[k]-p[k])**2 for k in range(3)))

    def step(self):
        self.en.publish(Bool(data=True))
        if self.p is None:
            return
        if self.i>=len(WPS):
            self.pub.publish(Twist())
            if not self.done and self.err:
                self.done=True
                r=math.sqrt(sum(e*e for e in self.err)/len(self.err))
                self.get_logger().info(f'max={max(self.err):.3f} rms={r:.3f} n={len(self.err)}')
            return
        if self.a is None:
            self.a=self.p
        if self.i>0:
            self.err.append(self.xt(self.a,WPS[self.i],self.p))
        ex,ey,ez=[w-c for w,c in zip(WPS[self.i],self.p)]
        d=math.sqrt(ex*ex+ey*ey+ez*ez)
        if d<TOL:
            self.i+=1
            self.a=WPS[self.i-1]
            return
        vx,vy,vz=KP*ex,KP*ey,KP*ez
        if BODY:
            c,s=math.cos(self.yaw),math.sin(self.yaw)
            vx,vy=c*vx+s*vy,-s*vx+c*vy
        n=math.sqrt(vx*vx+vy*vy+vz*vz)
        k=min(1.0,VMAX/n) if n>0 else 1.0
        t=Twist()
        t.linear.x,t.linear.y,t.linear.z=vx*k,vy*k,vz*k
        self.pub.publish(t)

def main():
    rclpy.init()
    rclpy.spin(Wp())

if __name__=='__main__':
    main()