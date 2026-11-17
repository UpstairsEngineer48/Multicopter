import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

BODY=True
SWEEP=True
KP=1.0
TOL=0.2
ALT=2.0
FOV=1.0472
ASPECT=480/640            # image height / width -> along-track footprint is narrower than the swath
OVERLAP=0.2
CAM_DZ=0.08
CAM_DX=0.20               # camera sits 0.2 m ahead of base_link (model.sdf)
AREA=(2.0,-5.6,15.6,5.6)  # region the camera footprint must cover: x0,y0,x1,y1 (world covers x~2.7-15.3, y~+-5.5)
VMAX=0.8 if SWEEP else 1.5
LATCH=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)

def swath(alt,fov,cam_dz=CAM_DZ):
    return 2*(alt-cam_dz)*math.tan(fov/2)

def lawnmower(x0,y0,x1,y1,z,fov,overlap=0.2):
    w=swath(z,fov)
    step=w*(1-overlap)
    n=max(1,math.ceil(((y1-y0)-w)/step)+1)
    gap=((y1-y0)-w)/max(1,n-1)
    # inset the ends of each line so the footprint (not the drone) covers [x0,x1]
    hx=(z-CAM_DZ)*math.tan(fov/2)*ASPECT
    xa,xb=x0+hx-CAM_DX,x1-hx-CAM_DX
    if xb<xa:
        xa=xb=(x0+x1)/2-CAM_DX
    wps=[]
    for i in range(n):
        y=y0+w/2+i*gap
        a,b=(xa,xb) if i%2==0 else (xb,xa)
        wps+=[(a,y,z),(b,y,z)]
    return wps

if SWEEP:
    WPS=[(0,0,ALT)]+lawnmower(*AREA,ALT,FOV,OVERLAP)+[(0,0,ALT)]
else:
    WPS=[(0,0,2),(3,0,2),(3,3,2),(0,3,2),(0,0,2)]

class Wp(Node):
    def __init__(self):
        super().__init__('wp')
        self.pub=self.create_publisher(Twist,'/X4/gazebo/command/twist',10)
        self.en=self.create_publisher(Bool,'/X4/enable',10)
        self.fin=self.create_publisher(Bool,'/sweep/done',LATCH)
        self.create_subscription(Odometry,'/model/X4/odometry',self.cb,10)
        self.p=None
        self.yaw=0.0
        self.i=0
        self.a=None
        self.err=[]
        self.done=False
        self.create_timer(0.05,self.step)
        if SWEEP:
            self.get_logger().info(f'sweep: swath={swath(ALT,FOV):.2f} m, {len(WPS)} waypoints, speed cap {VMAX} m/s')

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
            if not self.done:
                self.done=True
                self.pub.publish(Twist())              # last command = hover, then release the topic to the follower
                self.fin.publish(Bool(data=True))      # latched: mapper / detector / planner start their final stage
                if self.err:
                    r=math.sqrt(sum(e*e for e in self.err)/len(self.err))
                    self.get_logger().info(f'sweep done: max={max(self.err):.3f} rms={r:.3f} n={len(self.err)}')
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
