"""Measured odometry/joints -> TF; GPU lidar -> scan and self-filtered cloud."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

def split_rotation(q):
    """Planar localization must not cancel measured body roll/pitch."""
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    sy,cy=math.sin(yaw/2),math.cos(yaw/2)
    return sy,cy,(cy*q.x+sy*q.y,cy*q.y-sy*q.x,cy*q.z-sy*q.w,cy*q.w+sy*q.z)

def filter_scan(ranges,angle_min,angle_increment,folded):
    filtered=[]
    for i,v in enumerate(ranges):
        angle=angle_min+i*angle_increment
        x,y=v*math.cos(angle),v*math.sin(angle)
        own=folded and abs(x)<.29 and -.41<y<.30
        filtered.append(math.nan if v<.4 or own else v)
    return filtered

class Sensors(Node):
    def __init__(self):
        super().__init__("simulation_sensors")
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.joints = {}
        self.initial_pose_sent = False
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped,"/initialpose",1)
        self.create_timer(0.2,self.initialize_pose)
        self.create_subscription(Odometry,"/model/hnu2027_sentry/odometry",self.odom,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState,"/model/hnu2027_sentry/joint_state",self.joint,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan,"/simulation/raw_scan",self.scan,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,"/simulation/raw_tunnel_scan/points",
                                 self.cloud,qos_profile_sensor_data)
        self.odom_pub=self.create_publisher(Odometry,"/odom",10)
        self.scan_pub=self.create_publisher(LaserScan,"/scan",qos_profile_sensor_data)
        self.cloud_pub=self.create_publisher(PointCloud2,"/tunnel/cloud",qos_profile_sensor_data)
        self.static_tf.sendTransform(self.transform("base_link","nav_lidar",(0,0,.04),0,
                                                    self.get_clock().now().to_msg()))

    def initialize_pose(self):
        if self.initial_pose_sent or not self.initial_pub.get_subscription_count(): return
        if self.get_clock().now().nanoseconds == 0: return
        pose=PoseWithCovarianceStamped()
        pose.header.frame_id="map";pose.header.stamp=self.get_clock().now().to_msg()
        pose.pose.pose.position.x=-4.0;pose.pose.pose.position.y=3.5
        pose.pose.pose.orientation.w=1.0
        pose.pose.covariance[0]=pose.pose.covariance[7]=0.0025
        pose.pose.covariance[35]=0.0025
        self.initial_pub.publish(pose);self.initial_pose_sent=True

    def transform(self,parent,child,xyz,yaw,stamp):
        t=TransformStamped()
        t.header.frame_id,t.child_frame_id,t.header.stamp=parent,child,stamp
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
        t.transform.rotation.z=math.sin(yaw/2)
        t.transform.rotation.w=math.cos(yaw/2)
        return t

    def odom(self,msg):
        msg.header.frame_id="odom"
        msg.child_frame_id="base_link"
        t=TransformStamped()
        t.header=msg.header;t.child_frame_id="base_footprint"
        t.transform.translation.x=msg.pose.pose.position.x
        t.transform.translation.y=msg.pose.pose.position.y
        t.transform.translation.z=0.0
        q=msg.pose.pose.orientation
        sy,cy,body_q=split_rotation(q)
        t.transform.rotation.z=sy;t.transform.rotation.w=cy
        body=self.transform("base_footprint","base_link",
            (0,0,msg.pose.pose.position.z),0,msg.header.stamp)
        body.transform.rotation.x,body.transform.rotation.y=body_q[:2]
        body.transform.rotation.z,body.transform.rotation.w=body_q[2:]
        self.tf.sendTransform([t,body])
        self.odom_pub.publish(msg)

    def joint(self,msg):
        self.joints=dict(zip(msg.name,msg.position))
        yaw=self.joints.get("big_yaw",0.0)
        c,s=math.cos(yaw),math.sin(yaw)
        self.tf.sendTransform(self.transform("base_link","tunnel_lidar",
            (-.08*c-.06*s,-.08*s+.06*c,.045),yaw,msg.header.stamp))

    def scan(self,msg):
        msg.header.frame_id="nav_lidar"
        msg.range_min=.4
        folded=abs(self.joints.get("fold_roll",0)-math.pi/2)<.04
        msg.ranges=filter_scan(msg.ranges,msg.angle_min,msg.angle_increment,folded)
        self.scan_pub.publish(msg)

    def cloud(self,msg):
        fields={f.name:f for f in msg.fields}
        if not all(k in fields and fields[k].datatype==7 for k in ("x","y","z")):
            return
        dtype=np.dtype({"names":["x","y","z"],"formats":[">f4" if msg.is_bigendian else "<f4"]*3,
                        "offsets":[fields[k].offset for k in ("x","y","z")],"itemsize":msg.point_step})
        # Respect row padding instead of assuming tightly packed sensor data.
        data=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=bytes(msg.data),
                        strides=(msg.row_step,msg.point_step)).reshape(-1)
        x,y,z=[data[k].astype(float) for k in ("x","y","z")]
        yaw=self.joints.get("big_yaw",0.0);c,s=math.cos(yaw),math.sin(yaw)
        bx=c*(x-.08)-s*(y+.06);by=s*(x-.08)+c*(y+.06);bz=z+.045
        folded=abs(self.joints.get("fold_roll",0)-math.pi/2)<.04
        top=.09 if folded else .43
        own=(abs(bx)<.29)&(by>-.41)&(by<.30)&(bz>-.15)&(bz<top)
        keep=np.isfinite(x)&np.isfinite(y)&np.isfinite(z)&~own
        # Preserve original point fields and encoding, dropping robot self-returns.
        raw=np.ndarray((msg.height,msg.width,msg.point_step),dtype=np.uint8,
                       buffer=bytes(msg.data),strides=(msg.row_step,msg.point_step,1))
        selected=raw.reshape(-1,msg.point_step)[keep]
        msg.header.frame_id="tunnel_lidar"
        msg.height=1;msg.width=len(selected);msg.row_step=msg.width*msg.point_step
        msg.data=selected.tobytes();msg.is_dense=True
        self.cloud_pub.publish(msg)

def main():
    rclpy.init()
    n=Sensors()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
