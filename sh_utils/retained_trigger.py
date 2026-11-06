#!/usr/bin/env python3
import argparse, signal
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

parser=argparse.ArgumentParser(description='Publish one retained simulation trigger and keep its DDS writer alive until Ctrl+C.')
parser.add_argument('topic',choices=['/triger','/land_triger'])
args=parser.parse_args()
rclpy.init();node=rclpy.create_node('elastic_retained_trigger')
pub=node.create_publisher(PoseStamped,args.topic,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
msg=PoseStamped();msg.header.frame_id='world';msg.header.stamp=node.get_clock().now().to_msg();msg.pose.orientation.w=1.0
pub.publish(msg)
print('Published once to '+args.topic+'; retaining for late subscribers until Ctrl+C.',flush=True)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
