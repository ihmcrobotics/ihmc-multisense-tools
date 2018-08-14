#!/usr/bin/env python
import rospy
from  sensor_msgs.msg import PointCloud2
import tf
from scan_to_cloud.msg import PointCloud2WithSource
import logging
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

def callback(data) :
    global listener
    global cloudWithSourcePublisher
    try:
        listener.waitForTransform("/multisense/head", "/multisense/head_hokuyo_frame", data.header.stamp, rospy.Duration(0.01))
        #t = listener.getLatestCommonTime("/multisense/head_root", "/multisense/head_hokuyo_frame")
        (trans,rot) = listener.lookupTransform('/multisense/head', '/multisense/head_hokuyo_frame', data.header.stamp)
        msg = PointCloud2WithSource()
        msg.cloud = data
        msg.translation = Point()
        msg.translation.x = trans[0]
        msg.translation.y = trans[1]
        msg.translation.z = trans[2]
        msg.orientation = Quaternion()
        msg.orientation.x = rot[0] 
        msg.orientation.y = rot[1]
        msg.orientation.z = rot[2]
        msg.orientation.w = rot[3]
        cloudWithSourcePublisher.publish(msg) 

    except Exception as e:
        print(e)

if __name__ == '__main__':
    try:
        print('starting ScanToWorlPublisher')
        rospy.init_node('pointCloudWithOriginPublisher')
        listener = tf.TransformListener(True, rospy.Duration.from_sec(30.0))
        cloudWithSourcePublisher = rospy.Publisher('singleScanAsCloudWithSource', PointCloud2WithSource,queue_size=10)
        rospy.Subscriber("/multisense/filtered_cloud", PointCloud2, callback)
        rospy.spin()
 
    except rospy.ROSInterruptException:
        pass
