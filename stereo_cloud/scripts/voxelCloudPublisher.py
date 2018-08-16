#!/usr/bin/env python
import rospy
from  sensor_msgs.msg import PointCloud2
import tf2_ros
from scan_to_cloud.msg import PointCloud2WithSource
import logging
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point


mapFrame = "world"
sensorFrame = "multisense/head"

def callback(data) :
    print("DATA RECEIVED:")
    global listener
    global cloudWithSourcePublisher
    try:
        #listener.waitForTransform(mapFrame, sensorFrame, data.header.stamp, rospy.Duration(0.1))
        #t = listener.getLatestCommonTime("/multisense/head_root", "/multisense/head_hokuyo_frame")
	try:
        	transform = tfBuffer.lookup_transform(mapFrame, sensorFrame, data.header.stamp)
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		print "couldn't lookup transform"
		return

        msg = PointCloud2WithSource()
        msg.cloud = data
        msg.translation = Point()
        msg.translation.x = transform.transform.translation.x
        msg.translation.y = transform.transform.translation.y
        msg.translation.z = transform.transform.translation.z
        msg.orientation = Quaternion()
        msg.orientation.x = transform.transform.rotation.x
        msg.orientation.y = transform.transform.rotation.y
        msg.orientation.z = transform.transform.rotation.z
        msg.orientation.w = transform.transform.rotation.w
        cloudWithSourcePublisher.publish(msg) 
	print "publish successful"

    except Exception as e:
        print(e)

if __name__ == '__main__':
    try:
        print('starting VoxelCloudPublisher')
        rospy.init_node('pointCloudWithOriginPublisher')
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
        #listener = tf.TransformListener(True, rospy.Duration.from_sec(30.0))
        cloudWithSourcePublisher = rospy.Publisher('/multisense/filtered_cloud', PointCloud2WithSource,queue_size=10)
        rospy.Subscriber("/filter_chain/output", PointCloud2, callback)
        rospy.spin()
 
    except rospy.ROSInterruptException:
        pass
