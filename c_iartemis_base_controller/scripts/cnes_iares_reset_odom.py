#!/usr/bin/env python
import rospy
import roslib
import tf

#Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

#roslib.load_manifest('odom_publisher')

def publish_odom() :

    print "publish_odom"
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 10)
    tf_br = tf.TransformBroadcaster()
    rospy.init_node('odom_reset', anonymous=True)
    rospy.sleep(2.0) # wait init of publisher

    publish_odom_tf = True

    frame_id = 'odom'
    child_frame_id = 'base_link'

    msg = Odometry()
    msg.header.stamp = rospy.Time.now
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id

    msg.pose.pose.position = Point(0,0,0)
    print msg.pose.pose.position
    msg.pose.pose.orientation  = Quaternion(0,0,0,0)
    print msg.pose.pose.orientation
    
    # Publish odometry message
    odom_pub.publish(msg)

    # Publish tf
    #if publish_odom_tf:
    #      tf_br.sendTransform(pos, ori, msg.header.stamp, msg.child_frame_id, msg.header.frame_id)

if __name__ == "__main__":
    try:
        publish_odom()
    except rospy.ROSInterruptException:
        pass

        
            
