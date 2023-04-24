#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import tf
from geometry_msgs.msg import TransformStamped


def callback(msg):
    br = tf.TransformBroadcaster()
    trans = msg.transform
    # print(trans)
    br.sendTransform((trans.translation.x, trans.translation.y, trans.translation.z),
                    (trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w),
                    msg.header.stamp,
                    "vicon/firefly_sbx/firefly_sbx",
                    "world")
    
def repub_node():
    

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('repub_node', anonymous=True)

    rospy.Subscriber("/vicon/firefly_sbx/firefly_sbx", TransformStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    repub_node()