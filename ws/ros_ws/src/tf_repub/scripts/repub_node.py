#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import tf
from geometry_msgs.msg import TransformStamped


def callback(msg):
    br = tf.TransformBroadcaster()
    trans = msg.transform
    # print(trans)
    print("Pub")
    q1 = [-0.016, 0.023, 0.113, 0.993]
    q2 = [0.829, -0.01, 0.558, 0.015]

    q1[3] = -q1[3]
    qr = tf.transformations.quaternion_multiply(q2, q1)

    # V1 Bag Easy
    # br.sendTransform((trans.translation.x - 0.787, trans.translation.y - 2.178, trans.translation.z - 1.062),
    #                 qr,
    #                 msg.header.stamp,
    #                 "vicon/firefly_sbx/firefly_sbx",
    #                 "world")

    br.sendTransform((trans.translation.x - 0.443, trans.translation.y - 2.062, trans.translation.z - 1.076),
                    qr,
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