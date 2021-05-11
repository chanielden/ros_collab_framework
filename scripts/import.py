#!/usr/bin/env python
import rospy
from ros_openpose.msg import Frame

def callback(msg):
    text = [bodyPart.pixel for person in msg.persons for bodyPart in person.bodyParts]
    rospy.loginfo('%s\n' % text)
    # rospy.loginfo(rospy.get_caller_id(), data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # read the parameter from ROS parameter server
    #frame_topic = rospy.get_param('~pub_topic')

    rospy.Subscriber("frame", Frame, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()