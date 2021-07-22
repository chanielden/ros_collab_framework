#!/usr/bin/env python
import rospy
import numpy as np
import math
from ar_track_alvar_msgs.msg import AlvarMarkers

# Defining empty arrays
trans_arr = np.zeros(3)
quat_arr = np.zeros(4)

# Defining marker ID
marker_id = rospy.get_param("~marker_id",0)
print('launched')

def callback(markers):

    # Reading and storing data into local variables
    for m in markers.markers:
        if m.id == marker_id:
            p = m.pose.pose
            trans_arr = np.array([p.position.x,p.position.y,p.position.z])
            quat_arr = np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])

            # Printing data to verify
            print('For ID %d POSITION:',0)
            print('\nX: %2.4f \nY: %2.4f \nZ: %2.4f \n' %(trans_arr[0],trans_arr[1],trans_arr[2]))
            print('For ID %d QUATERNION:',0)
            print('\nX: %2.4f \nY: %2.4f \nZ: %2.4f \nW: %2.4f \n' %(quat_arr[0],quat_arr[1],quat_arr[2],quat_arr[3]))
            print('In zone?')
            
        else:
            print('NO MARKER FOUND')

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # read the parameter from ROS parameter server
    #frame_topic = rospy.get_param('~pub_topic')

    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    print('subbed')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()