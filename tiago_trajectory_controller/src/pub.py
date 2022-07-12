#!/usr/bin/env python

import rospy
import time
from tiago_trajectory_controller.msg import Control_msg

def pub():
    pub = rospy.Publisher('webcam_coordinates', Control_msg, queue_size=10)
    rospy.init_node('pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg=Control_msg()
    while not rospy.is_shutdown():

	  msg.gesture="raising"	
          pub.publish(msg)
	  #time.sleep(5)
          rate.sleep()
  
if __name__ == '__main__':
    try:
         pub()
    except rospy.ROSInterruptException:
         pass

