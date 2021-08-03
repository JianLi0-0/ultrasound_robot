#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import time

time.sleep(10)

rospy.init_node('ur_script_node', anonymous=True)
save_tf_command_publisher = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)
rate = rospy.Rate(10)


while not rospy.is_shutdown():
    text = input("Input 1 for starting free drive mode.\n")
    t0 = time.time()
    print("Free drive mode Started.")
    while not rospy.is_shutdown():
        if text == '1':
            save_tf_command_publisher.publish("freedrive_mode()")
        else:
            break
        if time.time()-t0 > 10:
            # save_tf_command_publisher.publish("end_freedrive_mode()")
            print("10s. Free drive mode Stopped.")
            break
        rate.sleep()