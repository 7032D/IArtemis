#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from c_iartemis_base_controller.msg import vfw_float32

def shutdownHandler():
    print "Shutdown"

def odomCallback(data):
    message = str(data.lfw) + " " \
        + str(data.lmw) + " " \
        + str(data.lrw) + " " \
        + str(data.rfw) + " " \
        + str(data.rmw) + " " \
        + str(data.rrw)
    logfile.write(message + "\n")
    rospy.loginfo(message)
    

def motorsCarac():
    rospy.init_node("c_iartemis_motors_carac")

    # Publish speed to motorss
    pub = rospy.Publisher('/motors/cmd', vfw_float32, queue_size=10)

    # Subscribe odom of motorss
    rospy.Subscriber("/motors/rpm_state", vfw_float32, odomCallback)

    # Send speed...
    rospy.sleep(1.0) # wait init of publisher
    for i in range(0,-256,-1): # stop at 255
        speed = vfw_float32(i, i, i, i, i, i)
        rospy.loginfo("Send %d to iartemis motors.", i)
        pub.publish(speed)
        logfile.write("#" + str(i) + "\n")
        rospy.sleep(4)
        logfile.write("\n")

    # Stop motors
    speed = vfw_float32(0, 0, 0, 0, 0, 0)
    pub.publish(speed)
    rospy.sleep(5.0)

if __name__ == "__main__":
    try:
        rospy.on_shutdown(shutdownHandler)
        logfile = open("c_iartemis_motors_carac.out", "w")
        motorsCarac()
    except:
        raise
    finally:
        logfile.close()
        exit()
        
        
