#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from c_iartemis_base_controller.msg import vfw_float32


counter =0

def shutdownHandler():
    print "Shutdown"

def topCallback(data):
    message = str(data) + " " + str(counter) + " " \

    logfile_top.write(message + "\n")
    rospy.loginfo(message)

def perieCallback(data):
    message = str(data) + " " + str(counter) + " " \

    logfile_perie.write(message + "\n")
    rospy.loginfo(message)
    

def motorsCarac():
    rospy.init_node("c_iartemis_peri_top_carac")

    # Publish speed to motorss
    pub = rospy.Publisher('/motors/cmd', vfw_float32, queue_size=10)

    # Subscribe top and perie
    rospy.Subscriber("/motors/top", String, topCallback)
    rospy.Subscriber("/motors/perie", String, perieCallback)

    # Send speed...
    rospy.sleep(1.0) # wait init of publisher
    for i in range(0,256,1): # stop at 255
        global counter
        counter = i
        speed = vfw_float32(0, 0, 0, 0, i, 0)
        rospy.loginfo("Send %d", i)
        pub.publish(speed)
        #logfile_perie.write("#" + str(i) + "\n")
        #logfile_top.write("#" + str(i) + "\n")
        rospy.sleep(3)
        #logfile_perie.write("\n")
        #logfile_top.write("\n")

    # Stop motors
    speed = vfw_float32(0, 0, 0, 0, 0, 0)
    pub.publish(speed)
    rospy.sleep(5.0)

if __name__ == "__main__":
    try:
        rospy.on_shutdown(shutdownHandler)
        logfile_perie = open("c_iartemis_peri_carac.out", "w")
        logfile_top = open("c_iartemis_top_carac.out", "w")
        motorsCarac()
    except:
        raise
    finally:
        logfile_perie.close()
        logfile_top.close()
        exit()
        
        
