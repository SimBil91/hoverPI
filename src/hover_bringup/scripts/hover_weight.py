#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
import time
import sys
import RPi.GPIO as GPIO
from hx711 import HX711

def weightPub():
    hx = HX711(2, 4)
    hx.set_reading_format("MSB", "MSB")
    hx.set_reference_unit(-454)
    hx.reset()
    hx.tare()
    pub = rospy.Publisher('weight', Float32, queue_size=10)
    rospy.init_node('weight_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        val = hx.get_weight(5)
        hx.power_down()
        hx.power_up()
        pub.publish(val/1000.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        weightPub()
    except rospy.ROSInterruptException:
        pass