#!/usr/bin/env python

import rospy, serial
from std_msgs.msg import String, Int16

def talker(d1, d2, d3, d4, d5, d6, d7, d8):
    """
    Iniciamos el nodo con los topics
    """
    auto_vel = rospy.Publisher('auto/vel', Int16, queue_size=10)
    auto_ste = rospy.Publisher('auto/ste', String, queue_size=10)
    auto_rpm = rospy.Publisher('auto/rpm', String, queue_size=10)
    auto_dist = rospy.Publisher('auto/dist', String, queue_size=10)
    auto_ax = rospy.Publisher('auto/giroscopio/ax', String, queue_size=10)
    auto_ay = rospy.Publisher('auto/giroscopio/ay', String, queue_size=10)
    auto_az = rospy.Publisher('auto/giroscopio/az', String, queue_size=10)
    auto_gx = rospy.Publisher('auto/giroscopio/gx', String, queue_size=10)
    auto_gy = rospy.Publisher('auto/giroscopio/gy', String, queue_size=10)
    auto_gz = rospy.Publisher('auto/giroscopio/gz', String, queue_size=10)
    rospy.init_node('tuCola', anonymous=True)
    
    while not rospy.is_shutdown():
        ax_str = d1
        ay_str = d2
        az_str = d3
        gx_str = d4
        gy_str = d5
        gz_str = d6
        vel_str = d7
        dist_str = d8
        auto_ax.publish(ax_str)
        auto_ay.publish(ay_str)
        auto_az.publish(az_str)
        auto_gx.publish(gx_str)
        auto_gy.publish(gy_str)
        auto_gz.publish(gz_str)
        auto_vel.publish(vel_str)
        auto_dist.publish(dist_str)
        break

 
if __name__ == '__main__':
    try:
        while True:
            serArduino = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)
            vector = serArduino.readline()
            vector = vector.split(',')
            rospy.loginfo(vector)
            
            if len(vector) == 8:
                ax_str = vector[0]
                ay_str = vector[1]
                az_str = vector[2]
                gx_str = vector[3]
                gy_str = vector[4]
                gz_str = vector[5]
                vel_str = vector[6]
                dist_str = vector[7]
                talker(ax_str, ay_str, az_str, gx_str, gy_str, gz_str, vel_str, dist_str)
            
        rospy.spin()
                

    except rospy.ROSInterruptException:
        pass