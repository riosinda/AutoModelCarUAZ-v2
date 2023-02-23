#!/usr/bin/env python

import rospy, serial
from std_msgs.msg import String, Int16

def talker(ax_str, ay_str, az_str, gx_str, gy_str, gz_str, rpm_str, dist_str):
    """
    Iniciamos el nodo con los topics de los sensores
    """
    auto_ax = rospy.Publisher('auto/giroscopio/ax', String, queue_size=10)
    auto_ay = rospy.Publisher('auto/giroscopio/ay', String, queue_size=10)
    auto_az = rospy.Publisher('auto/giroscopio/az', String, queue_size=10)
    auto_gx = rospy.Publisher('auto/giroscopio/gx', String, queue_size=10)
    auto_gy = rospy.Publisher('auto/giroscopio/gy', String, queue_size=10)
    auto_gz = rospy.Publisher('auto/giroscopio/gz', String, queue_size=10)
    auto_rpm = rospy.Publisher('auto/rpm', String, queue_size=10)
    auto_dist = rospy.Publisher('auto/dist', String, queue_size=10)
    rospy.init_node('sensores', anonymous=True)
    
    #Se publican los valores obtenidos por el arduino.
    auto_ax.publish(ax_str)
    auto_ay.publish(ay_str)
    auto_az.publish(az_str)
    auto_gx.publish(gx_str)
    auto_gy.publish(gy_str)
    auto_gz.publish(gz_str)
    auto_rpm.publish(rpm_str)
    auto_dist.publish(dist_str)

 
if __name__ == '__main__':
    try:
        while True:
            serArduino = serial.Serial("/dev/ttyUSB1", 115200, timeout=1) #Se define el puerto de conexion del arduino
            vector = serArduino.readline() #Lee lo que el arduino manda por el puerto serial
            vector = vector.split(',')
            print(vector) #Imprime en terminal lo que resive el arduino
            #rospy.loginfo(vector)
            
            if len(vector) == 8: #condicion para que solo resiva los 8 valores de los sensores
                ax_str = vector[0]
                ay_str = vector[1]
                az_str = vector[2]
                gx_str = vector[3]
                gy_str = vector[4]
                gz_str = vector[5]
                rpm_str = vector[6]
                dist_str = vector[7]
                #Se mandan los valores a los topics
                talker(ax_str, ay_str, az_str, gx_str, gy_str, gz_str, rpm_str, dist_str)
        rospy.spin()
                
    except rospy.ROSInterruptException:
        pass