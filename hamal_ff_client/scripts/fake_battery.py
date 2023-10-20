#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import BatteryState

def publish_battery_state():
    rospy.init_node('battery_state_publisher')
    pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
    rate = rospy.Rate(1)  # Mesajı saniyede bir yayınlayın

    battery_msg = BatteryState()
    battery_msg.voltage = 24.0  # Batarya gerilimi
    battery_msg.current = 2.0   # Batarya akımı
    battery_msg.charge = 100.0  # Şarj yüzdesi (tam dolu)
    battery_msg.capacity = 100.0  # Batarya kapasitesi
    battery_msg.percentage = 0.8

    while not rospy.is_shutdown():
        pub.publish(battery_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_battery_state()
    except rospy.ROSInterruptException:
        pass
