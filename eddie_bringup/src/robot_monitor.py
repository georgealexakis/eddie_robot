#!/usr/bin/env python
import rospy
import os
import subprocess
import time
# Psutil 5.8.0
import psutil
from std_msgs.msg import UInt64, UInt64MultiArray, String


def monitor():
    # Init node and rate
    rospy.init_node('robot_monitor', anonymous=True)
    rate = rospy.Rate(1)
    # Publishers
    pub_network = rospy.Publisher(
        '/controller/statistics', String, queue_size=10)
    pub_battery = rospy.Publisher('battery', UInt64, queue_size=10)
    pub_cpu_percent = rospy.Publisher(
        'cpu_percent', UInt64MultiArray, queue_size=10)
    pub_temperatures = rospy.Publisher(
        'temperatures', UInt64MultiArray, queue_size=10)
    pub_cpu_count = rospy.Publisher('cpu_count', UInt64, queue_size=10)
    pub_memory_percent = rospy.Publisher(
        'memory_percent', UInt64, queue_size=10)
    # Start publishing
    while not rospy.is_shutdown():
        # Network signal strength
        networkStats = String()
        networkStats.data = 'No signal'
        cmd = subprocess.Popen(
            'iwconfig wlan0', shell=True, stdout=subprocess.PIPE)
        for line in cmd.stdout:
            if 'Link Quality' in line:
                networkStats.data = line.strip().replace(
                    'Link Quality', 'L.Q').replace('Signal level', 'S.L')
            elif 'Not-Associated' in line:
                networkStats.data = 'No signal'
        if(networkStats.data == 'No signal'):
            cmd = subprocess.Popen(
                'iwconfig wlan1', shell=True, stdout=subprocess.PIPE)
            for line in cmd.stdout:
                if 'Link Quality' in line:
                    networkStats.data = line.strip().replace(
                        'Link Quality', 'L.Q').replace('Signal level', 'S.L')
                elif 'Not-Associated' in line:
                    networkStats.data = 'No signal'
        pub_network.publish(networkStats)
        # CPU usage per core
        cpu = UInt64MultiArray()
        cpu.data = []
        # CPU and GPU temperatures
        temperatures = UInt64MultiArray()
        temperatures.data = [0, 0]
        # Temperatures
        temperaturesFile = os.popen(
            'cat /sys/devices/virtual/thermal/thermal_zone*/temp').read()
        # Publish topics
        if(temperaturesFile != "cat: '/sys/devices/virtual/thermal/thermal_zone*/temp': No such file or directory"):
            for i in range(6):
                temperature = os.popen(
                    'cat /sys/devices/virtual/thermal/thermal_zone' + str(i) + '/temp').read()
                temperatureType = os.popen(
                    'cat /sys/devices/virtual/thermal/thermal_zone' + str(i) + '/type').read()
                if(temperatureType.strip() == 'CPU-therm'):
                    temperatures.data[0] = int(temperature.strip())
                if(temperatureType.strip() == 'GPU-therm'):
                    temperatures.data[1] = int(temperature.strip())
            pub_temperatures.publish(temperatures)
        if(psutil.cpu_percent()):
            for i in psutil.cpu_percent(interval=1, percpu=True):
                cpu.data.insert(0, i)
            pub_cpu_percent.publish(cpu)
        if(psutil.sensors_battery()):
            pub_battery.publish(UInt64(psutil.sensors_battery().percent))
        if(psutil.cpu_count()):
            pub_cpu_count.publish(UInt64(psutil.cpu_count()))
        if(psutil.virtual_memory()):
            pub_memory_percent.publish(UInt64(psutil.virtual_memory().percent))
        # Sleep for a while
        rate.sleep()


if __name__ == '__main__':
    try:
        monitor()
    except rospy.ROSInterruptException:
        pass
