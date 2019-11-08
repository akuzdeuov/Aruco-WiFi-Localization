import rospy
from ros_aruco.msg import wlan
import iwlist
import time
from datetime import date
from datetime import datetime

def talker():
    pub = rospy.Publisher('wlanInfo', wlan, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    filename = datetime.today().strftime('%Y-%m-%d-%H:%M:%S')
    f = open("WIFI-"+ filename + ".txt","w+")

    while not rospy.is_shutdown():
        msg = wlan();
        #start = time.time()
        content = iwlist.scan(interface='wlp7s0b1')
        cells = iwlist.parse(content)
        #end = time.time()
        #print("Sampling rate:", int(1/(end - start)), "Hz")
        now = rospy.Time.now()
        AppTimestamp = str(now.secs) + "." + str(now.nsecs/10e9)
        rospy.loginfo("Timestamp %i sec %i nsec", now.secs, now.nsecs)
        print("Number of access points:", len(cells))
        for i in range(len(cells)):
                temp = cells[i]
                essid = str(temp["essid"])
                mac_addr = str(temp["mac"]) + ";"
                frequency = str(temp["frequency"])
                rss = str(temp["signal_level_dBm"])

                #if temp["mac"] == "1C:DE:A7:1B:0B:B1":
                msg.header.stamp = now
                msg.mac = temp["mac"]
                msg.signal_strength = int(temp["signal_level_dBm"])
                pub.publish(msg)
                f.write("WIFI;%s;\r\n" % (AppTimestamp))
                #print(mac["signal_level_dBm"])
                #print("MAC Address:", cells[i].)
                print("Point:", i+1, "[INFO]", cells[i])                          
                #print(cells)
                print("----------------------------------------------")
                #time.sleep(1)
        rate.sleep()                
                
                
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass




