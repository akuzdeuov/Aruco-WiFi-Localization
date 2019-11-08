import rospy
from ros_aruco.msg import wlan
import iwlist
import time
from geometry_msgs.msg import PointStamped
from datetime import date
from datetime import datetime


def main():
    pub = rospy.Publisher('wlanInfo', wlan, queue_size=10)
    rospy.Subscriber("camera_pose", PointStamped, callback)
    rospy.init_node('wifi', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    filename = datetime.today().strftime('%Y-%m-%d-%H:%M:%S')
    f = open("WIFI-"+ filename + ".txt","w+")

    while not rospy.is_shutdown():
        #start = time.time()
        content = iwlist.scan(interface='wlp7s0b1')
        cells = iwlist.parse(content)
        #end = time.time()
        #print("Sampling rate:", int(1/(end - start)), "Hz")
        now = rospy.Time.now()
        #time_stamp = str(now.secs + now.nsecs/10e9)
        rospy.loginfo("Timestamp %i sec %i nsec", now.secs, now.nsecs)
        print("Number of access points:", len(cells))
        for i in range(len(cells)):
                temp = cells[i]
                essid = str(temp["essid"])
                channel = str(temp["channel"])
                mac_addr = str(temp["mac"])
                frequency = str(temp["frequency"])
                rss = str(temp["signal_level_dBm"])
                f.write("WIFI;%i %i;%s;%s;%s;%s;%s\r\n" % (now.secs, now.nsecs, essid, channel, mac_addr, frequency, rss))

                print("Point:", i+1, "[INFO]", cells[i])                          
                print("----------------------------------------------")
        rate.sleep() 

    f.close()                
                
                
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




