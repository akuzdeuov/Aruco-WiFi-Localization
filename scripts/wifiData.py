import rospy
from ros_aruco.msg import wlan
import iwlist
import time
from geometry_msgs.msg import PointStamped
from datetime import date
from datetime import datetime

global f
filename = datetime.today().strftime('%Y-%m-%d-%H:%M:%S')
f = open("WIFI-POSI-"+ filename + ".txt","w+")

def callback(camera_pose):
    f.write("POSI;%i;%i;%f;%f;%f\r\n" % (camera_pose.header.stamp.secs, camera_pose.header.stamp.nsecs, 
        camera_pose.point.x, camera_pose.point.y, camera_pose.point.z))

    content = iwlist.scan(interface='wlp7s0b1')
    cells = iwlist.parse(content)
    now = rospy.Time.now()
    #rospy.loginfo("Timestamp %i sec %i nsec", now.secs, now.nsecs)
    print("Number of Access Points:", len(cells))
    for i in range(len(cells)):
            temp = cells[i]
            essid = str(temp["essid"])
            channel = str(temp["channel"])
            mac_addr = str(temp["mac"])
            frequency = str(temp["frequency"])
            rss = str(temp["signal_level_dBm"])
            f.write("WIFI;%i;%i;%s;%s;%s;%s;%s\r\n" % (now.secs, now.nsecs, essid, channel, mac_addr, frequency, rss))

            #print("Point:", i+1, "[INFO]", cells[i])                          
            #print("----------------------------------------------")
    print("----------------------------------------------")
    #rospy.sleep(0.2)



def main():
    rospy.init_node('wifi', anonymous=True)
    #pub = rospy.Publisher('wlanInfo', wlan, queue_size=10)
    rospy.Subscriber("camera_pose", PointStamped, callback)
    rospy.spin()
    
    f.close()                
                
                
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




