#!/usr/bin/env python3

# Ova skripta predstavlja ROS servis kojim je implementiran
# unos komandi prilikom odredjenog rezima rada robota

# Ucitavanje potrebnih biblioteka i struktura poruka
import rospy
from domaci_2.srv import komanda, komandaResponse
from std_msgs.msg import String

# Inicijalizacija globalnog publisher-a na topic /komande
# preko kojeg se komunicira sa ostalim skriptama o tome koja komanda
# je upisana
pub = rospy.Publisher('komande', String, queue_size=1)
rospy.init_node('komande', anonymous=False)

# Callback funkcija za ROS servis
# koja publish-uje komande koje su zadate preko konzole
def response_callback(req):

    pub.publish(req.komanda)

    rospy.loginfo('Primljena komanda : ' + req.komanda)

    # Povratna vrednost servisa
    return komandaResponse(True)

# Inicijalizacija ROS servisa za unos komandi
s = rospy.Service('komanda', komanda, response_callback)
rospy.loginfo("Servis za zadavanje komandi robotu je spreman!")
rospy.spin()