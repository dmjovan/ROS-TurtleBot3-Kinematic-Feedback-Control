#!/usr/bin/env python3

# Ova skripta predstavlja ROS servis kojim je implementiran
# izbor moda rada robota

# Ucitavanje potrebnih biblioteka i struktura poruka
import rospy
from ROS_TurtleBot3_Kinematic_Feedback_Control.srv import izbor_moda, izbor_modaResponse
from std_msgs.msg import String

# Kreiranje jednog globalnog publisher-a na topic /mod_robota 
# preko kojeg ce se komunicirati u drugim skriptama za upravljanje robotom
pub = rospy.Publisher('mod_robota', String, queue_size=1)
rospy.init_node('izbor_moda_kretanja_robota', anonymous=False)

# Callback funkcija za servis 
# prima informaciju koji je mod izabran i publish-uje ga na /mod_robota topic
def response_callback(req):
    global  pub

    mod = req.mod
    
    back_info = ''

    if mod == 'man':
        back_info = 'Izabrali ste manuelni mod kretanja robota!'
    elif mod == 'auto':
        back_info = 'Izabrali ste automatski mod kretanja robota!'
    else:
        back_info = 'Pokusajte ponovo!'

    rospy.loginfo(back_info)
    pub.publish(mod)

    # Povratna vrednost servisa
    return izbor_modaResponse(True)

# Inicijalizacija ROS servisa za izbor moda upravljanja robota
s = rospy.Service('izbor_moda', izbor_moda, response_callback)
rospy.loginfo("Servis za izbor moda kretanja robota je spreman!")
rospy.spin()