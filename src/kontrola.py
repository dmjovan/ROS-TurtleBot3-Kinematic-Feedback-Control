#!/usr/bin/env python3

# Importovanje potrebnih biblioteka
import rospy
import math
from math import pi

# Importovanje potrebnih struktura poruka
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Inicijalizacija globalnog publisher-a na topic za upravljanje brzinama
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('Kontrola_kretanja', anonymous=False)

# Objekat koji ima strukturu koja se publish-uje na topic /cmd_vel
# upravljanje vrzinama
vel = Twist()

# Lista sa linearnom i ugaonom brzinom robota koja se zadaje
brzine = [0, 0]

# Pomeraji zadati u manuelnom modu
pomeraji = {'w':[0.1, 0], 's': [-0.1, 0], 'a': [0, 0.2], 'd': [0, -0.2]}

# Izmerene pozicije robota
xm, ym = 0, 0

# Ciljne koordinate za automatsko kretanje robota
cilj_x, cilj_y = math.inf, math.inf

# Vrednosti medjurezultata pri racunanju automatskog upravljanja robota
dx, dy = 0, 0 
rho = 0

# Izmerena trenutna orijentacija robota
theta = 0

# Inicijalni mod
mod = 'man'

# Parametri kontrolera (ka se racuna u toku kretanja)
kr = 0.15
kb = -0.05

# Konstantni odnos v i w
vw = 1.5

# Funkcija za pretvaranje kvaterniona u Ojlerove uglove
def quat2euler(x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z

# Callback funkcija za subscriber na topic /mod_robota kojim se definise trenutni mod robota
def mod_callback(data):

    global mod, posl_mod, brzine, cilj_x, cilj_y

    if data.data == 'man' or data.data == 'auto':

        brzine = [0, 0]
        publish_velocities(brzine)

        cilj_x = math.inf
        cilj_y = math.inf

        mod = data.data

# Funkcija za publish-ovanje brzina na topic /cmd_vel 
def publish_velocities(brzine):

    global mod, vel

    rospy.loginfo("Linearna brzina: %.2f, Ugaona brzina: %.2f (mod: %s)" %(brzine[0], brzine[1],mod))
    vel.linear.x = brzine[0]
    vel.angular.z = brzine[1]
    pub.publish(vel)

# Funkcija za odredjivanje smera ugaone brzine prilikom automatskog kretanja
def w_sign(robot, cilj):

    cilj = cilj if cilj>=0 else 2*pi+cilj
    robot = robot if robot>=0 else 2*pi+robot

    if(robot < pi):
        if(cilj - robot > pi):
            w = -1
        else:
            w = 1

        if(cilj < robot):
            w = -1
    else:
        if(robot-cilj > pi):
            w = 1
        else:
            w = -1

        if(cilj > robot):
            w = 1

    rospy.loginfo(str(w))
    return w

# Funkcija za automatsko kretanje robota sa zadatim ciljnim koordinatama
# iterativno se poziva u callback funkciji za odometrijska merenja
def automatsko_kretanje():

    global brzine, mod, rho, theta, kr, kb, vw, dx, dy

    if rho > 0:

        val = - theta + math.atan2(dy, dx)

        if(min(abs(val), 2*pi-abs(val))>pi/2):
            reverse = True
        else:
            reverse = False
        
        alpha = val%(pi/2)
        beta = (- theta - alpha)%pi

        ka = max(2/pi*kr - 5/3*kb, (kr*rho/vw-kb*beta)/alpha)

        brzine[0] = kr*rho
        brzine[1] = (ka*alpha + kb*beta)*w_sign(theta, math.atan2(dy, dx))

        if(reverse):
            brzine[0] = -brzine[0]
            brzine[1] = -brzine[1]
            
        rospy.loginfo("alpha = {0}".format(alpha))
        publish_velocities(brzine)

# Callback funkcija za subsriber na topic /komande 
# ukoliko je robot u manuelnom rezimu, onde se kao komande primaju stringovi 'w', 'a', 's', 'd'
# ukoliko je robot u automatskom rezimu, onda se kao komande primaju ciljne koordinate kao string 'x.xx y.yy'
def kom_callback(data):

    global pomeraji, brzine, cilj_x, cilj_y, posl_mod

    komanda = data.data

    if mod == 'man':
            
        if komanda in set(pomeraji.keys()):
            pomeraj = pomeraji[komanda]
        else:
            pomeraj = [0, 0]

        brzine[0] += pomeraj[0]
        brzine[1] += pomeraj[1]

        publish_velocities(brzine)

    elif mod == 'auto':

        cilj_x = float(komanda.split(' ')[0])
        cilj_y = float(komanda.split(' ')[1])

        rospy.loginfo("Koordinate cilja: %.2f, %.2f" %(cilj_x, cilj_y))

# Callback funkcija ugradjeni /odom topic 
# sluzi za ekstrakciju trenutnih koordinata robota i za upravljanje robotom na 
# osnovu tih ekstraktovanih koordinata
def odom_callback(data): 

    global theta, rho, cilj_x, cilj_y, dx, dy, mod

    position = data.pose.pose.position

    xm = position.x
    ym = position.y

    orientation = data.pose.pose.orientation

    _, _, theta = quat2euler(orientation.x, orientation.y, orientation.z, orientation.w)

    if ((mod == 'auto') and (cilj_x != math.inf) and (cilj_y != math.inf)):
        dx = cilj_x - xm
        dy = cilj_y - ym

        rho = math.sqrt(dx**2 + dy**2)

        automatsko_kretanje()

# Funkcija za subscribe-ovanje na sve potrebne topic-e 
def listener():
    rospy.Subscriber('mod_robota', String, mod_callback)
    rospy.Subscriber('komande', String, kom_callback)
    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.spin()

# main program za pokretanje svih gore navedenih funkcionalnosti
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass