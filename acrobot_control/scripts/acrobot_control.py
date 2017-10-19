#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan,atan2,sqrt,fabs
from numpy import *
from sensor_msgs.msg import JointState

def error_state(qd, q):
    dq = qd%(2*pi) - q%(2*pi)
    if dq > pi:
        dq = dq - 2*pi
    elif dq <= -pi:
        dq = dq + 2*pi
    return dq

class Acrobot:
    _m = (1,2)
    _l = (1,2)
    _lc = (0.5,1)
    _Ic = (0.083,0.33)
    _g = 9.81
    _b = (.1, .1)

    def __init__(self, m, l, lc, Ic, b):
        self._m = m
        self._l = l
        self._lc = lc
        self._Ic = Ic
        self._b = b
    
    def manipulator_dynamics(self, q, qd):
         # keep it readable:
        (m1,m2)=self._m
        (l1,l2)=self._l
        g=self._g
        (lc1,lc2)=self._lc
        (b1,b2)=self._b
        I1 = self._Ic[0] + m1*lc1**2
        I2 = self._Ic[1] + m2*lc2**2
        m2l1lc2 = m2*l1*lc2  # occurs often!

        c = cos(q[0:2,:])
        s = sin(q[0:2,:])

        s12 = sin(q.item(0)+q.item(1))

        h12 = I2 + m2l1lc2*c.item(1)
        H = array([[ I1 + I2 + m2*l1**2 + 2*m2l1lc2*c.item(1), h12],
                [h12, I2] ])
                
        C_ = array([[-2*m2l1lc2*s.item(1)*qd.item(1), -m2l1lc2*s.item(1)*qd.item(1)],
                [m2l1lc2*s.item(1)*qd.item(0), 0] ])
        
        G = g*array([[ m1*lc1*s.item(0) + m2*(l1*s.item(1)+lc2*s12)],

            [m2*lc2*s12] ])
            
        # accumate total C and add a damping term:
        C = C_.dot(qd) + G + array([[b1],[b2]])*qd
        B = array([[0], [1]])

        return (H, C, B)

def listen_joint_state(joint_state, (q, qd)):
    q[0] = joint_state.position[0]
    q[1] = joint_state.position[1]
    qd[0] = joint_state.velocity[0]
    qd[1] = joint_state.velocity[1]
    # rospy.loginfo(rospy.get_caller_id() + ', q=(%f, %f)', q[0], q[1]) #, q[1])

# acrobot pfl, lqr controller
def acrobot_control_publisher():
    acrobot = Acrobot((0.171, 0.289), (0.37779, 0.3882), (0.27948, 0.32843), (0.0027273, 0.0033484), (0.01, 0.01))

    q = deg2rad(array([[45],[0]]))
    q_dot = deg2rad(array([[0],[0]]))

    des_q2_ddot = 0
    des_q2_dot = 0
    des_q2 = 0
    
    alpha = 2
    kp = 100.0
    kd = 50

    #Initiate node for controlling joint1 and joint2 positions.
    rospy.init_node('acrobot_control_node', anonymous=True)

    #Define publishers for each joint position controller commands.
    rospy.Subscriber('/acrobot/joint_states', JointState, listen_joint_state, (q, q_dot) )
    pub2 = rospy.Publisher('/acrobot/joint2_torque_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(100) #100 Hz

    # While loop to have joints follow a certain position, while rospy is not shutdown.
    i = 0
    t = 0
    controller = 'swing up'
    while not rospy.is_shutdown():

        if abs(q[0]) < 0.5 and abs(q[1]) < 0.5 : # have to implement region of attraction later
            controller = 'lqr'

        if controller == 'swing up':
            # Acrobot collocated control
            (H, C, B) = acrobot.manipulator_dynamics(q, q_dot)

            H22_bar = H[1,1] - H[1,0]/H[0,0]*H[0,1]
            C2_bar = C[1] - H[1,0]/H[0,0]*C[0]
            
            des_q2 = 2*alpha/pi*atan(q_dot[0])     

            v2 = des_q2_ddot + kd*(des_q2_dot - q_dot[1]) + kp*error_state(des_q2, q[1])
            
            u = H22_bar*v2 + C2_bar
        else:
            K = array([-650.4009, -289.0746, -287.1833, -140.0615]) # update automatically calculate K
            u = -K.dot(array([[q.item(0)], [q_dot.item(0)], [q.item(1)], [q_dot.item(1)]]))
            rospy.loginfo(rospy.get_caller_id() + 'lqr')

        if (t == 100):
            rospy.loginfo(rospy.get_caller_id() + 'q1_dot = %f, des_q2 = %f, q2 = %f, u = %f', q_dot[0], des_q2, q[1], u)
            t=0
        t = t + 1

        pub2.publish(u)


        i = i+1 #increment i

        rate.sleep() #sleep for rest of rospy.Rate(100)

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    try: acrobot_control_publisher()
    except rospy.ROSInterruptException: pass
