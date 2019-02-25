#!/usr/bin/env python


#~ THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#~ WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#~ MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#~ ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#~ WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
#~ OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
#~ CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
import pypot.dynamixel
import math
import threading
import csv
import itertools

def cubiq_interpol(qi, qf, tf, rate):
    q = []
    a0 = qi
    a1 = 0
    a2 = (3*(qf-a0)) / (tf*tf)
    a3 = (-2*(qf-a0)) / (tf*tf*tf)

    dt = rate
    t = dt
    N = int(math.ceil(tf/dt) -1)
    for i in range(N):
        q.append(a0 + a1*t + a2*t*t + a3*t*t*t)
        t = t +dt
    return q

def joint_to_id(joint):
    if (joint == 'r_shoulder_joint'):
        return 1
    elif (joint == 'l_shoulder_joint'):
        return 2
    elif (joint == 'r_biceps_joint'):
        return 3
    elif (joint == 'l_biceps_joint'):
        return 4
    elif (joint == 'r_elbow_joint'):
        return 5
    elif (joint == 'l_elbow_joint'):
        return 6
    elif (joint == 'r_hip_joint'):
        return 7
    elif (joint == 'l_hip_joint'):
        return 8
    elif (joint == 'r_thigh_joint'):
        return 9
    elif (joint == 'l_thigh_joint'):
        return 10
    elif (joint == 'r_knee_joint'):
        return 11
    elif (joint == 'l_knee_joint'):
        return 12
    elif (joint == 'r_ankle_joint'):
        return 13
    elif (joint == 'l_ankle_joint'):
        return 14
    elif (joint == 'r_foot_joint'):
        return 15
    elif (joint == 'l_foot_joint'):
        return 16
    else:
        rospy.logerr('Joint name %d not valid', joint)

def ids_to_str(ids):
    id_str = []
    for id in ids:
        if (id == 1):
            id_str.append('r_shoulder_joint')
        elif (id == 2):
            id_str.append('l_shoulder_joint')
        elif (id == 3):
            id_str.append('r_biceps_joint')
        elif (id == 4):
            id_str.append('l_biceps_joint')
        elif (id == 5):
            id_str.append('r_elbow_joint')
        elif (id == 6):
            id_str.append('l_elbow_joint')
        elif (id == 7):
            id_str.append('r_hip_joint')
        elif (id == 8):
            id_str.append('l_hip_joint')
        elif (id == 9):
            id_str.append('r_thigh_joint')
        elif (id == 10):
            id_str.append('l_thigh_joint')
        elif (id == 11):
            id_str.append('r_knee_joint')
        elif (id == 12):
            id_str.append('l_knee_joint')
        elif (id == 13):
            id_str.append('r_ankle_joint')
        elif (id == 14):
            id_str.append('l_ankle_joint')
        elif (id == 15):
            id_str.append('r_foot_joint')
        elif (id == 16):
            id_str.append('l_foot_joint')
        else:
            rospy.logerr('ID %d not valid', id)
    return id_str

class Driver:

    def __init__(self):
        self.lock = threading.Lock()
        self.write_id = []
        self.write_pos = []

        #self.joint-name = ['r_shoulder_joint',
        #                   'l_shoulder_joint',
        #                   'r_biceps_joint',
        #                   'l_biceps_joint',
        #                   'r_elbow_joint',
        #                   'l_elbow_joint',
        #                   'r_hip_joint',
        #                   'l_hip_joint',
        #                   'r_thigh_joint',
        #                   'l_thigh_joint',
        #                   'r_knee_joint',
        #                   'l_knee_joint',
        #                   'r_ankle_joint',
        #                   'l_ankle_joint',
        #                   'r_foot_joint',
        #                   'l_foot_joint']
        # Connect to dynamixel motors
        ports = pypot.dynamixel.get_available_ports()
        print('available ports:', ports)

        if not ports:
            raise IOError('No port available.')

        port = ports[0]
        print('Using the first on the list', port)

        self.dxl_io = pypot.dynamixel.Dxl320IO(port, baudrate=1000000)
        print('Connected!')

        self.ids = self.dxl_io.scan()
        self.ids = self.ids[:-1]
        print('Found ids:', self.ids)


        # Ros init
        rospy.init_node('poppy_node')
        rate = rospy.Rate(50)

        pub = rospy.Publisher('/robotis_mini/joint_states', JointState, queue_size=10)
        sub = rospy.Subscriber('/robotis_mini/goal', JointState, self.write)

        torque_enabled = self.dxl_io.is_torque_enabled(self.ids)
        for i in torque_enabled:
            print(i)

        print("-----------")
        # Set motor specifications
        self.dxl_io.set_max_torque(dict(zip(self.ids, itertools.repeat(1023))))
        self.dxl_io.set_torque_limit(dict(zip(self.ids, itertools.repeat(1023))))
        self.dxl_io.enable_torque(self.ids)

        torque_enabled = self.dxl_io.is_torque_enabled(self.ids)
        for i in torque_enabled:
            print(i)

        print("-----------")

        ctrl_mode = self.dxl_io.get_control_mode(self.ids)
        for i in ctrl_mode:
            print(i)

        print("-----------")
        print(dir(self.dxl_io))


        self.dxl_io.set_pid_gain(dict(zip(self.ids, itertools.repeat((4, 0, 0.2)))))

        slope = self.dxl_io.get_pid_gain(self.ids)
        for i in slope:
            print(i)

        print("-----------")









        msg = JointState()
        msg.name = ids_to_str(self.ids)
        msg.velocity = []
        msg.effort = []


        qi_1 = self.dxl_io.get_present_position([1])
        qi_2 = self.dxl_io.get_present_position([2])
        qi_3 = self.dxl_io.get_present_position([3])
        qi_4 = self.dxl_io.get_present_position([4])
        qi_5 = self.dxl_io.get_present_position([5])
        qi_6 = self.dxl_io.get_present_position([6])
        qi_7 = self.dxl_io.get_present_position([7])
        qi_8 = self.dxl_io.get_present_position([8])
        qi_9 = self.dxl_io.get_present_position([9])
        qi_10 = self.dxl_io.get_present_position([10])
        qi_11 = self.dxl_io.get_present_position([11])
        qi_12 = self.dxl_io.get_present_position([12])
        qi_13 = self.dxl_io.get_present_position([13])
        qi_14 = self.dxl_io.get_present_position([14])
        qi_15 = self.dxl_io.get_present_position([15])
        qi_16 = self.dxl_io.get_present_position([16])

        qi_1 = qi_1[0]
        qi_2 = qi_2[0]
        qi_3 = qi_3[0]
        qi_4 = qi_4[0]
        qi_5 = qi_5[0]
        qi_6 = qi_6[0]
        qi_7 = qi_7[0]
        qi_8 = qi_8[0]
        qi_9 = qi_9[0]
        qi_10 = qi_10[0]
        qi_11 = qi_11[0]
        qi_12 = qi_12[0]
        qi_13 = qi_13[0]
        qi_14 = qi_14[0]
        qi_15 = qi_15[0]
        qi_16 = qi_16[0]

        qf = 0

        tf = 5


        q1_list = cubiq_interpol(qi_1, qf, tf, 1.0/50.0)
        q2_list = cubiq_interpol(qi_2, qf, tf, 1.0/50.0)
        q3_list = cubiq_interpol(qi_3, qf, tf, 1.0/50.0)
        q4_list = cubiq_interpol(qi_4, qf, tf, 1.0/50.0)
        q5_list = cubiq_interpol(qi_5, qf, tf, 1.0/50.0)
        q6_list = cubiq_interpol(qi_6, qf, tf, 1.0/50.0)
        q7_list = cubiq_interpol(qi_7, qf, tf, 1.0/50.0)
        q8_list = cubiq_interpol(qi_8, qf, tf, 1.0/50.0)
        q9_list = cubiq_interpol(qi_9, qf, tf, 1.0/50.0)
        q10_list = cubiq_interpol(qi_10, qf, tf, 1.0/50.0)
        q11_list = cubiq_interpol(qi_11, qf, tf, 1.0/50.0)
        q12_list = cubiq_interpol(qi_12, qf, tf, 1.0/50.0)
        q13_list = cubiq_interpol(qi_13, qf, tf, 1.0/50.0)
        q14_list = cubiq_interpol(qi_14, qf, tf, 1.0/50.0)
        q15_list = cubiq_interpol(qi_15, qf, tf, 1.0/50.0)
        q16_list = cubiq_interpol(qi_16, qf, tf, 1.0/50.0)

        while q1_list:
            q1 = q1_list.pop(0)
            q2 = q2_list.pop(0)
            q3 = q3_list.pop(0)
            q4 = q4_list.pop(0)
            q5 = q5_list.pop(0)
            q6 = q6_list.pop(0)
            q7 = q7_list.pop(0)
            q8 = q8_list.pop(0)
            q9 = q9_list.pop(0)
            q10 = q10_list.pop(0)
            q11 = q11_list.pop(0)
            q12 = q12_list.pop(0)
            q13 = q13_list.pop(0)
            q14 = q14_list.pop(0)
            q15 = q15_list.pop(0)
            q16 = q16_list.pop(0)

            #print("q1 = ", q1)
            #print("q2 = ", q2)
            #print("q3 = ", q3)
            self.dxl_io.set_goal_position({1:q1, 2:q2, 3:q3, 4:q4, 5:q5, 6:q6, 7:q7, 8:q8, 9:q9, 10:q10, 11:q11, 12:q12, 13:q13, 14:q14, 15:q15, 16:q16})
            rate.sleep()

        # Be sure that
        self.dxl_io.set_goal_position({1:q1, 2:q2, 3:q3, 4:q4, 5:q5, 6:q6, 7:q7, 8:q8, 9:q9, 10:q10, 11:q11, 12:q12, 13:q13, 14:q14, 15:q15, 16:q16})
        rate.sleep()
        self.dxl_io.set_goal_position({1:q1, 2:q2, 3:q3, 4:q4, 5:q5, 6:q6, 7:q7, 8:q8, 9:q9, 10:q10, 11:q11, 12:q12, 13:q13, 14:q14, 15:q15, 16:q16})
        rate.sleep()



        # Inital movement
        hipOffsetY = .024;  #OP, measured
        hipOffsetZ = .027;  #OP, Calculated from spec
        hipOffsetX = .015;  #OP, Calculated from spec
        thighLength = .045; #OP, spec
        tibiaLength = .042; #OP, spec
        footHeight = .0246; #OP, spec
        kneeOffsetX = 0.04; #This parameter can be modified

        dThigh = thighLength
        aThigh = 0;
        dTibia = tibiaLength
        aTibia = math.atan(kneeOffsetX/tibiaLength)
        a_hip_i = 0.45;
        a_foot_i = 0.45;


        qi_1 = q1
        qi_2 = q2
        qi_3 = q3
        qi_4 = q4
        qi_5 = q5
        qi_6 = q6
        qi_7 = q7
        qi_8 = q8
        qi_9 = q9
        qi_10 = q10
        qi_11 = q11
        qi_12 = q12
        qi_13 = q13
        qi_14 = q14
        qi_15 = q15
        qi_16 = q16


        tf = 10


        q7_list = cubiq_interpol(qi_7,   math.degrees(-a_hip_i), tf, 1.0/50.0)
        q8_list = cubiq_interpol(qi_8,   math.degrees(-a_hip_i), tf, 1.0/50.0)
        q9_list = cubiq_interpol(qi_9,   math.degrees(aThigh), tf, 1.0/50.0)
        q10_list = cubiq_interpol(qi_10, math.degrees(-aThigh), tf, 1.0/50.0)
        q11_list = cubiq_interpol(qi_11, math.degrees((aThigh+aTibia)), tf, 1.0/50.0)
        q12_list = cubiq_interpol(qi_12, math.degrees(-(aThigh+aTibia)), tf, 1.0/50.0)
        q13_list = cubiq_interpol(qi_13, math.degrees(aTibia), tf, 1.0/50.0)
        q14_list = cubiq_interpol(qi_14, math.degrees(-aTibia), tf, 1.0/50.0)
        q15_list = cubiq_interpol(qi_15, math.degrees(-a_foot_i), tf, 1.0/50.0)
        q16_list = cubiq_interpol(qi_16, math.degrees(-a_foot_i), tf, 1.0/50.0)

        while q7_list:
            q7 = q7_list.pop(0)
            q8 = q8_list.pop(0)
            q9 = q9_list.pop(0)
            q10 = q10_list.pop(0)
            q11 = q11_list.pop(0)
            q12 = q12_list.pop(0)
            q13 = q13_list.pop(0)
            q14 = q14_list.pop(0)
            q15 = q15_list.pop(0)
            q16 = q16_list.pop(0)

            #print("q1 = ", q1)
            #print("q2 = ", q2)
            #print("q3 = ", q3)
            self.dxl_io.set_goal_position({7:q7, 8:q8, 9:q9, 10:q10, 11:q11, 12:q12, 13:q13, 14:q14, 15:q15, 16:q16})
            rate.sleep()

        # Be sure that
        self.dxl_io.set_goal_position({7:q7, 8:q8, 9:q9, 10:q10, 11:q11, 12:q12, 13:q13, 14:q14, 15:q15, 16:q16})
        rate.sleep()
        self.dxl_io.set_goal_position({7:q7, 8:q8, 9:q9, 10:q10, 11:q11, 12:q12, 13:q13, 14:q14, 15:q15, 16:q16})
        rate.sleep()


        # With jacobian
        self.dxl_io.set_max_torque(dict(zip([8, 10, 12, 14, 16], itertools.repeat(1023))))
        self.dxl_io.set_torque_limit(dict(zip([8, 10, 12, 14, 16], itertools.repeat(1023))))
        self.dxl_io.enable_torque([8, 10, 12, 14, 16])
        q7_l = []
        q9_l = []
        q11_l = []
        q13_l = []
        q15_l = []

        ctr = 0
        with open("data.txt") as tsv:
            for line in csv.reader(tsv, delimiter='\t'):
                if ctr == 0:
                    ctr += 1
                    N = float(line[0])
                elif ctr ==  1:
                    ctr += 1
                else:
                    q7_l.append( math.degrees(float(line[1])))
                    q9_l.append( math.degrees(float(line[2])))
                    q11_l.append(math.degrees(float(line[3])))
                    q13_l.append(math.degrees(float(line[4])))
                    q15_l.append(math.degrees(float(line[5])))


        print(N)
        while q7_l:
            q7 = q7_l.pop(0)
            q9 = q9_l.pop(0)
            q11 = q11_l.pop(0)
            q13 = q13_l.pop(0)
            q15 = q15_l.pop(0)

            #print("q1 = ", q1)
            #print("q2 = ", q2)
            #print("q3 = ", q3)
            self.dxl_io.set_goal_position({7:q7, 9:q9, 11:q11, 13:q13, 15:q15})
            rate.sleep()

        # Be sure that
        self.dxl_io.set_goal_position({7:q7, 9:q9, 11:q11, 13:q13, 15:q15})
        rate.sleep()
        self.dxl_io.set_goal_position({7:q7, 9:q9, 11:q11, 13:q13, 15:q15})
        rate.sleep()




        exit()

        # Loop
        while not rospy.is_shutdown():
            # Update jointState
            msg.position = []

            flag = True
            while flag:
                try:
                    pos = self.dxl_io.get_present_position(self.ids)
                    for p in pos:
                        msg.position.append(math.radians(p))
                    msg.header.stamp = rospy.Time.now()
                    pub.publish(msg)
                    flag = False
                except pypot.dynamixel.io.abstract_io.DxlCommunicationError:
                    print("Error occured")
                    pass
            flag = True
            while flag:
                try:

                    self.lock.acquire()
                    if self.write_id:
                        write_id = self.write_id.pop(0)
                        write_pos = self.write_pos.pop(0)
                        self.lock.release()
                        self.dxl_io.set_goal_position(dict(zip(write_id, write_pos)), sync_write=True)
                        flag = False
                    else:
                        self.lock.release()
                except pypot.dynamixel.io.abstract_io.DxlCommunicationError:
                    pass
                finally:
                    flag = False


            rate.sleep()


    def write(self, data):
        pos = []
        ids = []
        if (len(data.position) == 16):
            #print("Name = ", data.name[15])
            for p in data.position:
                print("p = ", p)
                pos.append(math.degrees(p))
            for name in data.name:
                print("name = ", name)
                print("id = ", joint_to_id(name))
                ids.append(joint_to_id(name))

            self.lock.acquire()
            self.write_id.append(ids)
            self.write_pos.append(pos)
            self.lock.release()

            #self.lock.acquire()
            #try:
            #self.dxl_io.set_goal_position(dict(zip(ids, pos)))
            #finally:
            #    self.lock.release()

if __name__ == '__main__':
    try:
        Driver()
    except rospy.ROSInterruptException:
        pass
