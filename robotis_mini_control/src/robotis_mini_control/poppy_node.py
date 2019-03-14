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
    N = int(math.floor(tf/dt))
    for i in range(N):
        q.append(a0 + a1*t + a2*t*t + a3*t*t*t)
        t = t +dt
    return q


class Driver:

    def __init__(self):

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
        freq = 500.0
        rate = rospy.Rate(freq)


        # Set motor parameters
        self.dxl_io.disable_torque(self.ids)
        self.dxl_io.set_torque_limit(dict(zip(self.ids, itertools.repeat(100))))
        self.dxl_io.set_max_torque(dict(zip(self.ids, itertools.repeat(100))))

        self.dxl_io.set_pid_gain(dict(zip(self.ids, itertools.repeat((4, 0, 0)))))

        self.dxl_io.set_moving_speed(dict(zip(self.ids, itertools.repeat(0))))

        self.dxl_io.enable_torque(self.ids)
        # Valis motor parameters
        print("Validate messages")

        while True:
            try:
                msg = self.dxl_io.get_torque_limit(self.ids)
            except pypot.dynamixel.io.abstract_io.DxlTimeoutError:
                pass
            else:
                break
        print("Torque limit:")
        for m in msg:
            print('\t' + str(m))

        msg = self.dxl_io.get_max_torque(self.ids)
        print("Max torque:")
        for m in msg:
            print('\t' + str(m))

        while True:
            msg = self.dxl_io.is_torque_enabled(self.ids)
            print("Troque enabled:")
            flag = False
            for m in msg:
                print('\t' + str(m))
                if not m:
                    flag =  True
            if flag:
                self.dxl_io.enable_torque(self.ids)
            else:
                break

        msg = self.dxl_io.get_control_mode(self.ids)
        print("Control mode:")
        for m in msg:
            print('\t' + str(m))


        msg = self.dxl_io.get_pid_gain(self.ids)
        print("PID:")
        for m in msg:
            print('\t' + str(m))

        msg = self.dxl_io.get_moving_speed(self.ids)
        print("Moving speed:")
        for m in msg:
            print('\t' + str(m))


        qi_1 = self.dxl_io.get_present_position([1])[0]
        qi_2 = self.dxl_io.get_present_position([2])[0]
        qi_3 = self.dxl_io.get_present_position([3])[0]
        qi_4 = self.dxl_io.get_present_position([4])[0]
        qi_5 = self.dxl_io.get_present_position([5])[0]
        qi_6 = self.dxl_io.get_present_position([6])[0]
        qi_7 = self.dxl_io.get_present_position([7])[0]
        qi_8 = self.dxl_io.get_present_position([8])[0]
        qi_9 = self.dxl_io.get_present_position([9])[0]
        qi_10 = self.dxl_io.get_present_position([10])[0]
        qi_11 = self.dxl_io.get_present_position([11])[0]
        qi_12 = self.dxl_io.get_present_position([12])[0]
        qi_13 = self.dxl_io.get_present_position([13])[0]
        qi_14 = self.dxl_io.get_present_position([14])[0]
        qi_15 = self.dxl_io.get_present_position([15])[0]
        qi_16 = self.dxl_io.get_present_position([16])[0]

        qf = 0

        tf = 5

        q1_list = cubiq_interpol(qi_1, qf, tf, 1.0/freq)
        q2_list = cubiq_interpol(qi_2, qf, tf, 1.0/freq)
        q3_list = cubiq_interpol(qi_3, qf, tf, 1.0/freq)
        q4_list = cubiq_interpol(qi_4, qf, tf, 1.0/freq)
        q5_list = cubiq_interpol(qi_5, qf, tf, 1.0/freq)
        q6_list = cubiq_interpol(qi_6, qf, tf, 1.0/freq)
        q7_list = cubiq_interpol(qi_7, qf, tf, 1.0/freq)
        q8_list = cubiq_interpol(qi_8, qf, tf, 1.0/freq)
        q9_list = cubiq_interpol(qi_9, qf, tf, 1.0/freq)
        q10_list = cubiq_interpol(qi_10, qf, tf, 1.0/freq)
        q11_list = cubiq_interpol(qi_11, qf, tf, 1.0/freq)
        q12_list = cubiq_interpol(qi_12, qf, tf, 1.0/freq)
        q13_list = cubiq_interpol(qi_13, qf, tf, 1.0/freq)
        q14_list = cubiq_interpol(qi_14, qf, tf, 1.0/freq)
        q15_list = cubiq_interpol(qi_15, qf, tf, 1.0/freq)
        q16_list = cubiq_interpol(qi_16, qf, tf, 1.0/freq)
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

            self.dxl_io.set_goal_position({1:q1, 2:q2, 3:q3, 4:q4, 5:q5, 6:q6, 7:q7, 8:q8, 9:q9, 10:q10, 11:q11, 12:q12, 13:q13, 14:q14, 15:q15, 16:q16})
            rate.sleep()

        # Be sure that
        for i in range(50):
            self.dxl_io.set_goal_position({1:q1, 2:q2, 3:q3, 4:q4, 5:q5, 6:q6, 7:q7, 8:q8, 9:q9, 10:q10, 11:q11, 12:q12, 13:q13, 14:q14, 15:q15, 16:q16})
            rate.sleep()


        rospy.sleep(5)

        # Inital movement
        hipOffsetY = .024;  #OP, measured
        hipOffsetZ = .027;  #OP, Calculated from spec
        hipOffsetX = .015;  #OP, Calculated from spec
        thighLength = .045; #OP, spec
        tibiaLength = .042; #OP, spec
        footHeight = .031; #OP, spec
        kneeOffsetX = 0.03; #This parameter can be modified

        dThigh = thighLength
        aThigh = 0.3*math.atan(kneeOffsetX/thighLength)
        dTibia = tibiaLength
        aTibia = math.atan(kneeOffsetX/tibiaLength)
        a_hip_i = 0.45;
        a_foot_i = 0.45;


        qi_1 = self.dxl_io.get_present_position([1])[0]
        qi_2 = self.dxl_io.get_present_position([2])[0]
        qi_3 = self.dxl_io.get_present_position([3])[0]
        qi_4 = self.dxl_io.get_present_position([4])[0]
        qi_5 = self.dxl_io.get_present_position([5])[0]
        qi_6 = self.dxl_io.get_present_position([6])[0]
        qi_7 = self.dxl_io.get_present_position([7])[0]
        qi_8 = self.dxl_io.get_present_position([8])[0]
        qi_9 = self.dxl_io.get_present_position([9])[0]
        qi_10 = self.dxl_io.get_present_position([10])[0]
        qi_11 = self.dxl_io.get_present_position([11])[0]
        qi_12 = self.dxl_io.get_present_position([12])[0]
        qi_13 = self.dxl_io.get_present_position([13])[0]
        qi_14 = self.dxl_io.get_present_position([14])[0]
        qi_15 = self.dxl_io.get_present_position([15])[0]
        qi_16 = self.dxl_io.get_present_position([16])[0]


        tf = 5

        q3_list = cubiq_interpol(qi_3,   math.degrees(1.1), tf, 1.0/freq)
        #q4_list = cubiq_interpol(qi_4,   math.degrees(1.1), tf, 1.0/freq)
        q5_list = cubiq_interpol(qi_5,   math.degrees(0.8), tf, 1.0/freq)
        q7_list = cubiq_interpol(qi_7,   math.degrees(-a_hip_i), tf, 1.0/freq)
        q8_list = cubiq_interpol(qi_8,   math.degrees(-a_hip_i), tf, 1.0/freq)
        q9_list = cubiq_interpol(qi_9,   math.degrees(-aThigh), tf, 1.0/freq)
        q10_list = cubiq_interpol(qi_10, math.degrees(aThigh), tf, 1.0/freq)
        q11_list = cubiq_interpol(qi_11, math.degrees((aThigh+aTibia)), tf, 1.0/freq)
        q12_list = cubiq_interpol(qi_12, math.degrees(-(aThigh+aTibia)), tf, 1.0/freq)
        q13_list = cubiq_interpol(qi_13, math.degrees(aTibia), tf, 1.0/freq)
        q14_list = cubiq_interpol(qi_14, math.degrees(-aTibia), tf, 1.0/freq)
        q15_list = cubiq_interpol(qi_15, math.degrees(-a_foot_i), tf, 1.0/freq)
        q16_list = cubiq_interpol(qi_16, math.degrees(-a_foot_i), tf, 1.0/freq)

        while q7_list:
            q3 = q3_list.pop(0)
            #q4 = q4_list.pop(0)
            q5 = q5_list.pop(0)
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
            self.dxl_io.set_goal_position({3:q3, 5:q5, 7:q7, 8:q8, 9:q9, 10:q10, 11:q11, 12:q12, 13:q13, 14:q14, 15:q15, 16:q16})
            rate.sleep()

        for i in range(100):
            self.dxl_io.set_goal_position({1:q1, 2:q2, 3:q3, 4:q4, 5:q5, 6:q6, 7:q7, 8:q8, 9:q9, 10:q10, 11:q11, 12:q12, 13:q13, 14:q14, 15:q15, 16:q16})
            rate.sleep()

        rospy.sleep(5)


        # Quasi-static step
        q7_l = []
        q9_l = []
        q11_l = []
        q13_l = []
        q15_l = []

        ctr = 0
        with open("/home/biobot/.projects/ws/ros/catkin_ws/src/robotis_mini/robotis_mini_control/src/robotis_mini_control/data.txt") as tsv:
            for line in csv.reader(tsv, delimiter='\t'):
                if ctr == 0:
                    ctr += 1
                    N = float(line[0])
                    print("N = %", N)
                elif ctr ==  1:
                    ctr += 1
                    dt = 1.0/float(line[0])
                    print("dt = %", dt)
                    ros_dt = rospy.Rate(dt)
                else:
                    q7_l.append( math.degrees(float(line[0])))
                    q9_l.append( math.degrees(float(line[1])))
                    q11_l.append(math.degrees(float(line[2])))
                    q13_l.append(math.degrees(float(line[3])))
                    q15_l.append(math.degrees(float(line[4])))
        q8 = self.dxl_io.get_present_position([8])[0]
        q10 = self.dxl_io.get_present_position([10])[0]
        q12 = self.dxl_io.get_present_position([12])[0]
        q14 = self.dxl_io.get_present_position([14])[0]
        q16 = self.dxl_io.get_present_position([16])[0]

        print(N)
        while q7_l:
            q7 = q7_l.pop(0)
            q9 = q9_l.pop(0)
            q11 = q11_l.pop(0)
            q13 = q13_l.pop(0)
            q15 = q15_l.pop(0)

            #self.dxl_io.set_goal_position({7:q7, 8:q8, 9:q9, 10:q10, 11:q11, 12:q12, 13:q13, 14:q14, 15:q15, 16:q16})
            #self.dxl_io.set_goal_position({7:q7, 8:q8[0], 9:q9, 11:q11, 13:q13, 15:q15})
            self.dxl_io.set_goal_position({7:q7, 9:q9, 11:q11, 13:q13, 15:q15})
            ros_dt.sleep()

        # Be sure that
        for i in range(50):
            self.dxl_io.set_goal_position({7:q7, 9:q9, 11:q11, 13:q13, 15:q15})
            rate.sleep()







if __name__ == '__main__':
    try:
        Driver()
    except rospy.ROSInterruptException:
        pass
