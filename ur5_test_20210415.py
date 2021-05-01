from math import pi
import time
import sys
import csv
from typing import Union

import urx  # ur5 package
import logging
import numpy as np
import sympy as sp
from urx import URRobot, Robot

if sys.version_info[0] < 3:  # support python v2
    input = 2

def __init__(self):
    
def wait():
    if do_wait:
        print("Click enter to continue")
        input()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    do_wait = True
    if len(sys.argv) > 1:
        do_wait = False

    rob: Union[Robot, URRobot] = urx.Robot("IP Address")  # Connect Setting
    # rob = urx.Robot(“local host”)
    rob.set_tcp((0, 0, 0.289, 0, 0, 0))  # Setting the robot flange to the tool tip deformation TCP position
    rob.set_payload(0.98, (0, 0, 0))  # payload to kg
    # Robot payload = 0.09kg(bracket) + 0.65kg(RG2) + ??? (??? : work piece)
    try:
        f = open('D:\\new\\sampling.csv', 'w', newline="")  # .csv file open
        # open(' 경로 + 파일명(각자 파일 경로 파악 필요).csv', 'w', newline="") 
        makefile = csv.writer(f)

        makefile.writerow(['time', 'base', 'shoulder', 'elbow', 'joint1', 'joint2', 'joint3'])
        # Line break
        # Enter 7 items to be in the first line of the file to be created. , Add as column

        t_begin = time.time()  # Read current time
        t_now = time.time()

        # Value Function
        joint = rob.getj()  # Take Radian for each part of the robot arm [Base, shoulder, Elbow, Joint1, Joint2, Joint3]
        tcp_po = rob.getl()  # Import to PCP [ x, y, z, Rx, Ry, Rz ] (x,y,z is m(Meter)) (R~(Radian)))
        tcp_cur = rob.get_pose()  # get current transform from base to to tcp
        tool_pos = rob.get_pos()  # get tool tip pos(x,y,z) in base coordinate system
        tool_ori = rob.get_orientation()  # get tool orientation in base coordinate system

        print("get joint radius : getj()")
        print(joint)
        print(" ")
        print("get tcp position : getl()")
        print(tcp_po)
        print(" ")
        print("get current transform from base to to tcp : get_pose()")
        print(tcp_cur)
        print(" ")
        print("get tool tip pos(x,y,z) in base coordinate system : get_pos()")
        print(tool_pos)
        print(" ")
        print("get tool orientation in base coordinate system : get_orientation()")
        print(tool_ori)
        print(" ")
        print("Move URsim view front of me")
        wait()  # Wait until to type Enter
        wait()

        t_now = time.time() - t_begin # Time from initial time to now
        joint_now = rob.getj()  # Receiving the current joint value
        makefile.writerow([t_now, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]])
        # Record the time and values of each joint to a file

        # Command to move
        rob.movej([1.570796, -1.570796, 0, -1.570796, 0, 0], acc=1.5, vel=1, wait=False)
        # movej : Command to move by giving a value in radians to each joint
        # ( [Base, shoulder, Elbow, Joint1, Joint2, Joint3], acc = acceleration rad/s^2, vel = speed rad/s, wait = False)
        print("move position.")
        wait()

        t_now = time.time() - t_begin # Time from initial time to now
        joint_now = rob.getj()  # Receiving the current joint value
        makefile.writerow([t_now, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]])
        # Record the time and values of each joint to a file

        pre = [0, -1.57, 0, -1.57, 0, 0]
        rob.movej(pre, acc=1.5, vel=1, wait=False)
        print("Move by time")
        wait()  # Wait until enter key is pressed
        t_begin = time.time()  # Read current time
        x = sp.symbols('x')  # Declare that x will be used as an unknown variable

        # Route 1st Start

        t_now = time.time() - t_begin  # Time from initial time to now
        eq1_1 = 0 * x
        eq1_2 = -1.57 + 0 * x
        eq1_3 = -1.57 * sp.sin(0.314 * x)
        eq1_4 = -1.57 + 0 * x
        eq1_5 = 0 * x
        eq1_6 = 0 * x

        while t_now <= 5:  # 5 sec 
            t_now = time.time() - t_begin  # Time from initial time to now
            joint_now = rob.getj()  # Receiving the current joint value
            makefile.writerow(
                [t_now, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]])
            # Record the time and values of each joint to a file

            t_now = time.time() - t_begin  # Time from initial time to now

            rob.movej([eq1_1.subs(x, t_now), eq1_2.subs(x, t_now), eq1_3.subs(x, t_now), eq1_4.subs(x, t_now),
                       eq1_5.subs(x, t_now), eq1_6.subs(x, t_now)], acc=1, vel=5, wait=False)

            # Robot arm drive by adding (time*constant) value from the specified joint angle
            t_now = time.time() - t_begin  # Time from initial time to now
            time.sleep(0.5)  # waiting time 0.05sec


        # Route 2nd Start

        t_now = time.time() - t_begin  # Time from initial time to now
        eq2_1 = 1.57 * sp.sin(0.314 * x)-1.57
        eq2_2 = -1.57 + 0 * x
        eq2_3 = -1.57 + 0 * x
        eq2_4 = -1.57 + 0 * x
        eq2_5 = 0 * x
        eq2_6 = 0 * x

        while t_now <= 10:  # 10 sec
            t_now = time.time() - t_begin  # Time from initial time to now
            time_while_2 = t_now - 5  # If you want to measure or use the time of the loop only, use time_while_2
                                    # The content is the time from the beginning to the present-5 seconds 
                                    # (where 5 seconds is because 5 seconds were used in the previous loop)                              
            joint_now = rob.getj()  # Receiving the current joint value
            makefile.writerow([t_now, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]])
            # Record the time and values of each joint to a file

            t_now = time.time() - t_begin  # Time from initial time to now

            rob.movej([eq2_1.subs(x, t_now), eq2_2.subs(x, t_now), eq2_3.subs(x, t_now), eq2_4.subs(x, t_now),
                       eq2_5.subs(x, t_now), eq2_6.subs(x, t_now)], acc=1, vel=5, wait=False)

        # Robot arm drive by adding (time*constant) value from the specified joint angle
            t_now = time.time() - t_begin  # Time from initial time to now
            time.sleep(0.5)  # waiting time 0.05sec



        # **** Do not touch ****
        f.close()
        # End
    finally:
        rob.close()  # Robot connection done.