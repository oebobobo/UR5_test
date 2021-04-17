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
        # open(' 경로 + 파일명(변경가능).csv', 'w', newline="")
        makefile = csv.writer(f)

        makefile.writerow(['시간', '베이스', '숄더', '엘보우', '손목1', '손목2', '손목3'])
        # writerow(배열항목) 파일에 새로운row(열)로 배열항목을 추가 (줄바꿈)
        # 생성하는 파일의 첫 행에 들어갈 7개 항목 입력. ,로 열 추가

        # 1970년 1월 1일 자정 이후로 누적된 초를 float 단위로 반환
        t_begin = time.time()  # Read current time
        t_now = time.time()

        # Value Function
        joint = rob.getj()  # 로봇팔 관절마다의 각(라디안)을 가져옵니다 [베이스, 숄더, 엘보우, 손목1, 손목2, 손목3]
        tcp_po = rob.getl()  # TCP의 위치를 가져옵니다. [ x, y, z, Rx, Ry, Rz ] (x,y,z는 m단위) (R~ 의 단위도 라디안)
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
        wait()  # 엔터를 입력받을때까지 대기. 화면에는 Click enter to continue 출력.
        wait()

        t_now = time.time() - t_begin # Time from initial time to now
        joint_now = rob.getj()  # Receiving the current joint value
        makefile.writerow([t_now, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]])
        # Record the time and values of each joint to a file

        # Command to move
        rob.movej([1.570796, -1.570796, 0, -1.570796, 0, 0], acc=1.5, vel=1, wait=False)
        # movej : Command to move by giving a value in radians to each joint
        # ( [베이스, 숄더, 엘보우, 손목1, 손목2, 손목3], acc=가속도 rad/s^2, vel= 속도 rad/s, wait=False)
        print("move position.")
        wait()

        t_now = time.time() - t_begin # Time from initial time to now
        joint_now = rob.getj()  # Receiving the current joint value
        makefile.writerow([t_now, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]])
        # Record the time and values of each joint to a file

        pre = [0, -1.57, 0, -1.57, 0, 0]
        rob.movej(pre, acc=1.5, vel=1, wait=False)
        print("Move by time")
        wait()  # 아래 창에 "Move by time"이 나오고 수식에 따른 움직임(아래 eq1_~)을 주기 이전에 enter키를 누르기 전까지 대기
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

        while t_now <= 5:  # During to 5 sec
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

        while t_now <= 10:  # while문 내부를 5초동안 동작 (10초 - 이전루프시간 5초)
            t_now = time.time() - t_begin  # Time from initial time to now
            time_while_2 = t_now - 5  # If you want to measure or use the time of the loop only, use time_while_2
                                    # The content is the time from the beginning to the present-5 seconds 
                                    # (where 5 seconds is because 5 seconds were used in the previous loop)                              
            joint_now = rob.getj()  # Receiving the current joint value
            makefile.writerow([t_now, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]])
            # Record the time and values of each joint to a file
ㅕ
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