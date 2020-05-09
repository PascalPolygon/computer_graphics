import numpy as np
import matplotlib.pyplot as plt
from vpython import*
import random
import math
from pathFinding import *
from robot import *
from linalg_utils import *
from PIL import ImageGrab
import os
cwd = os.getcwd()

scene = canvas()

def init_linkage():
    joint_radius = 1

    # platform = box(pos=vector(0, 0, 0), size=vector(
    #     100, 3, 100), color=color.orange)

    # base = box(pos=vector(0, 6.5, 0), size=vector(
    #     10, 10, 10), color=color.orange)  # 6.5 = 3/2 + 10/2

    p2 = cylinder(pos=vector(
        0, 12.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius)  # diameter 2

    l1 = box(pos=vector(0, 18.5, 0), size=vector(
        3, 10, 3), color=color.purple)  # 18 = 11.5+2+10/2

    p3 = cylinder(pos=vector(
        0, 24.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius)  # diameter 2

    l2 = box(pos=vector(0, 30.5, 0), size=vector(3, 10, 3), color=color.purple)

    p4 = cylinder(pos=vector(
        0, 36.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius)  # diameter 2

    l3 = box(pos=vector(0, 42.5, 0), size=vector(3, 10, 3), color=color.purple)

    p5 = cylinder(pos=vector(
        0, 48.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius, color=color.cyan, make_trail=True)  # diameter 2

    return p2, l1, p3, l2, p4, l3, p5


def jacobian(phi, l):
    delt_i = rads(0.3)
    de_dphi = np.ndarray((3, len(phi)))
    d_phi = np.zeros(len(phi))
    for i in range(len(phi)):
        d_phi[i] = delt_i
        num1 = e(phi+d_phi, l)
        num2 = e(phi, l)
        de_dphi[i] = (num1-num2)/delt_i

    de_dphi = de_dphi.transpose()
    de_dphi = de_dphi[:2, :]
    return de_dphi


def e(phi, l):
    T0_1 = get_transformation(phi[0], 0, l[0])
    T1_2 = get_transformation(phi[1], 0, l[1])
    T2_3 = get_transformation(phi[2], 0, l[2])
    T3_4 = get_transformation(0, 0, l[3])

    T0_2 = np.dot(T0_1, T1_2)
    T0_3 = np.dot(T0_2, T2_3)
    T0_4 = np.dot(T0_3, T3_4)
    v = np.array([0, 0, 1])

    return np.dot(T0_4, v)


def main():
    print("Hello world!")
    # origin = sphere(pos=vector(0,0,0), radius=5, color = color.red)
    p2, l1, p3, l2, p4, l3, p5 = init_linkage()
    myBot = Robot(p2, l1, p3, l2, p4, l3, p5)
    print("Ready to boogy master chief!")
    phi1 = 0
    phi2 = 0
    phi3 = 0

    # magic_p5 = cylinder(pos=vector(
    #         0, 48.5, -1.5), axis=vector(0, 0, 3), radius=1, color=color.yellow)

    l = np.array([myBot.p2.pos.y, myBot.l1.height+(2*myBot.p3.radius),
                  myBot.l2.height+(2*myBot.p4.radius), myBot.l3.height+(2*myBot.p5.radius)])
    phi = np.array([rads(phi1), rads(phi2), rads(phi3)])
    # e_th = np.array([0, 48.5])
    e_c = np.array([0, 48.5])
    # g = np.array([35, 21])
    g = np.array([10, -21])
    goal = sphere(pos=vector(g[0], g[1], -1.5), radius=1, color=color.green)
    # obs = np.array([[25, 31], [30, 19]])
    obs = np.array([[25, 31]])
    for i in range(obs.shape[0]):
        sphere(pos=vector(obs[i][0], obs[i][1], -1.5),
               radius=1, color=color.red)

    i = 0
    C = computeCost(e_c[0], e_c[1], g[0], g[1])
    C_obs = costWithObstables(e_c[0], e_c[1], obs)
    C_total = C+C_obs

    while C_total > 0.5:
        J = jacobian(phi, l)
        J_pinv = np.linalg.pinv(J)
        de_c = -0.1*gradWithObst(g, obs, e_c)
        # angle step to take due to goal and obstacles
        d_phi = np.dot(J_pinv, de_c)
        # angle step due to angle limitations
        d_phi_cst = gradWithCstrnt(-1*degrees(phi), sigma=10)
        # print(f'd_phi: {d_phi} d_phi_cst: {d_phi_cst}')
        phi += (d_phi - d_phi_cst)
        # phi += d_phi
        phi_deg = degrees(phi)
        # print(f'Go to: {phi_deg}')
        if (i % 20):
            myBot.move(-1*phi_deg[0], -1*phi_deg[1], -1*phi_deg[2])
            # Video creation
            # im = ImageGrab.grab((220,180,1880,1200))  # screen box from (0,0)-(500,500)
            # # im.save(cwd+'frames\img-'+str(100+i)+'.png')  
            # im.save('c:/computer_graphics/Robot_inv_kinematics_HW6/frames/img-'+str(100+i)+'.png') 
        i += 1
        e_c = e(phi, l)[:2]
        # print(e_c)
        C = computeCost(e_c[0], e_c[1], g[0], g[1])
        C_obs = costWithObstables(e_c[0], e_c[1], obs)
        C_cst = cstrntCost(-1*phi_deg)
        C_cst = np.sum(C_cst)  # Sum cost due to each angle
        C_total = C+C_obs+C_cst
        # print(f'C_total: {C}')
        # print(f'C_obs: {C_obs}')
        print(f'End effector at: {e_c}, C_total: {C_total}')
        # print(f'Angles: {-1*phi_deg}, constraint: {C_cst}')
        print('_____________________________')


if __name__ == "__main__":
    main()
