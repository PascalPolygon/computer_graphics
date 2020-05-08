import numpy as np
import matplotlib.pyplot as plt
from vpython import*
import random
import math
from pathFinding import *
import sympy as sym
import time

scene = canvas()

class Robot:
    def __init__(self, p2, l1, p3, l2, p4, l3, p5):
        self.p2 = p2
        self.l1 = l1
        self.p3 = p3
        self.l2 = l2
        self.p4 = p4
        self.l3 = l3
        self.p5 = p5
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0

def rads(deg):
    return np.radians(deg)

def degrees(rads):
    return np.degrees(rads)

def cos(deg):
    return np.cos(deg)

def sin(deg):
    return np.sin(deg)

def rotate_link(robot, link, deg):
    deg = -1 * deg
    if link == 1:
        y = robot.l1.pos.y
        axis_y = y - (5+robot.p2.radius)
        for i in range(0, deg-1, -1):
            rate(100)
            # print("deg: %d, rad: %f" % (i, np.radians(i)))
            robot.l1.rotate(angle=rads(-1),axis=vector(0,0,1), origin=vector(0,axis_y,0))
            L = robot.l1.height+(2*robot.p3.radius)
            p3_x = np.dot(L,np.cos(rads(90+i)))
            p3_y = (robot.p2.pos.y) + np.dot(L, np.sin(rads(90+i)))
            # p3_y = robot.l1.pos.y+(robot.l1.height/2)+robot.p3.radius
            # print('deg: %f ->P3 x: %f, y: %f' % (i, p3_x, p3_y))
            robot.p3.pos.x = p3_x
            robot.p3.pos.y = p3_y
            # robot.p3.y = robot.l1.pos.y+(robot.l1.height/2)+robot.p3.radius*np.sin(i)

def get_transformation(angle, dx, dy):
    return np.array([[cos(angle), -1*sin(angle), dx], 
                    [sin(angle), cos(angle), dy],
                    [0, 0, 1]])

# def jacobian(v_str, f_list):
#     vars = sym.symbols(v_str)
#     f = sym.sympify(f_list)
#     J = sym.zeros(len(f),len(vars))
#     for i, fi in enumerate(f):
#         for j, s in enumerate(vars):
#             J[i,j] = sym.diff(fi, s)
#     return J

def jacobian(phi, l):
    delt_i = rads(0.1)
    de_dphi = np.ndarray((3,len(phi)))
    d_phi = np.zeros(len(phi))
    for i in range(len(phi)):
        d_phi[i] =  delt_i
        num1 = e(phi+d_phi,l)
        num2 = e(phi,l)
        de_dphi[i]= (num1-num2)/delt_i

    de_dphi = de_dphi.transpose()
    de_dphi = de_dphi[:2,:]
    return de_dphi
        

def rotate_frame(robot, deg, deg2, deg3):
    # angle = rads(-deg)
    print(f'deg: {deg}, deg2: {deg2}, deg3: {deg3}')
    deg = -1*deg
    axis_y = robot.p2.pos.y
    axis_x = robot.p2.pos.x
    start = robot.theta1
    print("Theta 1: %d" % start)
    if deg < start:
        step = -0.0001
    else:
        step = 0.0001
    angle1 = rads(deg-1)

    for i in np.arange(start, deg, step):
        # rate(100)
        robot.l1.rotate(angle=rads(step),axis=vector(0,0,1), origin=vector(axis_x, axis_y,0))
        robot.theta1 = deg
        angle1 = rads(i)
        T0_1 = get_transformation(angle1, 0, robot.p2.pos.y)
        # angle2 = rads(0)
        angle2 = rads(robot.theta2)
        T1_2 = get_transformation(angle2, 0, robot.l1.height+(2*robot.p3.radius))
        # angle3 = rads(0)
        angle3 = rads(robot.theta3)
        T2_3 = get_transformation(angle3, 0, robot.l2.height+(2*robot.p4.radius)) #Really doesn't make a difference p3 or p4 or p5
        # angle4 = rads(0)
        T3_4 = get_transformation(rads(0), 0, robot.l3.height+(2*robot.p5.radius))
        T0_2 = np.dot(T0_1, T1_2)
        T0_3 = np.dot(T0_2, T2_3)
        T0_4 = np.dot(T0_3, T3_4)

        p2_pos = np.dot(T0_1,np.array([0,0,1]))
        p3_pos = np.dot(T0_2, np.array([0,0,1]))
        p4_pos = np.dot(T0_3, np.array([0,0,1]))
        p5_pos = np.dot(T0_4, np.array([0,0,1]))

        # print('deg: %f -> end effector x: %f, y: %f' % (i, p5_pos[0], p5_pos[1]))
        robot.p3.pos.x = p3_pos[0]
        robot.p3.pos.y = p3_pos[1]

        Tp3_l2 = get_transformation(rads(i), 0,(robot.l2.height/2)+(robot.p4.radius))
        T0_l2 = np.dot(T0_2, Tp3_l2)
        l2_pos = np.dot(T0_l2, np.array([0,0,1]))
        robot.l2.rotate(angle=rads(step), axis=vector(0,0,1))
        robot.l2.pos.x = l2_pos[0]
        robot.l2.pos.y = l2_pos[1]

        robot.p4.pos.x = p4_pos[0]
        robot.p4.pos.y = p4_pos[1]

        Tp4_l3 = get_transformation(0, 0,(robot.l3.height/2)+(robot.p5.radius))
        T0_l3 = np.dot(T0_3, Tp4_l3)
        l3_pos = np.dot(T0_l3, np.array([0,0,1]))
        robot.l3.rotate(angle=rads(step), axis=vector(0,0,1))
        robot.l3.pos.x = l3_pos[0]
        robot.l3.pos.y = l3_pos[1]

        robot.p5.pos.x = p5_pos[0]
        robot.p5.pos.y = p5_pos[1]
        # print('deg: %f -> p5 x: %f, y: %f' % (i, p5_pos[0], p5_pos[1]))
    print("### SECOND LOOP ###")
    deg = -1*deg2
    start = robot.theta2
    if deg < start:
        step = -0.0001
    else:
        step = 0.0001
    angle2 = rads(deg-1)
    axis_y = robot.p3.pos.y
    axis_x = robot.p3.pos.x
    # print("start: %d, deg: %d" % (start, deg))
    for i in np.arange(start, deg, step):
        # rate(100)
        robot.l2.rotate(angle=rads(step),axis=vector(0,0,1), origin=vector(axis_x, axis_y,0))
        robot.theta2 = deg
        # angle1 = rads(0)
        T0_1 = get_transformation(angle1, 0, robot.p2.pos.y)
        angle2 = rads(i)
        T1_2 = get_transformation(angle2, 0, robot.l1.height+(2*robot.p3.radius))
        # angle3 = rads(0)
        angle3 = rads(robot.theta3)
        T2_3 = get_transformation(angle3, 0, robot.l2.height+(2*robot.p4.radius)) #Really doesn't make a difference p3 or p4 or p5
        # angle4 = rads(0)
        T3_4 = get_transformation(rads(0), 0, robot.l3.height+(2*robot.p5.radius))
        T0_2 = np.dot(T0_1, T1_2)
        T0_3 = np.dot(T0_2, T2_3)
        T0_4 = np.dot(T0_3, T3_4)

        p2_pos = np.dot(T0_1,np.array([0,0,1]))
        p3_pos = np.dot(T0_2, np.array([0,0,1]))
        p4_pos = np.dot(T0_3, np.array([0,0,1]))
        p5_pos = np.dot(T0_4, np.array([0,0,1]))

        robot.p4.pos.x = p4_pos[0]
        robot.p4.pos.y = p4_pos[1]

              ######## Animating link 3 using transformation matrix
        Tp4_l3 = get_transformation(0, 0,(robot.l3.height/2)+(robot.p5.radius))
        T0_l3 = np.dot(T0_3, Tp4_l3)
        l3_pos = np.dot(T0_l3, np.array([0,0,1]))
        robot.l3.rotate(angle=rads(step), axis=vector(0,0,1))
        robot.l3.pos.x = l3_pos[0]
        robot.l3.pos.y = l3_pos[1]

        robot.p5.pos.x = p5_pos[0]
        robot.p5.pos.y = p5_pos[1]

        # print('deg: %f -> p5 x: %f, y: %f' % (i, p5_pos[0], p5_pos[1]))
    print("#### THIRD LOOP ###")
    deg = -1*deg3
    start = robot.theta3
    if deg < start:
        step = -0.0001
    else:
        step = 0.0001
    angle3 = rads(deg-1)
    axis_y = robot.p4.pos.y
    axis_x = robot.p4.pos.x
    # print("start: %d, deg: %d" % (start, deg))
    for i in np.arange(start, deg, step):
        # rate(100)
        robot.l3.rotate(angle=rads(step),axis=vector(0,0,1), origin=vector(axis_x, axis_y,0))
        # print(i)
        robot.theta3 = deg
        # angle1 = rads(0)
        T0_1 = get_transformation(angle1, 0, robot.p2.pos.y)
        # angle2 = rads(0)
        T1_2 = get_transformation(angle2, 0, robot.l1.height+(2*robot.p3.radius))
        angle3 = rads(i)
        T2_3 = get_transformation(angle3, 0, robot.l2.height+(2*robot.p4.radius)) #Really doesn't make a difference p3 or p4 or p5
        # angle4 = rads(0)
        T3_4 = get_transformation(rads(0), 0, robot.l3.height+(2*robot.p5.radius))
        T0_2 = np.dot(T0_1, T1_2)
        T0_3 = np.dot(T0_2, T2_3)
        T0_4 = np.dot(T0_3, T3_4)

        p2_pos = np.dot(T0_1,np.array([0,0,1]))
        p3_pos = np.dot(T0_2, np.array([0,0,1]))
        p4_pos = np.dot(T0_3, np.array([0,0,1]))
        p5_pos = np.dot(T0_4, np.array([0,0,1]))

        robot.p5.pos.x = p5_pos[0]
        robot.p5.pos.y = p5_pos[1]

def e(phi, l):
    T0_1 = get_transformation(phi[0], 0, l[0])
    # print(T0_1)
    T1_2 = get_transformation(phi[1], 0, l[1])
    # print(T1_2)
    T2_3 = get_transformation(phi[2], 0, l[2])
    # print(T2_3)
    T3_4 = get_transformation(0, 0, l[3])

    T0_2 = np.dot(T0_1, T1_2)
    T0_3 = np.dot(T0_2, T2_3)
    T0_4 = np.dot(T0_3, T3_4)
    v = np.array([0,0,1])
    # print(T0_5)
    # print(np.dot(T0_5,v))
    return np.dot(T0_4,v)

def main():
    print("Hello world!")
    joint_radius = 1
    # platform = box(pos=vector(0, 0, 0), size=vector(
    #     100,3, 100), color=color.orange)

    # base =  box(pos=vector(0, 6.5, 0), size=vector(
    #     10,10, 10), color=color.orange) #6.5 = 3/2 + 10/2

    p2 = cylinder(pos=vector(
            0, 12.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius) #diameter 2
            
    l1 = box(pos=vector(0, 18.5, 0), size=vector(
        3,10, 3), color=color.purple) #18 = 11.5+2+10/2
 
    p3 = cylinder(pos=vector(
            0, 24.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius) #diameter 2

    p3_y = l1.pos.y+(l1.height/2)+p3.radius*np.sin(90)
    p3_x = l1.pos.x*np.cos(90)
    print('L1 length: %f' % l1.height)
    # print("p2 y: %f" % p2.pos.y)
    print('p3_y: %f' % p3_y)
    print('p3_x: %f' % p3_x)

    l2 = box(pos=vector(0, 30.5, 0), size=vector(3,10, 3), color=color.purple) 

    p4 = cylinder(pos=vector(
            0, 36.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius) #diameter 2

    l3 = box(pos=vector(0, 42.5, 0), size=vector(3,10, 3), color=color.purple)
   
    p5 = cylinder(pos=vector(
            0, 48.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius, color=color.red) #diameter 2

    # origin = sphere(pos=vector(0,0,0), radius=5, color = color.red)
  
    myBot = Robot(p2, l1, p3, l2, p4, l3, p5)
    # rotate_frame(myBot, -100, 100, -45)

    
    # rotate_frame(Robot, [90])
    print("Ready to boogy master chief!")
    # print("Robot awaiting your command...")
    # cmmd = input()
    # cmmd_array = list()

    x_g = 35
    y_g = 21

    goal = sphere(pos=vector(x_g, y_g, -1.5),radius=joint_radius, color=color.green) #diameter 2

    x = myBot.p5.pos.x
    y = myBot.p5.pos.y
    
    # lmbda = -0.1
   
    C = computeCost(x, y, x_g, y_g)

    Ts0_1 = sym.Matrix([['cos(phi1)', '-1*sin(phi1)', 0],['sin(phi1)', 'cos(phi1)', myBot.p2.pos.y],[0, 0, 1]])
    Ts1_2 = sym.Matrix([['cos(phi2)', '-1*sin(phi2)', 0],['sin(phi2)', 'cos(phi2)', myBot.l1.height+(2*myBot.p3.radius)],[0, 0, 1]])
    Ts2_3 = sym.Matrix([['cos(phi3)', '-1*sin(phi3)', 0],['sin(phi3)', 'cos(phi3)', myBot.l2.height+(2*myBot.p4.radius)],[0, 0, 1]])
    Ts3_4 = sym.Matrix([['cos(0)', '-1*sin(0)', 0],['sin(0)', 'cos(0)', myBot.l3.height+(2*myBot.p5.radius)],[0, 0, 1]])
    ######## UNCOMMENT TO GO BACK TO SYMPY
    # v = sym.Matrix([0, 0, 1])
    # e = Ts0_1*Ts1_2*Ts2_3*Ts3_4*v
    # J = jacobian('phi1 phi2 phi3', [e[0], e[1]])
    # e = e.subs([('phi1', rads(myBot.theta1)), ('phi2', rads(myBot.theta2)), ('phi3', rads(myBot.theta3))])
    # e = np.array(e, dtype=np.float64) #convert sympy array to numpy array (different float representation)
    # e_th = e
    ############################################
    # print(e[0])
    phi1 = 0
    phi2 = 0
    phi3 = 0
    
    magic_p5 = cylinder(pos=vector(
            0, 48.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius, color=color.yellow) #diameter 2
    
    # C_th = computeCost(e_th[0], e_th[1], x_g, y_g)

    # # Theoretical 
    # while C_th > 0.5:
    #     # dx_th, dy_th = gradientDesc(e_th[0], e_th[1], x_g, y_g)
    #     dx_th = x_g - e_th[0]
    #     dy_th = y_g - e_th[1]
    #     de_th = beta*sym.Matrix([dx_th, dy_th])
    #     # de_th = sym.Matrix([1.3817913413605396, -0.03095465824443977])
    #     print(f'de: {de_th}')
    #     J_num_th = J.subs([('phi1', rads(phi1)), ('phi2', rads(phi2)), ('phi3', rads(phi3))])
    #     J_pinv_th = J_num_th.pinv() #moore_prenrose pseudo inverse
    #     print(f'J_pinv: {J_pinv_th}')
    #     d_phi_th = J_pinv_th*de_th
    #     d_phi_th = np.array(d_phi_th, dtype=np.float64)

    #     phi1 = (phi1) + degrees(d_phi_th[0])
    #     phi2 = (phi2) + degrees(d_phi_th[1])
    #     phi3 = (phi3) + degrees(d_phi_th[2])
     
    #     print(f'New angles should be: {phi1}, {phi2}, {phi3}')

    #     # After rotate_frame is done executing global robot object is updated to new position
    #     e_th[0] = e_th[0]+de_th[0]
    #     e_th[1] = e_th[1]+de_th[1]
    #     magic_p5.pos.x = e_th[0]
    #     magic_p5.pos.y = e_th[1]
    #     print(f'Magic at: {e_th}')
    #     # rotate_frame(myBot, -1*phi1[0], -1*phi2[0], -1*phi3[0])
    #     print(f'Robot at: {myBot.p5.pos}')
    #     C_th = computeCost(e_th[0], e_th[1], x_g, y_g)
    #     print(f'thoeritical cost {C_th}')
    #     print('-----------------------------------------------------')
    # Theoretical
    # ########### Manual's jacobian computation ###################
    l = np.array([myBot.p2.pos.y, myBot.l1.height+(2*myBot.p3.radius), myBot.l2.height+(2*myBot.p4.radius), myBot.l3.height+(2*myBot.p5.radius)])
    phi = np.array([rads(1), rads(phi2), rads(phi3)])
    e_th = np.array([0, 48.5])
    e_c = np.array([0, 48.5])
    g = np.array([35, 21])
    beta = 0.0001
    C_th = computeCost(e_th[0], e_th[1], g[0], g[1])
    i = 0
    while C_th > 5:
    # for i in range(10000):
        J = jacobian(phi, l)
        # print(J)
        J_pinv = np.linalg.pinv(J)
        # print(J_pinv)
        de_c = beta*(g-e_c)
        de_th = beta*(g-e_th)
        d_phi = np.dot(J_pinv, de_c)
        # print(phi)
        # print(d_phi)
        phi+=d_phi
        # print(phi)
        phi_deg = degrees(phi)
        print(f'Go to: {phi_deg}')

        # if (i % 10):
        #     rotate_frame(myBot, -1*phi_deg[0], -1*phi_deg[1], -1*phi_deg[2])
        # i+= 1
        # time.sleep(0.5)
        e_c = e(phi, l)[:2]
        e_th+=de_th
        magic_p5.pos.x = e_c[0]
        magic_p5.pos.y = e_c[1]
        print(e_c)
        print(e_th)
        print('_____________________________')
        C_th = computeCost(e_c[0], e_c[1], g[0], g[1])
        # C_th = computeCost(e_th[0], e_th[1], g[0], g[1])
        # if (e_c[0] < 0):
        #     break
        
        # for i in range(len(phi)):
        #     d_phi = np.zeros(len(phi))
        #     d_phi[i] =  delt_i
        #     num1 = e(phi+d_phi,l)
        #     num2 = e(phi,l)
        #     de_dphi[i]= (num1-num2)/delt_i

        # de_dphi = de_dphi.transpose()
        # de_dphi = de_dphi[:2,:]
        # print(de_dphi)
    #################################################################

# Actual robot
    # while C > 20:
    #     dx, dy = gradientDesc(e[0], e[1], x_g, y_g)
    #     de = lmbda*sym.Matrix([dx, dy])
    #     print(f'de: {de}')
    #     J_num = J.subs([('phi1', rads(-1*(myBot.theta1))), ('phi2', rads(-1*(myBot.theta2))), ('phi3', rads(-1*(myBot.theta3)))]) #plug in angle values into symbolic jacobian
    #     print(f'J num: {J_num}')
    #     J_pinv = J_num.pinv() #moore_prenrose pseudo inverse
    #     d_phi = J_pinv*de
    #     print(f'd_phi: {d_phi}')
    #     d_phi = np.array(d_phi, dtype=np.float64) #convert sympy array to numpy array (different float representation)

    #     phi_1 = -1*(myBot.theta1) + degrees(d_phi[0])
    #     phi_2 = -1*(myBot.theta2) + degrees(d_phi[1])
    #     phi_3 = -1*(myBot.theta3) + degrees(d_phi[2])

    #     print(f'in degs: {phi_1[0]}, {phi_2[0]}, {phi_3[0]}')

    #     rotate_frame(myBot, phi_1[0], phi_2[0], phi_3[0])
    #     print(f'New angles: {myBot.theta1}, {myBot.theta2}, {myBot.theta3}')

    #     e[0] = myBot.p5.pos.x
    #     e[1] = myBot.p5.pos.y 
    #     print(f'Is at {e}')

    #     C = computeCost(e[0], e[1], x_g, y_g)
    #     print(f'cost: {C}')


if __name__ == "__main__":
    main()