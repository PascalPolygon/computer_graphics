import numpy as np
from linalg_utils import *
from vpython import*


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

    def move(self, deg, deg2, deg3):
        # angle = rads(-deg)
        print(f'deg: {deg}, deg2: {deg2}, deg3: {deg3}')
        # myStep = 0.003
        myStep = 0.01
        deg = -1*deg
        axis_y = self.p2.pos.y
        axis_x = self.p2.pos.x
        start = self.theta1
        print("Theta 1: %d" % start)
        if deg < start:
            step = -myStep
        else:
            step = myStep
        angle1 = rads(deg-1)

        for i in np.arange(start, deg, step):
            # rate(100)
            self.l1.rotate(angle=rads(step), axis=vector(
                0, 0, 1), origin=vector(axis_x, axis_y, 0))
            self.theta1 = deg
            angle1 = rads(i)
            T0_1 = get_transformation(angle1, 0, self.p2.pos.y)
            # angle2 = rads(0)
            angle2 = rads(self.theta2)
            T1_2 = get_transformation(
                angle2, 0, self.l1.height+(2*self.p3.radius))
            # angle3 = rads(0)
            angle3 = rads(self.theta3)
            # Really doesn't make a difference p3 or p4 or p5
            T2_3 = get_transformation(
                angle3, 0, self.l2.height+(2*self.p4.radius))
            # angle4 = rads(0)
            T3_4 = get_transformation(
                rads(0), 0, self.l3.height+(2*self.p5.radius))
            T0_2 = np.dot(T0_1, T1_2)
            T0_3 = np.dot(T0_2, T2_3)
            T0_4 = np.dot(T0_3, T3_4)

            p2_pos = np.dot(T0_1, np.array([0, 0, 1]))
            p3_pos = np.dot(T0_2, np.array([0, 0, 1]))
            p4_pos = np.dot(T0_3, np.array([0, 0, 1]))
            p5_pos = np.dot(T0_4, np.array([0, 0, 1]))

            # print('deg: %f -> end effector x: %f, y: %f' % (i, p5_pos[0], p5_pos[1]))
            self.p3.pos.x = p3_pos[0]
            self.p3.pos.y = p3_pos[1]

            Tp3_l2 = get_transformation(
                rads(i), 0, (self.l2.height/2)+(self.p4.radius))
            T0_l2 = np.dot(T0_2, Tp3_l2)
            l2_pos = np.dot(T0_l2, np.array([0, 0, 1]))
            self.l2.rotate(angle=rads(step), axis=vector(0, 0, 1))
            self.l2.pos.x = l2_pos[0]
            self.l2.pos.y = l2_pos[1]

            self.p4.pos.x = p4_pos[0]
            self.p4.pos.y = p4_pos[1]

            Tp4_l3 = get_transformation(
                0, 0, (self.l3.height/2)+(self.p5.radius))
            T0_l3 = np.dot(T0_3, Tp4_l3)
            l3_pos = np.dot(T0_l3, np.array([0, 0, 1]))
            self.l3.rotate(angle=rads(step), axis=vector(0, 0, 1))
            self.l3.pos.x = l3_pos[0]
            self.l3.pos.y = l3_pos[1]

            self.p5.pos.x = p5_pos[0]
            self.p5.pos.y = p5_pos[1]
            # print('deg: %f -> p5 x: %f, y: %f' % (i, p5_pos[0], p5_pos[1]))
        print("### SECOND LOOP ###")
        deg = -1*deg2
        start = self.theta2
        if deg < start:
            step = -myStep
        else:
            step = myStep
        angle2 = rads(deg-1)
        axis_y = self.p3.pos.y
        axis_x = self.p3.pos.x
        # print("start: %d, deg: %d" % (start, deg))
        for i in np.arange(start, deg, step):
            # rate(100)
            self.l2.rotate(angle=rads(step), axis=vector(
                0, 0, 1), origin=vector(axis_x, axis_y, 0))
            self.theta2 = deg
            # angle1 = rads(0)
            T0_1 = get_transformation(angle1, 0, self.p2.pos.y)
            angle2 = rads(i)
            T1_2 = get_transformation(
                angle2, 0, self.l1.height+(2*self.p3.radius))
            # angle3 = rads(0)
            angle3 = rads(self.theta3)
            # Really doesn't make a difference p3 or p4 or p5
            T2_3 = get_transformation(
                angle3, 0, self.l2.height+(2*self.p4.radius))
            # angle4 = rads(0)
            T3_4 = get_transformation(
                rads(0), 0, self.l3.height+(2*self.p5.radius))
            T0_2 = np.dot(T0_1, T1_2)
            T0_3 = np.dot(T0_2, T2_3)
            T0_4 = np.dot(T0_3, T3_4)

            p2_pos = np.dot(T0_1, np.array([0, 0, 1]))
            p3_pos = np.dot(T0_2, np.array([0, 0, 1]))
            p4_pos = np.dot(T0_3, np.array([0, 0, 1]))
            p5_pos = np.dot(T0_4, np.array([0, 0, 1]))

            self.p4.pos.x = p4_pos[0]
            self.p4.pos.y = p4_pos[1]

            # Animating link 3 using transformation matrix
            Tp4_l3 = get_transformation(
                0, 0, (self.l3.height/2)+(self.p5.radius))
            T0_l3 = np.dot(T0_3, Tp4_l3)
            l3_pos = np.dot(T0_l3, np.array([0, 0, 1]))
            self.l3.rotate(angle=rads(step), axis=vector(0, 0, 1))
            self.l3.pos.x = l3_pos[0]
            self.l3.pos.y = l3_pos[1]

            self.p5.pos.x = p5_pos[0]
            self.p5.pos.y = p5_pos[1]

            # print('deg: %f -> p5 x: %f, y: %f' % (i, p5_pos[0], p5_pos[1]))
        print("#### THIRD LOOP ###")
        deg = -1*deg3
        start = self.theta3
        if deg < start:
            step = -myStep
        else:
            step = myStep
        angle3 = rads(deg-1)
        axis_y = self.p4.pos.y
        axis_x = self.p4.pos.x
        # print("start: %d, deg: %d" % (start, deg))
        for i in np.arange(start, deg, step):
            # rate(100)
            self.l3.rotate(angle=rads(step), axis=vector(
                0, 0, 1), origin=vector(axis_x, axis_y, 0))
            # print(i)
            self.theta3 = deg
            # angle1 = rads(0)
            T0_1 = get_transformation(angle1, 0, self.p2.pos.y)
            # angle2 = rads(0)
            T1_2 = get_transformation(
                angle2, 0, self.l1.height+(2*self.p3.radius))
            angle3 = rads(i)
            # Really doesn't make a difference p3 or p4 or p5
            T2_3 = get_transformation(
                angle3, 0, self.l2.height+(2*self.p4.radius))
            # angle4 = rads(0)
            T3_4 = get_transformation(
                rads(0), 0, self.l3.height+(2*self.p5.radius))
            T0_2 = np.dot(T0_1, T1_2)
            T0_3 = np.dot(T0_2, T2_3)
            T0_4 = np.dot(T0_3, T3_4)

            p2_pos = np.dot(T0_1, np.array([0, 0, 1]))
            p3_pos = np.dot(T0_2, np.array([0, 0, 1]))
            p4_pos = np.dot(T0_3, np.array([0, 0, 1]))
            p5_pos = np.dot(T0_4, np.array([0, 0, 1]))

            self.p5.pos.x = p5_pos[0]
            self.p5.pos.y = p5_pos[1]
