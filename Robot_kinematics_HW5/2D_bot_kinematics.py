import numpy as np
import matplotlib.pyplot as plt
from vpython import*
import random

scene = canvas()

def rads(deg):
    return np.radians(deg)

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
            print('deg: %f ->P3 x: %f, y: %f' % (i, p3_x, p3_y))
            robot.p3.pos.x = p3_x
            robot.p3.pos.y = p3_y
            # robot.p3.y = robot.l1.pos.y+(robot.l1.height/2)+robot.p3.radius*np.sin(i)

def rotate_frame(robot, deg):
    # angle = rads(-deg)
    deg = -1*deg
    y = robot.l1.pos.y
    axis_y = y - (5+robot.p2.radius)
    for i in range(0, deg-1, -1):
        rate(100)
        robot.l1.rotate(angle=rads(-1),axis=vector(0,0,1), origin=vector(0,axis_y,0))
        angle1 = rads(i)
        T0_1 = np.array([[cos(angle1), -1*sin(angle1), 0],
                     [sin(angle1), cos(angle1), robot.p2.pos.y], 
                     [0, 0, 1]])
        angle2 = rads(0)
        T1_2 = np.array([[cos(angle2), -1*sin(angle2), 0], 
                        [sin(angle2), cos(angle2), robot.l1.height+(2*robot.p3.radius)],
                        [0, 0, 1]])
        T0_2 = np.dot(T0_1, T1_2)
        p2_pos = np.dot(T0_1,np.array([0,0,1]))
        p3_pos = np.dot(T0_2, np.array([0,0,1]))
        print('deg: %f ->P3 x: %f, y: %f' % (i, p3_pos[0], p3_pos[1]))
        robot.p3.pos.x = p3_pos[0]
        robot.p3.pos.y = p3_pos[1]
    

# def home(robot):
#     rotate_link(robot.l1, 0)
#     rotate_link(robot.l2, 0)
#     rotate_link(robot.l3, 0)
    # rotate_link(myBot.l1, 90)

def main():
    print("Hello world!")
    joint_radius = 1
    platform = box(pos=vector(0, 0, 0), size=vector(
        100,3, 100), color=color.orange)

    base =  box(pos=vector(0, 6.5, 0), size=vector(
        10,10, 10), color=color.orange) #6.5 = 3/2 + 10/2

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

    # l2 = box(pos=vector(0, 30.5, 0), size=vector(3,10, 3), color=color.purple) 

    # p4 = cylinder(pos=vector(
    #         0, 36.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius) #diameter 2

    # l3 = box(pos=vector(0, 42.5, 0), size=vector(3,10, 3), color=color.purple)
   
    # p5 = cylinder(pos=vector(
    #         0, 48.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius) #diameter 2

    # origin = sphere(pos=vector(0,0,0), radius=5, color = color.red)

    class Robot:
        def __init__(self, p2, l1, p3):
            self.p2 = p2
            self.l1 = l1
            self.p3 = p3
            # self.l2 = l2
            # self.p4 = p4
            # self.l3 = l3
            # self.p5 = p5
   
    myBot = Robot(p2, l1, p3)

    rotate_frame(myBot, 90)
   


if __name__ == "__main__":
    main()