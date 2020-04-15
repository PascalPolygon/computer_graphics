import numpy as np
import matplotlib.pyplot as plt
from vpython import*
import random
import math

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

def rotate_frame(robot, deg, deg2, deg3):
    # angle = rads(-deg)
    print('deg: %d, deg1: %d, deg2: %d' % (deg, deg2, deg3))
    deg = -1*deg
    axis_y = robot.p2.pos.y
    axis_x = robot.p2.pos.x
    start = robot.theta1
    print("Theta 1: %d" % start)
    if deg < start:
        step = -1
    else:
        step = 1
    angle1 = rads(deg-1)

    for i in range(start, deg, step):
        rate(100)
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
     

        ######## Animating link2 using trig from p3 reference frame
        # robot.l2.pos.x = p3_pos[0] + np.dot((robot.l2.height/2+robot.p3.radius), np.cos(rads(90+i)))
        # robot.l2.pos.y = p3_pos[1] + np.dot((robot.l2.height/2+robot.p3.radius), np.sin(rads(90+i)))
        # robot.l2.rotate(angle=rads(step), axis=vector(0,0,1))

        Tp3_l2 = get_transformation(rads(i), 0,(robot.l2.height/2)+(robot.p4.radius))
        T0_l2 = np.dot(T0_2, Tp3_l2)
        l2_pos = np.dot(T0_l2, np.array([0,0,1]))
        robot.l2.rotate(angle=rads(step), axis=vector(0,0,1))
        robot.l2.pos.x = l2_pos[0]
        robot.l2.pos.y = l2_pos[1]

        robot.p4.pos.x = p4_pos[0]
        robot.p4.pos.y = p4_pos[1]
        # print('deg: %f -> p4 x: %f, y: %f' % (i, p4_pos[0], p4_pos[1]))
        # robot.l3.pos.x = p4_pos[0] + np.dot((robot.l3.height/2+robot.p4.radius), np.cos(rads(90+i)))
        # robot.l3.pos.y = p4_pos[1] + np.dot((robot.l3.height/2+robot.p4.radius), np.sin(rads(90+i)))
        # robot.l3.rotate(angle=rads(step), axis=vector(0,0,1))
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
        step = -1
    else:
        step = 1
    angle2 = rads(deg-1)
    axis_y = robot.p3.pos.y
    axis_x = robot.p3.pos.x
    # print("start: %d, deg: %d" % (start, deg))
    for i in range(start, deg, step):
        rate(100)
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
        # print('deg: %f -> p4 x: %f, y: %f' % (i, p4_pos[0], p4_pos[1]))
        # robot.l3.pos.x = p4_pos[0] + np.dot((robot.l3.height/2+robot.p4.radius), np.cos(rads(90+i)))
        # robot.l3.pos.y = p4_pos[1] + np.dot((robot.l3.height/2+robot.p4.radius), np.sin(rads(90+i)))
        # robot.l3.rotate(angle=rads(step), axis=vector(0,0,1))

              ######## Animating link 3 using transformation matrix
        Tp4_l3 = get_transformation(0, 0,(robot.l3.height/2)+(robot.p5.radius))
        T0_l3 = np.dot(T0_3, Tp4_l3)
        l3_pos = np.dot(T0_l3, np.array([0,0,1]))
        robot.l3.rotate(angle=rads(step), axis=vector(0,0,1))
        robot.l3.pos.x = l3_pos[0]
        robot.l3.pos.y = l3_pos[1]

        robot.p5.pos.x = p5_pos[0]
        robot.p5.pos.y = p5_pos[1]
        
    print("#### THIRD LOOP ###")
    deg = -1*deg3
    start = robot.theta3
    if deg < start:
        step = -1
    else:
        step = 1
    angle3 = rads(deg-1)
    axis_y = robot.p4.pos.y
    axis_x = robot.p4.pos.x
    # print("start: %d, deg: %d" % (start, deg))
    for i in range(start, deg, step):
        rate(100)
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
        # print('deg: %f -> p5 x: %f, y: %f' % (i, p5_pos[0], p5_pos[1]))

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

    l2 = box(pos=vector(0, 30.5, 0), size=vector(3,10, 3), color=color.purple) 

    p4 = cylinder(pos=vector(
            0, 36.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius) #diameter 2

    l3 = box(pos=vector(0, 42.5, 0), size=vector(3,10, 3), color=color.purple)
   
    p5 = cylinder(pos=vector(
            0, 48.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius) #diameter 2

    # origin = sphere(pos=vector(0,0,0), radius=5, color = color.red)
  
    myBot = Robot(p2, l1, p3, l2, p4, l3, p5)
    # rotate_frame(myBot, -100, 100, -45)

    
    # rotate_frame(Robot, [90])
    print("Ready to boogy master chief!")
    # print("Robot awaiting your command...")
    # cmmd = input()
    cmmd_array = list()
    while True:
        # rotate_frame(myBot, int(cmmd))
        print("Robot awaiting your command...")
        for i in range(3):
            cmmd = input("theta"+str(i+1)+":")
            angle = int(cmmd)
            if math.isnan(angle):
                break
            cmmd_array.append(angle)
        # cmmd = input()
        print("cmmd1: %d, cmmd2: %d, cmmd3: %d" % (cmmd_array[0], cmmd_array[1], cmmd_array[2]))
        rotate_frame(myBot, cmmd_array[0], cmmd_array[1], cmmd_array[2])
        cmmd_array.clear()
        # print(cmmd)

# print("Shutting down...")


if __name__ == "__main__":
    main()



# def rotate_frame(robot, cmmds):
#     for l in range(len(robot)-1):
#         deg = -1*cmmds[l]
#         bot = robot[l]
#         y = bot.l.pos.y
#         axis_y = y - ((bot.l.height/2)+bot.p.radius)
#         for i in range(0, deg-1, -1):
#             rate(100)
#             bot.l.rotate(angle=rads(-1),axis=vector(0,0,1), origin=vector(0,axis_y,0))
#             angle1 = rads(i)
#             T_i = get_transformation(angle1, 0, bot.p.pos.y)
#             angle2 = rads(0)
#             T_ii = get_transformation(angle2, 0, bot.l.height+(2*robot[l+1].p.radius))
#             T = np.dot(T_i, T_ii)
#             p_i_pos = np.dot(T_i, np.array([0,0,1]))
#             p_ii_pos = np.dot(T, np.array([0,0,1]))
#             print('deg: %f ->P3 x: %f, y: %f' % (i, p_i_pos[0], p_ii_pos[1]))
#             robot[l+1].p.pos.x = p_ii_pos[0]
#             robot[l+1].p.pos.y = p_ii_pos[1]
#             robot

  # class Link:
    #     def __init__(self, p, l):
    #         self.p = p
    #         self.l = l

    # Robot = []
    # Robot.append(Link(p2, l1))
    # Robot.append(Link(p3, 0))
    # Robot.append(Link(p4, l3))
    # Robot.append(Link(p5, 0)) #There is no link 

       ######## Animating link 2 using transformation matrix
        # Tp3_l2 = get_transformation(rads(i), 0,(robot.l2.height/2)+(robot.p4.radius))
        # T0_l2 = np.dot(T0_2, Tp3_l2)
        # l2_pos = np.dot(T0_l2, np.array([0,0,1]))
        # robot.l2.rotate(angle=rads(-1), axis=vector(0,0,1))
        # robot.l2.pos.x = l2_pos[0]
        # robot.l2.pos.y = l2_pos[1]