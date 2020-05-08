import numpy as np

def rads(deg):
    return np.radians(deg)

def degrees(rads):
    return np.degrees(rads)

def cos(deg):
    return np.cos(deg)

def sin(deg):
    return np.sin(deg)

def get_transformation(angle, dx, dy):
    return np.array([[cos(angle), -1*sin(angle), dx], 
                    [sin(angle), cos(angle), dy],
                    [0, 0, 1]])