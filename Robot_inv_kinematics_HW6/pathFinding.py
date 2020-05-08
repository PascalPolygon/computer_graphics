import numpy as np

def computeCost(X, Y, x_g, y_g):
    return np.sqrt(np.square(X-x_g) + np.square(Y-y_g))

def gradientDesc(X, Y, x_g, y_g):
    d_g = computeCost(X, Y, x_g, y_g)  # euclidian distance to goal point
    dx = (X-x_g)/d_g
    dy = (Y-y_g)/d_g
    return dx, dy

