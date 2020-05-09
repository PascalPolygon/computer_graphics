import numpy as np


def computeCost(X, Y, x_g, y_g):
    return np.sqrt(np.square(X-x_g) + np.square(Y-y_g))


def gradientDesc(X, Y, x_g, y_g):
    d_g = computeCost(X, Y, x_g, y_g)  # euclidian distance to goal point
    dx = (X-x_g)/d_g
    dy = (Y-y_g)/d_g
    return np.array([dx, dy])


def costWithObstables(X, Y, Obs, R=3):
    # obsField = np.zeros([100, 100])
    C_obs = 0
    for i in range(0, Obs.shape[0]):
        x_o = Obs[i][0]
        y_o = Obs[i][1]
        d = computeCost(X, Y, x_o, y_o)
        C_obs += 0 if d > R else np.log(R/d)

    return C_obs


def cstrntCost(phi, sigma=10):
    C_cst = np.array([0.0, 0.0, 0.0])
    _min = -160
    _max = 160
    # print('###############_cstrntCost_################')
    for i in range(len(phi)):
        if _min < phi[i] and phi[i] <= _min+sigma:
            C_cst[i] = np.log(sigma/(phi[i] - _min))
            # print('###############################')
            # print(f'Angles: {phi}, constraint: {C_cst}')
        elif _max-sigma <= phi[i] and phi[i] < _max:
            C_cst[i] = np.log(sigma/(_max - phi[i]))
            # print('###############################')
            # print(f'Angles: {phi}, constraint: {C_cst}')
    return C_cst


def gradWithCstrnt(phi, sigma=10):
    d_phi = np.array([0.0, 0.0, 0.0])
    _min = -160
    _max = 160
    # print('###############_Angle Constraint Gradients_################')
    for i in range(len(phi)):
        if _min < phi[i] and phi[i] <= _min+sigma:
            d_phi[i] = - np.log(sigma)/(np.square(phi[i] + _min))
        elif _max-sigma <= phi[i] and phi[i] < _max:
            d_phi[i] = - np.log(sigma)/(np.square(_max - phi[i]))
    return d_phi


def gradDesc(g, e, beta=0.001):
    return beta*(g-e)


def gradWithObst(g, o, e, R=3):
    # de = gradDesc(g, e)
    de = gradientDesc(e[0], e[1], g[0], g[1])
    for i in range(0, o.shape[0]):
        x_o = o[i][0]
        y_o = o[i][1]
        C_o = computeCost(e[0], e[1], x_o, y_o)
        do = gradientDesc(e[0], e[1], x_o, y_o)
        de += 0 if C_o > R else -do
    return de
