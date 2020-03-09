import numpy as np
# from numba import jit, cuda  # To execute on GPU
# from numba import vectorize
import matplotlib.pyplot as plt
from vpython import*
import random
# from numbapro import vectorize
scene = canvas()
# sphere()


def computeCost(X, Y, x_g, y_g):
    return np.sqrt(np.square(X-x_g) + np.square(Y-y_g))


def euclDist(x1, y1, x2, y2):
    return np.sqrt(np.square(x2-x1) + np.square(y2-y1))


def gradWithObst(X, Y, x_g, y_g, Obs, R):
    d_g = euclDist(X, Y, x_g, y_g)  # euclidian distance to goal point
    dx = (X-x_g)/d_g
    dy = (Y-y_g)/d_g
    for i in range(0, Obs.shape[0]):
        x_o = Obs[i][0]
        y_o = Obs[i][1]
        d_o = euclDist(X, Y, x_o, y_o)  # euclidian distance to obstacle
        dx += 0 if d_o > R else -((X-x_o)/d_o)
        dy += 0 if d_o > R else -((Y-y_o)/d_o)
    return dx, dy


# @jit(target="cuda")  # Decorator for GPU acceleration
def costWithObstables(X, Y, Obs, R):
    # obsField = np.zeros([100, 100])
    C_obs = 0
    for i in range(0, Obs.shape[0]):
        x_o = Obs[i][0]
        y_o = Obs[i][1]
        d = euclDist(X, Y, x_o, y_o)
        C_obs += 0 if d > R else np.log(R/d)

    return C_obs


def animGradDesc(x, y, x_g, y_g, obs, R):
    C = computeCost(x, y, x_g, y_g)
    C_obs = costWithObstables(x, y, obs, R)
    C_total = C + C_obs

    # print("Initial Cost: %f" % C_total)
    lmbda = 0.1
    i = 0
    # 0,0 in animation is not a corner but the center -50 corrects that
    ball = sphere(pos=vector(x-50, 10, y-50), radius=1,
                  color=color.red, make_trail=True)

    goal = sphere(pos=vector(x_g-50, 10, y_g-50), radius=1,
                  color=color.green, make_trail=True)
    box(pos=vector(0, 0, 0), size=vector(
        100, 0.5, 100), color=color.orange)

    ball.velocity = vector(0, 0, 0)

    for i in range(0, obs.shape[0]):
        cylinder(pos=vector(
            obs[i][0]-50, 0, obs[i][1]-50), axis=vector(0, 20, 0), radius=R)

    print("Finding minimum...")
    while C_total > 0.05:
        rate(70)

        dx, dy = gradWithObst(x, y, x_g, y_g, obs, R)

        x = x - lmbda*dx
        y = y - lmbda*dy

        ball.pos = vector(x-50, 10, y-50)
        print("x: %f, y: %f" % (ball.pos.x+50, ball.pos.z+50))

        C = computeCost(x, y, x_g, y_g)
        C_obs = costWithObstables(x, y, obs, R)
        C_total = C + C_obs

        i += 1
    print("Done!")
    print("Minimum at: %f, %f" % (x, y))
    return x, y


def main():
    Grid = np.mgrid[0:100, 0:100]
    X = Grid[0]
    Y = Grid[1]

    x_g = X[78][0]
    y_g = Y[0][93]

    obs = np.array([[random.randint(0, 70), random.randint(0, 85)]])

    print(obs.shape)

    for i in range(0, 14):
        x_o = random.randint(0, 100)
        y_o = random.randint(0, 100)
        obs = np.append(obs, [[x_o, y_o]], axis=0)

    animGradDesc(30, 30, x_g, y_g, obs, 2)
    del X, Y  # free memory


if __name__ == "__main__":
    main()
