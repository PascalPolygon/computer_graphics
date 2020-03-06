import numpy as np
import matplotlib.pyplot as plt
from vpython import*
scene = canvas()
# sphere()


def computeCost(X, Y, x_g, y_g):
    return np.sqrt(np.square(X-x_g) + np.square(Y-y_g))


def computeGradient(X, Y, x_g, y_g):
    C = computeCost(X, Y, x_g, y_g)
    dx = (X-x_g)/C
    dy = (Y-y_g)/C
    return dx, dy


def euclDist(x1, y1, x2, y2):
    return np.sqrt(np.square(x2-x1) + np.square(y2-y1))


def gradWithObst(X, Y, x_g, y_g, Obs, R):
    # dx = np.zeros([100, 100])
    # dy = np.zeros([100, 100])
    d_g = euclDist(X, Y, x_g, y_g)  # euclidian distance to goal point
    dx = (X-x_g)/d_g
    dy = (Y-y_g)/d_g
    for i in range(0, Obs.shape[0]):
        x_o = Obs[i][0]
        y_o = Obs[i][1]
        d_o = euclDist(X, Y, x_o, y_o)  # euclidian distance to obstacle
        dx += 0 if d_o > R else -((X-x_o)/d_o)
        dy += 0 if d_o > R else -((Y-y_o)/d_o)
        # dy = 0+dy if d_o > R else ((Y-y_g)/d_g) - ((Y-y_o)/d_o)
        # dx += ((X-x_g)/d_g) - ((X-x_o)/d_o)
        # dy += ((Y-y_g)/d_g) - ((Y-y_o)/d_o)
        # print("Opencountry")
    return dx, dy


def gradDescent(x, y, x_g, y_g):
    C = computeCost(x, y, x_g, y_g)
    print("Initial Cost: %f" % C)
    lmbda = 0.1
    i = 0
    while C > 0.02:
        dx, dy = computeGradient(x, y, x_g, y_g)
        x = x - lmbda*dx
        y = y - lmbda*dy
        C = computeCost(x, y, x_g, y_g)
        print("Epoch %d, cost: %f" % (i, C))
        i += 1
    print("Done!")
    print("Minimum at: %d, %d" % (x, y))
    return x, y


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

    print("Initial Cost: %f" % C_total)
    lmbda = 0.1
    i = 0
    # 0,0 in animation is not a corner but the center -50 corrects that
    ball = sphere(pos=vector(x-50, 10, y-50), radius=1,
                  color=color.red, make_trail=True)
    floor = box(pos=vector(0, 0, 0), size=vector(
        100, 0.5, 100), color=color.orange)
    ball.velocity = vector(0, 0, 0)

    for i in range(0, obs.shape[0]):
        obstacle = cylinder(pos=vector(
            obs[i][0]-50, 0, obs[i][0]-50), axis=vector(0, 20, 0), radius=R)

    print("Finding minimum...")
    while C_total > 0.02:
        rate(70)
        # dx, dy = computeGradient(x, y, x_g, y_g)
        dx, dy = gradWithObst(x, y, x_g, y_g, obs, R)
        # ball.velocity = vector(dx, dy, 0)
        # ball.pos = ball.pos-ball.velocity*lmbda
        x = x - lmbda*dx
        y = y - lmbda*dy
        ball.pos = vector(x-50, 10, y-50)
        print(ball.pos)

        C = computeCost(x, y, x_g, y_g)
        C_obs = costWithObstables(x, y, obs, R)
        C_total = C + C_obs

        # print("Epoch %d, cost: %f" % (i, C))
        i += 1
    print("Done!")
    print("Minimum at: %d, %d" % (x, y))
    return x, y


def main():
    Grid = np.mgrid[0:100, 0:100]
    X = Grid[0]
    Y = Grid[1]
    # Arb goal at 78, 93
    print(X[78][0])
    print(Y[0][93])
    x_g = X[78][0]
    y_g = Y[0][93]
    # obstacles
    obs = np.array([52, 53])
    obs = np.reshape(obs, (1, 2))

    # C = computeCost(X, Y, x_g, y_g)

    # plt.contourf(X, Y, C, cmap='jet')
    # plt.colorbar()
    # plt.title('Cost Field')
    # plt.show()

    # dx, dy = computeGradient(X, Y, x_g, y_g)
    # dx = -dx
    # dy = -dy
    # fig, ax = plt.subplots()
    # ax.contourf(X, Y, C, cmap='jet')
    # # fig.colorbar()
    # ax.quiver(X, Y, dx, dy)
    # ax.set(aspect=1, title='Gradient vector field')

    # plt.show()

    # gradDescent(50, 50, x_g, y_g)
    animGradDesc(30, 30, x_g, y_g, obs, 5)


if __name__ == "__main__":
    main()
