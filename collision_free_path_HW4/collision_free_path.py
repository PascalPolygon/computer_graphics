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


def animGradDesc(x, y, x_g, y_g):
    C = computeCost(x, y, x_g, y_g)
    print("Initial Cost: %f" % C)
    lmbda = 0.1
    i = 0

    # 0,0 in animation is not a corner but the center -50 corrects that
    ball = sphere(pos=vector(x-50, 10, y-50), radius=1, color=color.red)
    floor = box(pos=vector(0, 0, 0), size=vector(
        100, 0.5, 100), color=color.orange)
    ball.velocity = vector(0, 0, 0)
    print("Finding minimum...")
    while C > 0.02:
        rate(10)
        dx, dy = computeGradient(x, y, x_g, y_g)
        # ball.velocity = vector(dx, dy, 0)
        # ball.pos = ball.pos-ball.velocity*lmbda
        x = x - lmbda*dx
        y = y - lmbda*dy
        ball.pos = vector(x-50, 10, y-50)
        print(ball.pos)
        C = computeCost(x, y, x_g, y_g)
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
    animGradDesc(50, 50, x_g, y_g)


if __name__ == "__main__":
    main()
