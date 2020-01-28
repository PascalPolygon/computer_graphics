# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
# %%
import matplotlib.pyplot as plt  # matlab-like way of plotting
from scipy import misc  # scipy is image processing lib get miscelaneaou module
from scipy import ndimage  # importing ndimage so we can do multi-dimensional processing
import PIL  # Python image library
import sys
import glob
import imageio
import numpy as np
from IPython import get_ipython

# %%
# f = misc.face(); #face of a raccoon offered my misc module
#img = imageio.imread('imageio:astronaut.png')
# afroImg = imageio.imread('/home/pascal/computer_graphics/santa-fung-afro-007.jpg')
afroImg = imageio.imread('/home/pascal/Pictures/santa-fung-afro-007.jpg')
print(afroImg.shape)
#imageio.imwrite('face.png', f);


# %%
# TODO: create genereic homogenize function for any size matrix
def homogenize3(m):
    hom = np.array([
                   [m[0][0], m[1][0], m[2][0]],
                   [m[3][0], m[4][0], m[5][0]],
                   [0, 0, 1]
                   ])
    return hom


# %%
def estimateAffine(X, X_prime):
    M = np.array([
        [X[0][0], X[0][1], 1, 0, 0, 0],
        [0, 0, 0, X[0][0], X[0][1], 1],
        [X[1][0], X[1][1], 1, 0, 0, 0],
        [0, 0, 0, X[1][0], X[1][1], 1],
        [X[2][0], X[2][1], 1, 0, 0, 0],
        [0, 0, 0, X[2][0], X[2][1], 1]
    ])  # Source matrix represneted with 3 points homogenously (eq.7)
    # b = np.array([X_prime[0][0], X_prime[0][1], X_prime[1][0], X_prime[1][1], X_prime[2][0], X_prime[2][1]])
    b = np.array([[X_prime[0][0]],
                  [X_prime[0][1]],
                  [X_prime[1][0]],
                  [X_prime[1][1]],
                  [X_prime[2][0]],
                  [X_prime[2][1]]])  # Destination matrix as a column vector
    print(M)
    print(b)
    M_inv = np.linalg.inv(M)  # Computer inverse of M matrix
    M_id = np.dot(M, M_inv)  # M identity matrix
    print(M_id)  # Verifying that I get an identity matrix
    I = np.identity(6)
    print(np.identity(6))
    a = np.dot(M_inv, b)  # Compute transformation coefficients
    print(a)
    a = homogenize3(a)
    return a

    # Attempt to make sure that inverse result is accurate
    # by comparing M identity to perfect identiy matrix (currently not working)
    # if np.allclose(M,I,1,1) :
    #     print('Good identity')
    # else:
    #     print('Bad identity')


# %%
def getDstImageSize(X_prime):
    s = X_prime.shape
    print(s)
    x_s = X_prime[:, 0]  # All of first col (x's)
    y_s = X_prime[:, 1]  # All of second col (y's)
    x_min = np.amin(x_s)
    x_max = np.amax(x_s)
    y_min = np.amin(y_s)
    y_max = np.amax(y_s)
    n_rows = y_max - y_min  # compute min # of rows to contain dst image
    n_cols = x_max - x_min  # compute min # of cols to contain dst image
    return n_rows, n_cols


# %%
def isWithinBoundaries(x, y, nRows, nCols):
    # Return True if (x,y) is within the boundaries of matrix A(nRows,nCols)

    return (x >= 0 and x < nRows and y >= 0 and y < nCols)


# %%
def inverseMapping(A, X, X_prime, img):
    # Compute minimum size of destination image from destination triangle
    n_rows, n_cols = getDstImageSize(X_prime)
    dstImg = np.zeros((n_rows, n_cols, 3))
    print('Dest image shape: ')
    print(dstImg.shape)
    print('Inverse of transformation matrix: ')
    A_inv = np.linalg.inv(A)
    print(A_inv)
    print(img.shape)
    outputHmg = open('outputHmg.txt', 'w')
    # outputMtrx = open('outputMtrx.txt', 'w')
    i = 0
    srcImgRows = img.shape[0]
    srcImgCols = img.shape[1]

    for x_prime in range(0, n_rows):
        for y_prime in range(0, n_cols):
            dstMatrx = np.array([[x_prime],
                                 [y_prime],
                                 [1]
                                 ])
            # This matrix will be homogeneous
            srcHmgMtrx = np.dot(A_inv, dstMatrx)
            # get rid of the 1 and make it 2d vector
            srcMtrx = np.array([srcHmgMtrx[0][0], srcHmgMtrx[1][0]])
            if i < 100:
                print('\nHomogeneous:', file=outputHmg)
                print(srcHmgMtrx, file=outputHmg)
                print('Non Homogeneous form (x rounded): ', file=outputHmg)
                print(int(srcMtrx[0]), file=outputHmg)
                i += 1

        if isWithinBoundaries(srcMtrx[0], srcMtrx[1], srcImgRows, srcImgCols):
            dstImg[x_prime, y_prime, 0] = img[int(
                srcMtrx[0]), int(srcMtrx[1]), 0]
            dstImg[x_prime, y_prime, 1] = img[int(
                srcMtrx[0]), int(srcMtrx[1]), 1]
            dstImg[x_prime, y_prime, 2] = img[int(
                srcMtrx[0]), int(srcMtrx[1]), 2]
    outputHmg.close()
    # outputMtrx.close()
    dstImg = 255 - dstImg  # Why do I have to take its negative?
    return dstImg


# %%
plt.figure()
plt.imshow(afroImg)
# Source triangle
plt.scatter(200, 200, s=100, c='red', marker='o')
plt.scatter(1400, 200, s=100, c="red", marker="o")
plt.scatter(1000, 1000, s=100, c="red", marker="o")
X = np.array([[200, 200],
              [1400, 200],
              [1000, 1000]
              ])

# Destination triangle


class dstTri:
    class p1:
        x = 200
        y = 800

    class p2:
        x = 1200
        y = 400

    class p3:
        x = 900
        y = 1000

# class dstTri :
#     class p1 :
#         x = 600
#         y = 400
#     class p2 :
#         x = 1200
#         y = 400
#     class p3 :
#         x = 900
#         y = 1000
# class dstTri :
#     class p1 :
#         x = 400
#         y = 200
#     class p2 :
#         x = 1300
#         y = 800
#     class p3 :
#         x = 400
#         y = 800


plt.scatter(dstTri.p1.x, dstTri.p1.y, s=100, c='blue', marker='o')
plt.scatter(dstTri.p2.x, dstTri.p2.y, s=100, c="blue", marker="o")
plt.scatter(dstTri.p3.x, dstTri.p3.y, s=100, c="blue", marker="o")

X_prime = np.array([[dstTri.p1.x, dstTri.p1.y],
                    [dstTri.p2.x, dstTri.p2.y],
                    [dstTri.p3.x, dstTri.p3.y]])

plt.show()
A = estimateAffine(X, X_prime)
print('Transformation matrix: ')
print(A)
dstImg = inverseMapping(A, X, X_prime, afroImg)
# plt.figure(figsize = (10,10))
plt.figure()
plt.imshow(dstImg)


# %%


# %%
