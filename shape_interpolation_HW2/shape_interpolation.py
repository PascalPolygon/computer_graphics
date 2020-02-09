import numpy as np

_a = np.array([1, 2, 3, 4, 5, 6])
print(_a)
a_reshaped = np.reshape(_a, (2, 3))
print(a_reshaped)

# a = np.array([[1, 2, 3], [4, 5, 6]])
# np.reshape(a, 6)
# array([1, 2, 3, 4, 5, 6])
# np.reshape(a, 6, order='F')
# array([1, 4, 2, 5, 3, 6])
