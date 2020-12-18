from geometry_msgs.msg import TwistStamped

from geometry_msgs.msg import Vector3
import math
import numpy as np

from scipy.optimize import linear_sum_assignment


'''
>>> cost = np.array([[4, 1, 3], [2, 0, 5], [3, 2, 2]])
>>> from scipy.optimize import linear_sum_assignment
>>> row_ind, col_ind = linear_sum_assignment(cost)
>>> col_ind
array([1, 0, 2])
>>> cost[row_ind, col_ind].sum()
5
    p1  p2  p3 p4
0   e1  e2  e3 e4
1   e5  e6  e7 e8
2   e9  .   .   .
3   .   .
4
     p1  p2  p3
uav0 4   1   3 
uav1 2   0   5
uav2 3   2   2
'''

def vec(x,y):
    return np.array([x,y])

def dist(a,b):
    return np.linalg.norm(a - b)

cost = np.array([[4, 1, 3], [2, 0, 5], [3, 2, 2]])
row_ind, col_ind = linear_sum_assignment(cost)
print(row_ind, "\n----------")
print(col_ind)


'''
4 1
2 0
'''
cost = np.array([[4, 1], [2, 0]])
row_ind, col_ind = linear_sum_assignment(cost)
print(row_ind, "\n----------") # 0 to 1, 1 to 0
print(col_ind)

uav_poses = [vec(0.0,0.0), vec(0.0,1.0),vec(0.0,2.0)]
point_cloud = [vec(0.0,2.0), vec(0.0,1.0),vec(0.0,0.0)]
uav_count = 3
cost_matrix = np.zeros((uav_count, uav_count), dtype=np.float64)

for uav in range(uav_count):
    for point in range(uav_count):
        cost_matrix[uav, point] = dist(uav_poses[uav], point_cloud[point])
        

print("#########################################33")
row_ind, col_ind = linear_sum_assignment(cost_matrix)
print(row_ind, "\n----------")
print(col_ind)






'''
### row_ind, col_ind = linear_sum_assignment(cost_matrix) ??
# Author: Brian M. Clapper, G. Varoquaux, Lars Buitinck
# License: BSD

from numpy.testing import assert_array_equal
from pytest import raises as assert_raises

import numpy as np

from scipy.optimize import linear_sum_assignment
from scipy.sparse.sputils import matrix


def test_linear_sum_assignment():
    for cost_matrix, expected_cost in [
        # Square
        ([[400, 150, 400],
          [400, 450, 600],
          [300, 225, 300]],
         [150, 400, 300]
         ),

        # Rectangular variant
        ([[400, 150, 400, 1],
          [400, 450, 600, 2],
          [300, 225, 300, 3]],
         [150, 2, 300]),

        # Square
        ([[10, 10, 8],
          [9, 8, 1],
          [9, 7, 4]],
         [10, 1, 7]),

        # Rectangular variant
        ([[10, 10, 8, 11],
          [9, 8, 1, 1],
          [9, 7, 4, 10]],
         [10, 1, 4]),

        # n == 2, m == 0 matrix
        ([[], []],
         []),
    ]:
        cost_matrix = np.array(cost_matrix)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        assert_array_equal(row_ind, np.sort(row_ind))
        assert_array_equal(expected_cost, cost_matrix[row_ind, col_ind])

        cost_matrix = cost_matrix.T
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        assert_array_equal(row_ind, np.sort(row_ind))
        assert_array_equal(np.sort(expected_cost),
                           np.sort(cost_matrix[row_ind, col_ind]))


def test_linear_sum_assignment_input_validation():
    assert_raises(ValueError, linear_sum_assignment, [1, 2, 3])

    C = [[1, 2, 3], [4, 5, 6]]
    assert_array_equal(linear_sum_assignment(C),
                       linear_sum_assignment(np.asarray(C)))
    assert_array_equal(linear_sum_assignment(C),
                       linear_sum_assignment(matrix(C)))

    I = np.identity(3)
    assert_array_equal(linear_sum_assignment(I.astype(np.bool)),
                       linear_sum_assignment(I))
    assert_raises(ValueError, linear_sum_assignment, I.astype(str))

    I[0][0] = np.nan
    assert_raises(ValueError, linear_sum_assignment, I)

    I = np.identity(3)
    I[1][1] = np.inf
    assert_raises(ValueError, linear_sum_assignment, I)

'''