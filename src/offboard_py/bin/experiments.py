from geometry_msgs.msg import TwistStamped

from geometry_msgs.msg import Vector3
import math
import numpy as np

a = np.array([1,0,0])
b = np.array([0,2,0])
u = [1, 2, 3]


print(a + b)
b += u
print(b)
print(np.linalg.norm(a + b) * 2)

