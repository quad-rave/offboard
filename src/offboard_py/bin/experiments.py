from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3

class DataBuffer():
    def __init__(self):
        self.data = ""
        self.list = []
        self.index = 0
        self.string = ""
    def to_string(self):
        self.string = ''.join([str(elem)+"," for elem in self.list]).strip(",")     
    def from_string(self, value):
        self.list = self.string.split(",")

    def write_float(self, value):
        self.list.append(str(value))
        
    def read_float(self):
        return float(self.list.pop())

    def write_vector3(self, vector3):
        x = vector3.x
        y = vector3.y
        z = vector3.z
        self.write_float(x)
        self.write_float(y)
        self.write_float(z)

    def read_vector3(self):
        x = self.read_float()
        y = self.read_float()
        z = self.read_float()
        return Vector3(x,y,z)


buffer = DataBuffer()

a = Vector3(1,2,3)
b = Vector3(3,2,1)

buffer.write_vector3(a)
buffer.write_vector3(b)
print(buffer.read_vector3())
print(buffer.read_vector3())
