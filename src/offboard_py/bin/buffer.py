from std_msgs.msg import String
from mavros import command
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3

class DataBuffer():
    def __init__(self):
        self.list = []

    def to_string(self):
        return ''.join([str(elem)+"," for elem in self.list]).strip(",")     
    def from_string(value):
        data_buffer = DataBuffer()
        data_buffer.list = value.split(",")
        return data_buffer

    def write_int(self, value):
        self.list.append(str(value))
    def read_int(self):
        return int(self.list.pop())

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

