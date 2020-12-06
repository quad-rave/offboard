from std_msgs.msg import String
from mavros import command
import numpy as np

class DataBuffer():
    def __init__(self):
        self.list = []

    def to_string(self):
        return ''.join([str(elem)+"," for elem in self.list]).strip(",")     

    @staticmethod
    def from_string(value):
        data_buffer = DataBuffer()
        data_buffer.list = value.split(",")
        return data_buffer

    def write_int(self, value):
        self.list.append(str(value))
    def read_int(self):
        return int(self.list.pop(0))

    def write_float(self, value):
        self.list.append(str(value))
        
    def read_float(self):
        return float(self.list.pop(0))

    def write_vector3(self, vector3):
        x = vector3[0]
        y = vector3[1]
        z = vector3[2]
        self.write_float(x)
        self.write_float(y)
        self.write_float(z)

    def read_vector3(self):
        x = self.read_float()
        y = self.read_float()
        z = self.read_float()
        return np.array([x,y,z])

