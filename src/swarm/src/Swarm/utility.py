import numpy as np

class TwoWayDict(object):
    def __init__(self):
        self.key_to_value = {}
        self.value_to_key = {}
    
    def add(self, key, value):
        self.key_to_value[key] = value
        self.value_to_key[value] = key

    def get_value(self, key):
        return self.key_to_value[key]
    def get_key(self, value):
        return self.value_to_key[value]

def dist(a,b):
    return np.linalg.norm(a - b)


def vec(x,y,z):
    return np.array([x,y,z])