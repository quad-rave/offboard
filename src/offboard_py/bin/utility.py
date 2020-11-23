class TwoWayDict(object):
    def __init__(self):
        self.key_to_value = {}
        self.value_to_key = {}
    
    def add(self, key, value):
        self.key_to_value[key] = value
        self.value_to_key[value] = key

    def get_value(key):
        return self.key_to_value[key]
    def get_key(value):
        return self.value_to_key[value]