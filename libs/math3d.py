import numpy as np

class vec3:
    def __init__(self, x, y, z) -> None:
        self.x, self.y, self.z = x, y, z

    def __add__(self, o):
        return vec3(self.x+o.x, self.y+o.y, self.z+o.z)
    
    def __sub__(self, o):
        return vec3(self.x-o.x, self.y-o.y, self.z-o.z)
    
    def __mul__(self, o):
        if isinstance(o, vec3):
            return self.x*o.x + self.y*o.y + self.z*o.z
        else:
            return vec3(self.x*o, self.y*o, self.z*o)
        
    def __truediv__(self, o):
        return vec3(self.x/o, self.y/o, self.z/o)
    
    def __str__(self):
        return f"({self.x}, {self.y}, {self.z})"

    def __repr__(self):
        return f"vec3({self.x}, {self.y}, {self.z})"
    
    def dot(self, o):
        return self.x * o.x + self.y * o.y + self.z * o.z

    def length(self):
        return np.sqrt(self.dot(self))
    
    @property
    def xy(self):
        return (self.x, self.y)

    @property
    def xyz(self):
        return (self.x, self.y, self.z)
    
class vec2:
    def __init__(self, x, y) -> None:
        self.x, self.y = x, y

    def __add__(self, o):
        return vec2(self.x+o.x, self.y+o.y)
    
    def __sub__(self, o):
        return vec2(self.x-o.x, self.y-o.y)
    
    def __mul__(self, o):
        if isinstance(o, vec2):
            return self.x*o.x + self.y*o.y
        else:
            return vec2(self.x*o, self.y*o)
        
    def __truediv__(self, o):
        return vec2(self.x/o, self.y/o)
    
    def __str__(self):
        return f"({self.x}, {self.y})"

    def __repr__(self):
        return f"vec2({self.x}, {self.y})"

    def dot(self, o):
        return self.x * o.x + self.y * o.y

    def length(self):
        return np.sqrt(self.dot(self))

    @property
    def xy(self):
        return (self.x, self.y)

def get_real_xyz(xyd: vec3, size: vec2):
    a = 49.5 * np.pi / 180
    b = 60.0 * np.pi / 180
    x, y, d = xyd.xyz
    h, w = size.xy           
    x = int(x) - int(w // 2)
    y = int(y) - int(h // 2)
    real_y = round(y * 2 * d * np.tan(a / 2) / h)
    real_x = round(x * 2 * d * np.tan(b / 2) / w)
    return vec3(real_x, real_y, d)