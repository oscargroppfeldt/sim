import numpy as np


class Quart:
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.vec = [b,c,d]
    
    def __mul__(self, other):
        new_a = self.a*other.a - self.b*other.b - self.c*other.c - self.d*other.d
        new_b = self.a*other.b + self.b*other.a + self.c*other.d - self.d*other.c
        new_c = self.a*other.c - self.b*other.d + self.c*other.a + self.d*other.b
        new_d = self.a*other.d + self.b*other.c - self.c*other.b + self.d*other.a
        return Quart(new_a, new_b, new_c, new_d)

    def __rmul__(self, other):
        self.a = self.a*other
        self.b = self.b*other
        self.c = self.c*other
        self.d = self.d*other
        return self

    def __add__(self, other):
        new_a = self.a + other.a
        new_b = self.b + other.b
        new_c = self.c + other.c
        new_d = self.d + other.d
        return Quart(new_a, new_b, new_c, new_d)

    def inv(self):
        norm = np.sqrt(self.a*self.a + self.b*self.b + self.c*self.c + self.d*self.d)
        self.a = self.a/norm
        self.b = -self.b/norm
        self.c = -self.c/norm
        self.d = -self.d/norm
        return self

    def toEuler(self):
        temp_yaw = np.arctan2(2*(self.c*self.d+ self.a*self.b),
                        self.a*self.a - self.b*self.b - self.c*self.c + self.d*self.d)
        temp_pitch = np.arcsin(-2*(self.b*self.d - self.a*self.c))
        temp_roll = np.arctan2(2*(self.b*self.c + self.a*self.d), 
                        self.a*self.a + self.b*self.b - self.c*self.c - self.d*self.d)

        return [temp_yaw, temp_pitch, temp_roll]

    def __str__(self):
        return str([self.a, [self.b, self.c, self.d]])




def main():
    
    return 0


if __name__ == "__main__":
    main()
