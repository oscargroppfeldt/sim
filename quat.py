import numpy as np


class Quat:
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
        return Quat(new_a, new_b, new_c, new_d)

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
        return Quat(new_a, new_b, new_c, new_d)

    def inv(self):
        norm = np.sqrt(self.a*self.a + self.b*self.b + self.c*self.c + self.d*self.d)
        temp_a = self.a/norm
        temp_b = -self.b/norm
        temp_c = -self.c/norm
        temp_d = -self.d/norm
        return Quat(temp_a, temp_b, temp_c, temp_d)

    def toEuler(self):
        temp_yaw = np.arctan2(2*(self.c*self.d+ self.a*self.b),
                        self.a*self.a - self.b*self.b - self.c*self.c + self.d*self.d)
        temp_pitch = np.arcsin(-2*(self.b*self.d - self.a*self.c))
        temp_roll = np.arctan2(2*(self.b*self.c + self.a*self.d), 
                        self.a*self.a + self.b*self.b - self.c*self.c - self.d*self.d)

        return [temp_yaw, temp_pitch, temp_roll]

    def Slerp(self, q_end, step=10):
        res_lst = []
        for t in range(step+1):
            newQuat = Quat(0,0,0,0)
            newQuat = self.inv()*q_end
            QuatPow = self*newQuat.qexp(t/step)
            res_lst.append(QuatPow)

        return res_lst

    def exp(self, pot):
        vec_size = np.sqrt(self.b*self.b + self.c*self.c + self.d*self.d)
        temp_a = np.exp(self.a)*np.cos(vec_size)
        temp_b = np.exp(self.a)*self.b*np.sin(vec_size)/vec_size
        temp_c = np.exp(self.a)*self.c*np.sin(vec_size)/vec_size
        temp_d = np.exp(self.a)*self.d*np.sin(vec_size)/vec_size

        return Quat(temp_a, temp_b, temp_c, temp_d)

    def qlog(self):
        norm = np.sqrt(self.a*self.a+self.b*self.b+self.c*self.c+self.d*self.d)
        vec_size = np.sqrt(self.b*self.b + self.c*self.c + self.d*self.d)
        temp_a = np.log(norm)
        factor = np.arccos(self.a/norm)/vec_size
        temp_b = self.b*factor
        temp_c = self.c*factor
        temp_d = self.d*factor

        return Quat(temp_a, temp_b, temp_c, temp_d)

    
    def pow(self, power):
        self = self.exp(power*self.qlog())
        return self

    def qexp(self, power):
        norm = np.sqrt(self.a*self.a+self.b*self.b+self.c*self.c+self.d*self.d)
        angle = np.arccos(self.a/norm)

        temp_a = np.cos(angle*power)*norm**power

        vec_factor = np.sin(angle*power)*norm**power

        temp_b = self.b*vec_factor
        temp_c = self.c*vec_factor
        temp_d = self.d*vec_factor

        return Quat(temp_a, temp_b, temp_c, temp_d)

   
    def pow(self, power):
        self = self.exp(power*self.qlog())
        return self

    def vecNorm(self):
        return np.sqrt(self.b*self.b + self.c*self.c + self.d*self.d)

    def normVal(self):
        return np.sqrt(self.a*self.a+self.b*self.b+self.c*self.c+self.d*self.d)
    
    def slerpSteps(self, q_end):
    # Needs calibration

        dot = self.a*q_end.a + self.b*q_end.b + self.c*q_end.c + self.d*q_end.d

        ang = np.arccos(dot)

        steps = ang/np.pi * 10

        print(ang, steps)
        return int(steps)

    def qexp(self, power):
        norm = self.normVal()
        angle = np.arccos(self.a/norm)
        temp_a = np.cos(angle*power)*norm**power
        
        vec_factor = np.sin(angle*power)*norm**power

        temp_b = self.b*vec_factor
        temp_c = self.c*vec_factor
        temp_d = self.d*vec_factor
        q = Quat(temp_a, temp_b, temp_c, temp_d)
        return q


    def newSlerp(self, q_end, per):
        dot = self.a*q_end.a + self.b*q_end.b + self.c*q_end.c + self.d*q_end.d

        print(dot)

        theta = np.arccos(dot)*per
        
        q_res = Quat(q_end.a - self.a*dot, q_end.b - self.b*dot, q_end.c - self.c*dot, q_end.d - self.d*dot)

        q_res.norm()

        return(Quat(self.a*np.cos(theta),self.b*np.cos(theta),self.c*np.cos(theta),self.d*np.cos(theta)) + 
                    Quat(q_res.a*np.sin(theta), q_res.b*np.sin(theta), q_res.c*np.sin(theta), q_res.d*np.sin(theta)))




    def __str__(self):
        return str([self.a, [self.b, self.c, self.d]])


def main():

    q1 = Quat(0,1,0,0)
    q2 = Quat(0,0,1,0)
    sphereical = q1.Slerp(q2)
    print(type(sphereical))
    for element in sphereical:
        print(type(element))
    return 0


if __name__ == "__main__":
    main()
