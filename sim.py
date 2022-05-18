import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import colors
from matplotlib.patches import Rectangle
import random


SIZE = 80
VIEW_SIZE = 50

def plot(sy, sp, sr, phi, theta, gimQ, camQ, sysQ, aimQ):
    sy = sy * np.pi/180
    sp = sp * np.pi/180
    sr = sr * np.pi/180
    phi = phi * np.pi/180
    theta = theta * np.pi/180
    Raim = invRotMatrix(sy, sp, sr, phi, theta)
    Rsys = normRotMatrix(sy, sp, sr)
    Rsysinv = sysInvMatrix(sy, sp, sr)
    # Figure
    fig = plt.figure(figsize=(9,9))
    ax = fig.gca(projection='3d', xlim=(-1 * SIZE, SIZE), ylim=(-1 * SIZE, SIZE),
    zlim=(-SIZE,SIZE), autoscale_on = False, aspect = 'auto')

    # Hide grid lines
    ax.grid(False)

    # Hide axes ticks
    #ax.set_xticks([])
    #ax.set_yticks([])
    ax.set_zticks([])

    # Plot plane
    point = np.array([[0], [0], [-VIEW_SIZE]])
    point_r = rotatePoint(point, Rsysinv)
    normal = np.array([[0], [0], [1]])
    normal_r = rotatePoint(normal, Rsysinv)
    p = np.array([point_r.item(0), point_r.item(1), point_r.item(2)])
    n = np.array([normal_r.item(0), normal_r.item(1), normal_r.item(2)])
    d = -p.dot(n)
    xx, yy = np.meshgrid(range(-SIZE*2,SIZE*2+15,15), range(-SIZE*2,SIZE*2+15,15),
    sparse = True)
    colors_list = np.empty([len(xx[0]), len(yy)], dtype=list)
    rownum = 0
    colnum = 0
    for row in yy:
        for col in xx[0]:
            alpha = 1/(1*(1+np.power(np.e, (-2*np.sqrt(row[0]**2 + col**2) - 100)/80)))
            colors_list[rownum, colnum] = [0.1, 0.3, 0.9, 1 - alpha]
            colnum += 1
        rownum += 1
        colnum = 0

    z = (-n[0] * xx - n[1] * yy - d) * 1. /n[2]
    ax.plot_surface(xx, yy, z, facecolors=colors_list)

    # Plot camera sphere
    u, v = np.mgrid[0:2*np.pi:20j, -np.pi/2:np.pi/2:20j]
    x = (VIEW_SIZE)*np.cos(u)*np.sin(v)
    y = (VIEW_SIZE)*np.sin(u)*np.sin(v)
    z = -(VIEW_SIZE)*np.cos(v)
    ax.plot_wireframe(x, y, z, color=[0.2, 0.4, 0.2, 0.2])

    # Plot drone
    p1 = np.array([[-10], [-10], [0]])
    p2 = np.array([[10], [-10], [0]])
    p3 = np.array([[0], [20], [0]])
    p1r = p1 #Rsys.dot(p1)
    p2r = p2 #Rsys.dot(p2)
    p3r = p3 #Rsys.dot(p3)

    dx = [p1r.item(0),p2r.item(0),p3r.item(0)]
    dy = [p1r.item(1),p2r.item(1),p3r.item(1)]
    dz = [p1r.item(2),p2r.item(2),p3r.item(2)]
    verts1 = [list(zip(dx,dy,dz))]
    coll1 = Poly3DCollection(verts1)
    coll1.set_color(colors.rgb2hex([1, 0.5, 0.2]))
    coll1.set_edgecolor('k')

    p4 = np.array([[0], [-10], [0]])
    p5 = np.array([[0], [-10], [-5]])
    p6 = np.array([[0], [20], [0]])

    p4r = p4 #rotatePoint(p4, Rsys)
    p5r = p5 #rotatePoint(p5, Rsys)
    p6r = p6 #rotatePoint(p6, Rsys)

    dx2 = [p4r.item(0),p5r.item(0),p6r.item(0)]
    dy2 = [p4r.item(1),p5r.item(1),p6r.item(1)]
    dz2 = [p4r.item(2),p5r.item(2),p6r.item(2)]
    verts2 = [list(zip(dx2,dy2,dz2))]
    coll2 = Poly3DCollection(verts2)
    coll2.set_color(colors.rgb2hex([0.2, 1, 0.5]))
    coll2.set_edgecolor('k')
    ax.add_collection3d(coll2)
    ax.add_collection3d(coll1)

    # Plot aim

    aim = np.array([[0], [1], [0]])
    aim_final = Raim.dot(aim)
    ax.scatter(VIEW_SIZE*aim_final.item(0), VIEW_SIZE*aim_final.item(1), VIEW_SIZE*aim_final.item(2), marker = 'x')

    # Plot aim vector from quaternion

    
    lst = []
    gimQ.norm()
    camQ.norm()
    steps = 15
    for t in np.linspace(0,1,steps):
        lst.append(gimQ.newSlerp(camQ, t))

    lst.append(camQ)
    lst.insert(0, gimQ)

    for element in lst:
        vec = [element.b,element.c,element.d]
        ax.quiver(0,0,0,
        VIEW_SIZE*vec[0], VIEW_SIZE*vec[1], VIEW_SIZE*vec[2], color='r')

    newQ = gimQ * camQ
    sysQ.norm()
    steps = 15

    for t in np.linspace(0,1,steps):
        lst.append(newQ.newSlerp(sysQ, t))

    lst.append(sysQ)
    lst.insert(0, newQ)

    for element in lst:
        vec = [element.b,element.c,element.d]
        ax.quiver(0,0,0,
        VIEW_SIZE*vec[0], VIEW_SIZE*vec[1], VIEW_SIZE*vec[2], color='r')


    plt.show()
    

def sysInvMatrix(y, p, r):
    a = np.cos(y)
    b = np.sin(y)
    c = np.cos(p)
    d = np.sin(p)
    e = np.cos(r)
    f = np.sin(r)

    ry = np.array([
        [a, -b, 0],
        [b, a, 0],
        [0, 0, 1]
    ])

    rp = np.array([
        [1, 0, 0],
        [0, c, d],
        [0, -d, c]
    ])

    rr = np.array([
        [e, 0, -f],
        [0, 1, 0],
        [f, 0, e]
    ])
    return rr.dot(rp.dot(ry))

def normRotMatrix(y, p, r):
    a = np.cos(y)
    b = np.sin(y)
    c = np.cos(p)
    d = np.sin(p)
    e = np.cos(r)
    f = np.sin(r)

    ry = np.array([
        [a, b, 0],
        [-b, a, 0],
        [0, 0, 1]
    ])

    rp = np.array([
        [1, 0, 0],
        [0, c, -d],
        [0, d, c]
    ])

    rr = np.array([
        [e, 0, f],
        [0, 1, 0],
        [-f, 0, e]
    ])
    return ry.dot(rp.dot(rr))

def invRotMatrix(y, p, r, phi, theta):
    a = np.cos(y)
    b = np.sin(y)
    c = np.cos(p)
    d = np.sin(p)
    e = np.cos(r)
    f = np.sin(r)
    h = np.cos(phi)
    j = np.sin(phi)
    k = np.cos(theta)
    l = np.sin(theta)

    return np.array([
        [h*(e*a + b*d*f) - j*(a*d*f - e*b), k*(h*(a*d*f - e*b) + j*(e*a + b*d*f)) - c*f*l, -l*(h*(a*d*f - e*b) + j*(e*a + b*d*f)) - c*f*k],
        [b*c*h - a*c*j, k*(a*c*h + b*c*j) + d*l, d*k - l*(a*c*h + b*c*j)],
        [h*(a*f - e*b*d) - j*(-e*a*d - b*f), k*(h*(-e*a*d - b*f) + j*(a*f - e*b*d)) + e*c*l, e*c*k - l*(h*(-e*a*d - b*f) + j*(a*f - e*b*d))]
        ])

def planeNormal(rotMatrix):
    return rotMatrix.dot(np.array([[0], [0], [1]]))

def ang2sph(theta, phi):
    return np.array([[np.cos(theta)*np.sin(phi)], [np.cos(theta)*np.cos(phi)], [np.sin(theta)]])

def rotatePoint(p, rotMatrix):
    return rotMatrix.dot(p)

def sysNormal(rotMatrix):
    return rotMatrix.dot(np.array([[0], [0], [1]]))

def get_cropping_point(q, wFOV, hFOV, imgW, imgH):
    theta = np.arctan2(q.b, q.c)
    phi = np.arcsin(-q.d)
    scaleH = 1 + (1/(hFOV - (np.pi / 2)))*(phi - hFOV)
    scaleW = 1 + (1/(wFOV - (np.pi / 2)))*(phi - wFOV)
    return (imgW/2 + (imgW/2)*np.sin(theta)*(scaleW), imgH/2 - (imgH/2)*np.cos(theta)*(scaleH))

def test_gimbal_correction(gimbal_y, gimbal_p, aim_y, aim_p, sys_y, sys_p, sys_r):
    cam_y = 0
    cam_p = np.pi / 2

    oriQ = Quat(0, 0, 1, 0)

    camQ = Quat()
    camQ.fromEuler(cam_y, cam_p, 0)
    camQ.con()
    gimQ = Quat()
    gimQ.fromEuler(gimbal_y, gimbal_p, 0)
    gimQ.con()
    sysQ = Quat()
    sysQ.fromEuler(sys_y, sys_p, sys_r)
    sysQ.con()
    aimQ = Quat()
    aimQ.fromEuler(aim_y, aim_p, 0)
    
    # Might need to normalize??

    rotQ = camQ * gimQ * sysQ * aimQ

    finQ = rotQ * oriQ
    rotQ.con()
    finQ = finQ * rotQ

    finQ *= 1 / (finQ.vecNorm())
    crop = get_cropping_point(finQ, 0.8, 0.6, 640, 480)
    print("Gimbal yaw, pitch:       ", gimbal_y, gimbal_p)
    print("Aim yaw, pitch           ", aim_y, aim_p)
    print("System yaw, pitch, roll: ", sys_y, sys_p, sys_r)
    print("Cropping point x, y:     ", crop[0], crop[1])


class Quat:
    def __init__(self, a = 1, b = 0, c = 0, d = 0):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.vec = [b,c,d]

    def fromEuler(self, yaw, pitch, roll):
        cy = np.cos(yaw/2)
        sy = np.sin(yaw/2)
        cp = np.cos(pitch/2)
        sp = np.sin(pitch/2)
        cr = np.cos(roll/2)
        sr = np.sin(roll/2)
        
        self.a = cy*cp*cr + sy*sp*sr
        self.b = cy*sp*cr + sy*cp*sr
        self.c = cy*cp*sr - sy*sp*cr
        self.d = cy*sp*sr - sy*cp*cr
    
    def __mul__(self, other):
        """
        w*q.w - x*q.x - y*q.y - z*q.z,
        x*q.w + w*q.x - z*q.y + y*q.z,
        y*q.w + z*q.x + w*q.y - x*q.z,
        z*q.w - y*q.x + x*q.y + w*q.z
        """
        new_a = self.a*other.a - self.b*other.b - self.c*other.c - self.d*other.d
        new_b = self.b*other.a + self.a*other.b - self.d*other.c + self.c*other.d
        new_c = self.c*other.a + self.d*other.b + self.a*other.c - self.b*other.d
        new_d = self.d*other.a - self.c*other.b + self.b*other.c + self.a*other.d
        return Quat(new_a, new_b, new_c, new_d)

    def __rmul__(self, other):
        self.a = self.a*other
        self.b = self.b*other
        self.c = self.c*other
        self.d = self.d*other
        return self
    
    def __imul__(self, other):
        return self.__rmul__(other)

    def __add__(self, other):
        new_a = self.a + other.a
        new_b = self.b + other.b
        new_c = self.c + other.c
        new_d = self.d + other.d
        return Quat(new_a, new_b, new_c, new_d)

    def norm(self):
        norm = np.sqrt(self.a*self.a + self.b*self.b + self.c*self.c+ self.d*self.d)
        self.a = self.a/norm
        self.b = self.b/norm
        self.c = self.c/norm
        self.d = self.d/norm

    def con(self):
        self.b = -self.b
        self.c = -self.c
        self.d = -self.d

    def inv(self):
        norm = np.sqrt(self.a*self.a + self.b*self.b + self.c*self.c + self.d*self.d)
        temp_a = self.a/norm
        temp_b = -self.b/norm
        temp_c = -self.c/norm
        temp_d = -self.d/norm
        return Quat(temp_a, temp_b, temp_c, temp_d)

    def copy(self):
        return Quat(self.a, self.b, self.c, self.d)

    def toEuler(self):
        temp_yaw = np.arctan2(2*(self.c*self.d+ self.a*self.b),
                        self.a*self.a - self.b*self.b - self.c*self.c + self.d*self.d)
        temp_pitch = np.arcsin(-2*(self.b*self.d - self.a*self.c))
        temp_roll = np.arctan2(2*(self.b*self.c + self.a*self.d), 
                        self.a*self.a + self.b*self.b - self.c*self.c - self.d*self.d)

        return [temp_yaw, temp_pitch, temp_roll]

    def Slerp(self, q_end, t, step):
        q0 = self.copy()
        q0_inv = q0.inv()
        q1 = q_end
        q_pow = q1*q0_inv
        q_temp = q_pow.qexp(t/step)
        q_res = q_temp*q0
        q_res.norm()
        return q_res

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
    DEG2RAD = 0.01745329251
    while(1):
        gimbal_y, gimbal_p = input("Enter gimbal **yaw pitch**").split()
        aim_y, aim_p = input("Enter aim **yaw pitch**").split()
        system_y, system_p, system_r = input("Enter system **yaw pitch roll**").split()
        gimbal_y = float(gimbal_y)
        gimbal_p = float(gimbal_p)
        aim_y = float(aim_y)
        aim_p = float(aim_p)
        system_y = float(system_y)
        system_p = float(system_p)
        system_r = float(system_r)

        gimbal_y *= DEG2RAD
        gimbal_p *= DEG2RAD
        aim_y *= DEG2RAD
        aim_p *= DEG2RAD
        system_y *= DEG2RAD
        system_p *= DEG2RAD
        system_r *= DEG2RAD

        test_gimbal_correction(gimbal_y, gimbal_p, aim_y, aim_p, system_y, system_p, system_r)
