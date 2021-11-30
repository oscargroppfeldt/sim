import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import colors
from matplotlib.patches import Rectangle
import random


SIZE = 80
VIEW_SIZE = 50

def plot(sy, sp, sr, phi, theta):
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
    aimQ = euler2q(phi, theta, 0)
    aimQd = euler2q(phi + 0.2, theta, 0)
    sysQ = euler2q(sy, sp, sr)
    oriQ = [0,0,1,0]
    sysQinv = invQ(sysQ)
    aimQinv = invQ(aimQ)
    sysQcon = conQ(sysQ)
    aimQcon = conQ(aimQ)
    aimQdcon = conQ(aimQd)
    finQ = qMult(qMult(qMult(qMult(sysQcon, aimQ), oriQ), aimQcon), sysQ)
    finQd = qMult(qMult(qMult(qMult(sysQcon, aimQd), oriQ), aimQdcon), sysQ)
    rotQ = qMult(sysQinv, aimQ)
    

    normF = np.sqrt(finQ[1]*finQ[1] + finQ[2]*finQ[2] + finQ[3]*finQ[3])
    normR = np.sqrt(rotQ[1]*rotQ[1] + rotQ[2]*rotQ[2] + rotQ[3]*rotQ[3])

    print("final: ", finQ)
    print("rot: ", rotQ)


    q_start = Quat(0,1,0,0)
    q_end = Quat(0,-1,1,0)
    lst = []
    q_start.norm()
    q_end.norm()
    steps = q_start.slerpSteps(q_end)
    for t in range(steps):
        lst.append(q_start.Slerp(q_end,t ,steps))


    for element in lst:
        vec = [element.b,element.c,element.d]
        ax.quiver(0,0,0,
        VIEW_SIZE*vec[0], VIEW_SIZE*vec[1], VIEW_SIZE*vec[2], color='r')
        print(vec, element.a)


    # Gränsvärden: när q1q2 + q0q3 beräknas yaw och roll annorlunda. Detta syns nedan:
    # Detta är fel, det blir annorlunda för vårt system. Se länken nedan för hur de kom fram till gränsvärdena.
    pole = rotQ[1]*rotQ[1] + rotQ[2]*rotQ[2]
    sgn = rotQ[0]*rotQ[1] + rotQ[2]*rotQ[3]
    print(pole)
    if(abs(pole - 0.5) < 0.08):
        print("Degenerate")
        # North pole
        if(sgn > 0):
            yaw = 2*np.arctan2(rotQ[2], rotQ[0])
            roll = 0
        # South pole
        elif(sgn < 0):        
            yaw = 2*np.arctan2(rotQ[2], rotQ[0])
            roll = 0
    # Mer info: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
    # Pitch och roll är ombytta pga vår X-Y-invertering
    else:
        roll = np.arcsin(2*(rotQ[0]*rotQ[2] - rotQ[3]*rotQ[1]))
        yaw = -np.arctan2(2*(rotQ[0]*rotQ[3] + rotQ[1]*rotQ[2]), 1 - 2*(rotQ[2]*rotQ[2] + rotQ[3]*rotQ[3]))
    
    pitch = np.arctan2(2*(rotQ[0]*rotQ[1] + rotQ[2]*rotQ[3]), 1 - 2*(rotQ[1]*rotQ[1] + rotQ[2]*rotQ[2]))
    
    print("roll: ", np.rad2deg(roll), ", pitch: ", np.rad2deg(pitch), ", yaw: ", np.rad2deg(yaw))
    """
    ax.quiver(0, 0, 0,
    VIEW_SIZE*(finQ[1]/normF), VIEW_SIZE*(finQ[2]/normF), VIEW_SIZE*(finQ[3]/normF), color="g")
    ax.quiver(0, 0, 0,
    VIEW_SIZE*(finQd[1]/normF), VIEW_SIZE*(finQd[2]/normF), VIEW_SIZE*(finQd[3]/normF), color="b")
    ax.quiver(0, 0, 0,
    2*VIEW_SIZE*(rotQ[1]/normR), 2*VIEW_SIZE*(rotQ[2]/normR), 2*VIEW_SIZE*(rotQ[3]/normR), color="r")
    ax.quiver(0, 0, 0,
    -2*VIEW_SIZE*(rotQ[1]/normR), -2*VIEW_SIZE*(rotQ[2]/normR), -2*VIEW_SIZE*(rotQ[3]/normR), color="r")
    """
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

def euler2q(y, p, r):
    cy = np.cos(y/2)
    sy = np.sin(y/2)
    cp = np.cos(p/2)
    sp = np.sin(p/2)
    cr = np.cos(r/2)
    sr = np.sin(r/2)
    qw = cy*cp*cr + sy*sp*sr
    qx = cy*sp*cr + sy*cp*sr
    qy = cy*cp*sr - sy*sp*cr
    qz = cy*sp*sr - sy*cp*cr
    return [qw, qx, qy, qz]

def invQ(q):
    norm = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]
    return [q[0]/norm, -q[1]/norm, -q[2]/norm, -q[3]/norm]

def conQ(q):
    return [q[0], -q[1], -q[2], -q[3]]

def qMult(q1, q2):
    return [
        q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
        q1[1]*q2[0] + q1[0]*q2[1] - q1[3]*q2[2] + q1[2]*q2[3],
        q1[2]*q2[0] + q1[3]*q2[1] + q1[0]*q2[2] - q1[1]*q2[3],
        q1[3]*q2[0] - q1[2]*q2[1] + q1[1]*q2[2] + q1[0]*q2[3]
    ]

def test_crop(phi, theta):
    phi = np.deg2rad(phi)
    theta = np.deg2rad(theta)
    a = np.sin(phi) * np.cos(theta)
    b = np.cos(phi) * np.cos(theta)
    c = np.sin(theta)

    # Med skalning 1-t
    a2b2 = a*a + b*b
    if(c == np.sqrt(a2b2) or c == -np.sqrt(a2b2)):
        t = (np.sqrt(a2b2) + 2*a2b2)/(2*a2b2)
    else:
        t = (np.sqrt(a2b2) + a2b2 - c*c - c)/(a2b2 - c*c)

    # Med skalning t
    if(c == np.sqrt(a2b2) or c == -np.sqrt(a2b2)):
        t = 1/(2*np.sqrt(a2b2))
    else:
        t = (c + np.sqrt(a2b2))/(a2b2 - c*c)

    print("a = ", a, "b = ", b, "c = ", c, "t = ", t)
    print("x = ", a*t, "\ny = ", -b*t)


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

    def norm(self):
        norm = np.sqrt(self.a*self.a + self.b*self.b + self.c*self.c+ self.d*self.d)
        self.a = self.a/norm
        self.b = self.b/norm
        self.c = self.c/norm
        self.d = self.d/norm

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

    def Slerp(self, q_end, t, step=10):
        q0 = self.copy()
        q0_inv = q0.inv()
        q1 = q_end
        q_pow = q0_inv*q1
        print(f"Pow norm: {q_pow.a*q_pow.a+q_pow.b*q_pow.b+q_pow.c*q_pow.c+q_pow.d*q_pow.d}")    
        q_temp = q_pow.qexp(t/step)
        q_res = q0*q_temp
        print(f"After mult give norm {q_res.a*q_res.a+q_res.b*q_res.b+q_res.c*q_res.c+q_res.d*q_res.d}")
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
    
    def slerpSteps(this, q_end):
    # Needs calibration

        q_used = q_end.copy().inv()
        arg = (this*q_used).vecNorm()
        theta = 2*np.arcsin(arg)
        #print(f"Arg: {arg}, theta: {theta}")
        steps = 10*theta/np.pi
        #print(f"Suggested num. of steps: {steps}")
        return int(steps)

    def qexp(self, power):
        norm = np.sqrt(self.a*self.a+self.b*self.b+self.c*self.c+self.d*self.d)
        angle = np.arccos(self.a/norm)
        print(f"qexp calculated a norm of {norm} with the argument {angle}")
        temp_a = np.cos(angle*power)*norm**power

        vec_factor = np.sin(angle*power)*norm**power

        temp_b = self.b*vec_factor
        temp_c = self.c*vec_factor
        temp_d = self.d*vec_factor
        q = Quat(temp_a, temp_b, temp_c, temp_d)
        q_norm = q.a*q.a+q.b*q.b+q.c*q.c+q.d*q.d
        print(f"New norm is now: {q_norm}")
        return q

    def __str__(self):
        return str([self.a, [self.b, self.c, self.d]])




if __name__ == "__main__":
    plot(0,0,0,0,0)