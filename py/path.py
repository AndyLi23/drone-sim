import math
import numpy as np

class Vector3():
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z
        
    def cpy(self):
        return Vector3(self.x, self.y, self.z)
    
    def scl(self, s):
        self.x *= s
        self.y *= s
        self.z *= s
        return self
    
    def dst2(self, p):
        return (self.x-p.x) * (self.x-p.x) + (self.y-p.y) * (self.y-p.y) + (self.z-p.z) * (self.z-p.z)
    
    def add(self, other):
        self.x += other.x
        self.y += other.y
        self.z += other.z
        return self
    
    def arr(self):
        return np.array([self.x, self.y, self.z], dtype=np.float64)
    
    def len(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    
    def __str__(self) -> str:
        return "<" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ">"
    

class CubicHermite3D():
    """ generated source for class CubicHermite3D """

    def __init__(self, pos0, pos1, vel0, vel1):
        """ generated source for method __init__ """
        self.pos0 = pos0
        self.pos1 = pos1
        self.vel0 = vel0
        self.vel1 = vel1
        self.length = self.getGaussianQuadratureLength(0, 1)

    def get(self, t, nD=0):
        """ generated source for method get_0 """
        if t < 0:
            return self.pos0
        if t > 1:
            return self.pos1
        return self.pos0.cpy().scl(self.basis(t, 0, nD)) \
                .add(self.pos1.cpy().scl(self.basis(t, 3, nD))) \
                    .add(self.vel0.cpy().scl(self.basis(t, 1, nD))) \
                        .add(self.vel1.cpy().scl(self.basis(t, 2, nD)))

    def basis(self, t, i, nD):
        """ generated source for method basis """
        if nD==0:
            if i==0:
                return 1 - 3 * t * t + 2 * t * t * t
            elif i==1:
                return t - 2 * t * t + t * t * t
            elif i==2:
                return -t * t + t * t * t
            elif i==3:
                return 3 * t * t - 2 * t * t * t
            else:
                return 0
        elif nD==1:
            if i==0:
                return -6 * t + 6 * t * t
            elif i==1:
                return 1 - 4 * t + 3 * t * t
            elif i==2:
                return -2 * t + 3 * t * t
            elif i==3:
                return 6 * t - 6 * t * t
            else:
                return 0
        elif nD==2:
            if i==0:
                return -6 + 12 * t
            elif i==1:
                return -4 + 6 * t
            elif i==2:
                return -2 + 6 * t
            elif i==3:
                return 6 - 12 * t
            else:
                return 0
        else:
            return 0

    def getGaussianCoefs(self):
        """ generated source for method getGaussianCoefs """
        return [
            [0.179446470356207, 0.0000000000000000],
            [0.176562705366993, -0.178484181495848],
            [0.176562705366993, 0.178484181495848],
            [0.1680041021564500, -0.351231763453876],
            [0.1680041021564500, 0.351231763453876],
            [0.15404576107681, -0.512690537086477],
            [0.15404576107681, 0.512690537086477],
            [0.135136368468526, -0.657671159216691],
            [0.135136368468526, 0.657671159216691],
            [0.1118838471934040, -0.781514003896801],
            [0.1118838471934040, 0.781514003896801],
            [0.0850361483171792, -0.880239153726986],
            [0.0850361483171792, 0.880239153726986],
            [0.0554595293739872, -0.950675521768768],
            [0.0554595293739872, 0.950675521768768],
            [0.0241483028685479, -0.990575475314417],
            [0.0241483028685479, 0.990575475314417]
        ]

    def getGaussianQuadratureLength(self, start, end):
        """ generated source for method getGaussianQuadratureLength """
        coefficients = self.getGaussianCoefs()
        half = (end - start) / 2.0
        avg = (start + end) / 2.0
        length = 0
        for coefficient in coefficients:
            length += self.get(((avg + half * coefficient[1])), 1).len() * coefficient[0]
        return length * half

    def getClosestPoint(self, point):
        """ generated source for method getClosestPoint """
        return self.get(self.findClosestPointOnSpline(point))

    def findClosestPointOnSpline(self, point, steps=50, iterations=5):
        """ generated source for method findClosestPointOnSpline_0 """
        cur_dist = float("inf")
        cur_min = 0

        i = 0
        while i <= 1:
            cur_t = i
            
            i += 1. / steps
            
            if self.getSecondDerivAtT(cur_t, point) == 0: dt = 0
            else: dt = self.getFirstDerivAtT(cur_t, point) / self.getSecondDerivAtT(cur_t, point)
            counter = 0
            
            while dt != 0 and counter < iterations:
                # adjust based on Newton's method, get new derivatives
                cur_t -= dt
                if self.getSecondDerivAtT(cur_t, point) == 0: dt = 0
                else: dt = self.getFirstDerivAtT(cur_t, point) / self.getSecondDerivAtT(cur_t, point)
                counter += 1
                                
            cur_d = self.get(cur_t).dst2(point)
            # if distance is less than previous min, update distance and t
            if cur_d < cur_dist:
                cur_dist = cur_d
                cur_min = cur_t
            i += 1. / steps
        return (min(1, max(0, cur_min)))

    def getFirstDerivAtT(self, t, point):
        """ generated source for method getFirstDerivAtT """
        p = self.get(t)
        d1 = self.get(t, 1)
        x_a = p.x - point.x
        y_a = p.y - point.y
        z_a = p.z - point.z
        return 2 * (x_a * d1.x + y_a * d1.y + z_a * d1.z)

    def getSecondDerivAtT(self, t, point):
        """ generated source for method getSecondDerivAtT """
        p = self.get(t)
        d1 = self.get(t, 1)
        d2 = self.get(t, 2)
        x_a = p.x - point.x
        y_a = p.y - point.y
        z_a = p.z - point.z
        return 2 * (d1.x * d1.x + x_a * d2.x + d1.y * d1.y + y_a * d2.y + d1.z * d1.z + z_a * d2.z)

    def getTFromLength(self, length):
        """ generated source for method getTFromLength """
        t = length / self.length
       
        i = 0
        while i < 5:
            derivativeMagnitude = self.get(t, 1).len()
            if derivativeMagnitude > 0.0:
                t -= (self.getGaussianQuadratureLength(0, t) - length) / derivativeMagnitude
                # Clamp to [0, 1]
                t = min(1, max(t, 0))
            i += 1
        return t
    
    
# path = CubicHermite3D(Vector3(0, 0, 0), Vector3(30, 30, 5), Vector3(0, 0, 5), Vector3(5, 0, 0))
# print(path.getClosestPoint(Vector3(35, 35, 0)))
    