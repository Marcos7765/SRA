import numpy as np
import math
__all__ = ["PathGenerator"]

def _linspace(a=0, b=1, num_points=2):
    incr = (b-a)/(num_points-1)
    return [a + incr*i for i in range(0,num_points-1)]+[b]

class PathGenerator:
    def __init__(self, qi, qf):
        delta = math.radians(1)

        dx = qf[0] - qi[0]
        dy = qf[1] - qi[1]
        alpha_i = math.tan(qi[2])
        alpha_f = math.tan(qf[2])

        meio_pi = math.pi/2

        self.qi = qi
        self.qf = qf
        self.a0 = qi[0]
        self.b0 = qi[1]

        if ((meio_pi - delta <= abs(qi[2]) < meio_pi + delta) and (meio_pi - delta <= abs(qf[2]) < meio_pi + delta)):

            self.a1 = 0
            self.a2 = 3 * dx
            self.a3 = -2 * dx

            self.b1 = dy
            self.b2 = 0
            self.b3 = dy - self.b1 - self.b2

        elif (meio_pi - delta <= abs(qi[2]) < meio_pi + delta):

            self.a3 = -dx / 2
            self.a2 = dx - self.a3
            self.a1 = 0

            self.b1 = 2 * (dy - alpha_f * dx) - alpha_f * self.a3 + self.b3
            self.b2 = 1  # Qualquer valor
            self.b3 = (2 * alpha_f * dx - dy) + alpha_f * self.a3 - 2 * self.b3

        elif (meio_pi - delta <= abs(qf[2]) < meio_pi + delta):

            self.a1 = 3 * dx / 2
            self.a2 = 3 * dx - 2 * self.a1
            self.a3 = self.a1 - 2 * dx

            self.b1 = alpha_i * self.a1
            self.b2 = 1  # Qualquer valor
            self.b3 = dy - alpha_i * self.a1 - self.b2

        else:

            self.a1 = dx
            self.a2 = 0
            self.a3 = dx - self.a1 - self.a2

            self.b1 = alpha_i * self.a1
            self.b2 = 3 * (dy - alpha_f * dx) + 2 * (alpha_f - alpha_i) * self.a1 + alpha_f * self.a2
            self.b3 = 3 * alpha_f * dx - 2 * dy - (2 * alpha_f - alpha_i) * self.a1 - alpha_f * self.a2

        #pode ter algo de errado aqui
        self.beta_1 = math.sqrt(self.a1**2 + self.b1**2)
        self.beta_2 = 2*(self.a1*self.a2 + self.b1*self.b2)/self.beta_1
        self.beta_3 = (2*(2*(self.a2**2 + self.b2**2) + 3*(self.a1*self.a3 + self.b1*self.b3))- 4*((self.a1*self.a2 + self.b1*self.b2)**2)/(self.beta_1**2))/self.beta_1
        
        self.comprimento_total = self.beta_1 + self.beta_2/2 + self.beta_3/6 # s(1) da func. comprimento da pág 35
    def __call__(self, lambd):
        x = self.a0 + self.a1*lambd + self.a2*lambd**2 + self.a3*lambd**3
        y = self.b0 + self.b1*lambd + self.b2*lambd**2 + self.b3*lambd**3
        theta = math.atan2(self.b1 + 2*self.b2*lambd + 3*self.b3*lambd**2, self.a1+2*self.a2*lambd+3*self.a3*lambd**2)
        return [x, y, theta]
    
    def getGeneratedPath(self, nPoints=100):
        points = _linspace(0, 1, nPoints)
        return [self(t) for t in points]
    
    def calc_distancia_percorrida(self, lambd):
        return self.beta_1*lambd + self.beta_2*(lambd**2)/2 + self.beta_3*(lambd**3)/6