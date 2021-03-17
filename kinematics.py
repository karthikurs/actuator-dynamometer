#!/usr/bin/env python3

import numpy as np
from numpy import sin, cos
from numpy import linalg as LA
from numpy import linspace
# expm is a matrix exponential function
from scipy.linalg import expm
from copy import deepcopy
import matplotlib as mpl
from matplotlib import cm
import matplotlib.pyplot as plt

# mm
# l1 = 125
# l2_pll = 25
# l2_perp = 140
# l3_pll = 20
# l3_perp = 120

# print(r1)
# print(r2)
# print(gamma1)
# print(gamma2)

def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle

def rss(x, y):
    return np.sqrt(x*x + y*y)

class Kinematics:
    def __init__(self, l1_in=125, l2_pll_in=25, l2_perp_in=140, l3_pll_in=20, l3_perp_in=120):
        self.l1 = l1_in
        self.l2_pll = l2_pll_in
        self.l2_perp = l2_perp_in
        self.l3_pll = l3_pll_in
        self.l3_perp = l3_perp_in
        self.r1 = np.sqrt( (self.l1 + self.l3_perp)*(self.l1 + self.l3_perp) + self.l3_pll*self.l3_pll)
        self.r2 = np.sqrt( (self.l2_perp)*(self.l2_perp) + self.l2_pll*self.l2_pll)

        self.gamma1 = np.arctan2(self.l3_pll, self.l1+self.l3_perp)
        self.gamma2 = np.arctan2(self.l2_pll, self.l2_perp)

    def theta2alpha(self, theta1, theta2):
        a1 = -0.5*np.pi - theta1 - self.gamma1
        a2 = -0.5*np.pi - theta2 + self.gamma2 + self.gamma1
        return a1, a2

    def alpha2theta(self, a1, a2):
        theta1 = -a1 - self.gamma1 - 0.5*np.pi
        theta2 = -a2 + self.gamma1 + self.gamma2 - 0.5*np.pi
        return theta1, theta2

    def fk_vec(self, theta1, theta2):
        p0 = np.array([0,0])
        p1 = p0 + [self.l1*sin(-theta1), -self.l1*cos(-theta1)]
        p21 = p1 + [self.l2_pll*sin(-theta1-theta2), -self.l2_pll*cos(-theta1-theta2)]
        p22 = p21 + [-self.l2_perp*cos(-theta1-theta2), -self.l2_perp*sin(-theta1-theta2)]
        p31 = p22 + [-self.l3_pll*cos(-theta1), -self.l3_pll*sin(-theta1)]
        p32 = p31 + [self.l3_perp*sin(-theta1), -self.l3_perp*cos(-theta1)]

        return np.array([p0, p1, p21, p22, p31, p32])

    def fk_2link(self, theta1, theta2):
        # a1 = -0.5*np.pi - theta1 - self.gamma1
        # a2 = -0.5*np.pi - theta2 + self.gamma2 + self.gamma1
        a1, a2 = self.theta2alpha(theta1, theta2)

        p0 = np.array([0,0])
        p1 = p0 + [self.r1*cos(a1), self.r1*sin(a1)]
        p2 = p1 + [self.r2*cos(a1+a2), self.r2*sin(a1+a2)]

        return np.array([p0,p1,p2])

    def ik_2link(self, x, y):
        cosarg = (x*x + y*y - self.r1*self.r1 - self.r2*self.r2) / (2*self.r1*self.r2)
        if cosarg > 1 or cosarg < -1 :
            print('invalid arccos arg: {}'.format(cosarg))
        a2 = -np.arccos(cosarg)
        a1 = np.arctan2(y,x) - np.arctan2(self.r2*np.sin(a2), self.r1+self.r2*np.cos(a2))

        # theta1 = -a1 - self.gamma1 - 0.5*np.pi
        # theta2 = -a2 + self.gamma1 + self.gamma2 - 0.5*np.pi
        theta1, theta2 = self.alpha2theta(a1, a2)

        return np.array([clamp(theta1), clamp(theta2)])

    def jacobian_alpha(self, a1, a2):
        s1 = sin(a1)
        c1 = cos(a1)
        s12 = sin(a1+a2)
        c12 = cos(a1+a2)

        J11 = -self.r1*s1 - self.r2*s12
        J21 = self.r1*c1 + self.r2*c12
        J12 = -self.r2*s12
        J22 = self.r2*c12

        return np.array([[J11, J12],[J21, J22]])

    def jacobian_theta(self, theta1, theta2):
        # a1 = -0.5*np.pi - theta1 - self.gamma1
        # a2 = -0.5*np.pi - theta2 + self.gamma2 + self.gamma1
        a1, a2 = self.theta2alpha(theta1, theta2)
        return -self.jacobian_alpha(a1, a2)


def main():
    kin = Kinematics()
    theta1 = -0.1*np.pi
    theta2 = 0.2*np.pi
    a1, a2 = kin.theta2alpha(theta1, theta2)

    print(kin.fk_vec(-0.15*np.pi/3, -1.05*np.pi/3))

    # print(np.array([thetfrom numpy import linspacea1, theta2]))
    x0 = 0
    y0 = 0

    fig, ax = plt.subplots()

    start = 0.0
    stop = 1.0
    number_of_lines= 30
    cm_subsection = linspace(start, stop, number_of_lines) 
    colors = [ cm.viridis(x) for x in cm_subsection ]

    for ii in range(20):
        vec_pts = kin.fk_vec(theta1, theta2)
        link_pts = kin.fk_2link(theta1, theta2)
        x = vec_pts[-1, 0]
        y = vec_pts[-1, 1]
        if ii==0:
            x0 = x
            y0 = y
        ik_soln = kin.ik_2link(x, y)
        link_pts2 = kin.fk_2link(ik_soln[0], ik_soln[1])

        ax.plot(vec_pts[:,0], vec_pts[:,1], 'o-',color=colors[ii])
        # ax.plot(link_pts[:,0], link_pts[:,1], 'ro-')
        # ax.plot(link_pts2[:,0], link_pts2[:,1], 'go-')
        plt.title('theta={},     alpha={},\nfoot={},     ik_soln={}'.format(\
            [round(e,2) for e in [theta1, theta2]],\
            [round(e,2) for e in [a1,a2]],\
            [round(e,2) for e in [x,y]],\
            [round(e,2) for e in ik_soln]))

        dx = (x0+ii*10 + 10) - x
        dy = y0 - y
        J = kin.jacobian_theta(theta1, theta2)
        Jinv = LA.inv(J)
        dt = np.matmul(Jinv, np.array([[dx],[dy]]))
        dt1 = dt[0,0]
        dt2 = dt[1,0]

        theta1 += dt1
        theta2 += dt2
    
    for ii in range(10):
        vec_pts = kin.fk_vec(theta1, theta2)
        link_pts = kin.fk_2link(theta1, theta2)
        x = vec_pts[-1, 0]
        y = vec_pts[-1, 1]
        if ii==0:
            x0 = x
            y0 = y
        ik_soln = kin.ik_2link(x, y)
        link_pts2 = kin.fk_2link(ik_soln[0], ik_soln[1])

        ax.plot(vec_pts[:,0], vec_pts[:,1], 'o-',color=colors[ii+20])
        # ax.plot(link_pts[:,0], link_pts[:,1], 'ro-')
        # ax.plot(link_pts2[:,0], link_pts2[:,1], 'go-')
        plt.title('theta={},     alpha={},\nfoot={},     ik_soln={}'.format(\
            [round(e,2) for e in [theta1, theta2]],\
            [round(e,2) for e in [a1,a2]],\
            [round(e,2) for e in [x,y]],\
            [round(e,2) for e in ik_soln]))

        dx = x0 - x
        dy = (y0+ii*10 + 10) - y
        J = kin.jacobian_theta(theta1, theta2)
        Jinv = LA.inv(J)
        dt = np.matmul(Jinv, np.array([[dx],[dy]]))
        dt1 = dt[0,0]
        dt2 = dt[1,0]

        theta1 += dt1
        theta2 += dt2

    ax.set_aspect(1)

    plt.show()

if __name__ == "__main__" :
    main()