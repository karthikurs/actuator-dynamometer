#!/usr/bin/env python3

import numpy as np
from numpy import sin, cos
# expm is a matrix exponential function
from scipy.linalg import expm
from copy import deepcopy
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# mm
l1 = 125
l2_pll = 25
l2_perp = 140
l3_pll = 20
l3_perp = 120

r1 = np.sqrt( (l1 + l3_perp)*(l1 + l3_perp) + l3_pll*l3_pll)
r2 = np.sqrt( (l2_perp)*(l2_perp) + l2_pll*l2_pll)

gamma1 = np.arctan2(l3_pll, l1+l3_perp)
gamma2 = np.arctan2(l2_pll, l2_perp)

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

def fk_vec(theta1, theta2):
    p0 = np.array([0,0])
    p1 = p0 + [l1*sin(-theta1), -l1*cos(-theta1)]
    p21 = p1 + [l2_pll*sin(-theta1-theta2), -l2_pll*cos(-theta1-theta2)]
    p22 = p21 + [-l2_perp*cos(-theta1-theta2), -l2_perp*sin(-theta1-theta2)]
    p31 = p22 + [-l3_pll*cos(-theta1), -l3_pll*sin(-theta1)]
    p32 = p31 + [l3_perp*sin(-theta1), -l3_perp*cos(-theta1)]

    return np.array([p0, p1, p21, p22, p31, p32])

def fk_2link(theta1, theta2):
    a1 = -0.5*np.pi - theta1 - gamma1
    a2 = -0.5*np.pi - theta2 + gamma2 + gamma1

    p0 = np.array([0,0])
    p1 = p0 + [r1*cos(a1), r1*sin(a1)]
    p2 = p1 + [r2*cos(a1+a2), r2*sin(a1+a2)]

    return np.array([p0,p1,p2])

def ik_2link(x, y):
    cosarg = (x*x + y*y - r1*r1 - r2*r2) / (2*r1*r2)
    if cosarg > 1 or cosarg < -1 :
        print('invalid arccos arg: {}'.format(cosarg))
    a2 = -np.arccos(cosarg)
    a1 = np.arctan2(y,x) - np.arctan2(r2*np.sin(a2), r1+r2*np.cos(a2))

    theta1 = -a1 - gamma1 - 0.5*np.pi
    theta2 = -a2 + gamma1 + gamma2 - 0.5*np.pi

    return np.array([clamp(theta1), clamp(theta2)])

def jacobian(theta1, theta2):
    a1 = -0.5*np.pi - theta1 - gamma1
    a2 = -0.5*np.pi - theta2 + gamma2 + gamma1

def rss(x, y):
    return np.sqrt(x*x + y*y)

def main():
    theta1 = -0.1*np.pi
    theta2 = 0.2*np.pi

    print(np.array([theta1, theta2]))

    vec_pts = fk_vec(theta1, theta2)
    link_pts = fk_2link(theta1, theta2)
    x = vec_pts[-1, 0]
    y = vec_pts[-1, 1]
    ik_soln = ik_2link(x, y)
    link_pts2 = fk_2link(ik_soln[0], ik_soln[1])
    print(ik_soln)
    print([x,y])
    fig, ax = plt.subplots()

    ax.plot(vec_pts[:,0], vec_pts[:,1], 'bo-')
    ax.plot(link_pts[:,0], link_pts[:,1], 'ro-')
    ax.plot(link_pts2[:,0], link_pts2[:,1], 'go-')

    x_left, x_right = ax.get_xlim()
    y_low, y_high = ax.get_ylim()
    ax.set_aspect(1)

    plt.show()

if __name__ == "__main__" :
    main()