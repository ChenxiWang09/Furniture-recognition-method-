import numpy as np
import math


def matrix_generate(angle,axis):
    # angle unit
    angle=math.radians(angle)
    if axis=='x':
        return np.array([(1,0,0),(0,math.cos(angle),-math.sin(angle)),(0,math.sin(angle),math.cos(angle))])
    if axis=='y':
        return np.array([(math.cos(angle),0,math.sin(angle)),(0,1,0),(-math.sin(angle),0,math.cos(angle))])
    if axis=='z':
        return np.array([(math.cos(angle),-math.sin(angle),0),(math.sin(angle),math.cos(angle),0),(0,0,1)])

if __name__ == '__main__':
    rot1=matrix_generate(90,'y')
    rot2=matrix_generate(90,'z')
    rot3=np.array([(0,1,0),(1,0,0),(0,0,-1)])
    rot=np.dot(rot1,rot2)
    rot_c=np.dot(np.linalg.inv(rot3),rot)
    print(rot_c)