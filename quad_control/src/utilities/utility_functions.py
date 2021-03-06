#!/usr/bin/env python
# this line is just used to define the type of document



#TODO is rospy needed?
import rospy

import numpy

import math


from numpy import cos as c
from numpy import sin as s


#TODO decapitalize all the function names


GRAVITY = 9.81
E3_VERSOR = numpy.array([0.0, 0.0, 1.0])


def skew(xx):
    """Skew matrix blabla"""
    x = xx[0]
    y = xx[1]
    z = xx[2]
    return numpy.array([[0,-z,y],[z,0,-x],[-y,x,0]])

# def skew(x):
#     out = numpy.zeros((3,3))
#     out[0,1] = -x[2]
#     out[0,2] =  x[1]
#     out[1,2] = -x[0]
#     out[1,0] =  x[2]
#     out[2,0] = -x[1]
#     out[2,1] =  x[0]
#     return out

def unskew(X):
    out = numpy.zeros(3)
    out[0] = -X[1,2]
    out[1] =  X[0,2]
    out[2] = -X[0,1]
    return out    

# print skew([1,2,3])

#--------------------------------------------------------------------------#
# orthogonal projection operator
#TODO rename to projection_operator or something like that
def OP(x):
    
    return -skew(x).dot(skew(x))

#print OP([1,2,3])
#print OP([1,0,0])

#TODO rename all the rotations descriptive names,
# not capitalized

def Rx(tt):
    
    return numpy.array([[1.0,0.0,0.0],[0.0,c(tt),-s(tt)],[0.0,s(tt),c(tt)]])

# print Rx(60*3.14/180)

def Ry(tt):
    
    return numpy.array([[c(tt),0.0,s(tt)],[0.0,1,0.0],[-s(tt),0.0,c(tt)]])

# print Ry(60*3.14/180)

def Rz(tt):
    
    return numpy.array([[c(tt),-s(tt),0.0],[s(tt),c(tt),0.0],[0.0,0.0,1]])

# print Rz(60*3.14/180)

#--------------------------------------------------------------------------#
# unit vector
def unit_vec(psi,theta):

    e1  = numpy.array([1.0,0.0,0.0])
    aux = Rz(psi).dot(e1)
    aux = Ry(theta).dot(aux)

    return aux

#print unit_vec(45*3.14/180,0)
#print unit_vec(45*3.14/180,45*3.14/180)
#print unit_vec(0*3.14/180,-90*3.14/180)

def psi_theta_from_unit_vec(v):
    theta = numpy.arctan2(-v[2],v[0])
    
    psi = numpy.arcsin(v[1])
    return psi, theta

def unit_vec_from_lat_long(psi,theta):
    v_0 = c(theta)*c(psi)
    v_1 = c(theta)*s(psi)
    v_2 = s(theta)
    return numpy.array([v_0, v_1, v_2])

#print unit_vec_from_lat_long(math.pi/3, -math.pi/4)

def lat_long_from_unit_vec(v):
    psi = numpy.arctan2(v[1],v[0])
    theta = numpy.arcsin(v[2])
    return psi, theta


#print lat_long_from_unit_vec([ 0.35355339,  0.61237244, -0.70710678])
def bound(x,maxmax,minmin):

    return numpy.maximum(minmin,numpy.minimum(maxmax,x))

#--------------------------------------------------------------------------

#TODO decapitalize: get_euler_angles(rot_max)
def GetEulerAngles(R):

    #phi   = atan2(R(3,2),R(3,3));
    #theta = asin(-R(3,1));
    #psi   = atan2(R(2,1),R(1,1));

    EULER = numpy.array([0.0,0.0,0.0])

    EULER[0] = numpy.arctan2(bound(R[2,1],1,-1),bound(R[2,2],1,-1));
    EULER[1] = numpy.arcsin(-bound(R[2,0],1,-1));
    EULER[2] = numpy.arctan2(bound(R[1,0],1,-1),bound(R[0,0],1,-1));    

    return EULER


def GetEulerAnglesDeg(R):
    return GetEulerAngles(R)*180.0/math.pi


def GetRotFromEulerAngles(ee_rad):
    return Rz(ee_rad[2]).dot(Ry(ee_rad[1]).dot(Rx(ee_rad[0])))


def GetRotFromEulerAnglesDeg(ee_deg):
    return GetRotFromEulerAngles(ee_deg*math.pi/180.0)



# testing skew matrix    
# print skew(numpy.array([1,2,3]))

def quaternion_to_rot(quaternion):

    q   = quaternion
    q_v = q[0:3] 
    q_n = q[3]
    qc  = numpy.concatenate([-q_v,[q_n]])

    R  = numpy.dot(q,qc)*numpy.identity(3) + 2*q_n*skew(q_v) + 2*numpy.outer(q_v,q_v)

    return R


def quaternion_from_unit_vector(unit_vector,psi):

    r3 = unit_vector
    r1 = numpy.array([numpy.cos(psi),numpy.sin(psi),0.0])
    r1 = numpy.dot(numpy.identity(3) - numpy.outer(r3,r3),r1)
    r1 = r1/numpy.linalg.norm(r1)
    r2 = numpy.dot(skew(r3),r1)

    RR = column_stack((r1,r2,r3))

    return rot_to_quaternion(RR)


def quaternion_from_unit_vector(unit_vector,psi):

    r3 = unit_vector
    r1 = numpy.array([numpy.cos(psi),numpy.sin(psi),0.0])
    r1 = numpy.dot(numpy.identity(3) - numpy.outer(r3,r3),r1)
    r1 = r1/numpy.linalg.norm(r1)
    r2 = numpy.dot(skew(r3),r1)

    RR = column_stack((r1,r2,r3))

    return rot_to_quaternion(RR)    



def roll_pitch(Full_actuation,psi):

    #--------------------------------------#
    # Rz(psi)*Ry(theta_des)*Rx(phi_des) = n_des

    # desired roll and pitch angles
    n_des     = Full_actuation/numpy.linalg.norm(Full_actuation)
    n_des_rot = Rz(-psi).dot(n_des)


    sin_phi   = -n_des_rot[1]
    sin_phi   = bound(sin_phi,1,-1)
    phi       = numpy.arcsin(sin_phi)

    sin_theta = n_des_rot[0]/numpy.cos(phi)
    sin_theta = bound(sin_theta,1,-1)
    cos_theta = n_des_rot[2]/numpy.cos(phi)
    cos_theta = bound(cos_theta,1,-1)
    pitch     = numpy.arctan2(sin_theta,cos_theta)

    return (phi,pitch)
#--------------------------------------------------------------------------#
#--------------------------------------------------------------------------#
# For computing velocity from position measurements


class Median_Filter():
    # N is order of median filter
    def __init__(self, N):
        self.N = N
        self.data = numpy.zeros(N)
    
    def update_data(self,new_data):
        self.data[:-1] = self.data[1:]
        self.data[-1]  = new_data

    def output(self):
        return numpy.median(self.data)

    def up_and_out(self,new_data):
        self.update_data(new_data)
        return self.output()


class Median_Filter_3D():
    # N is order of median filter
    def __init__(self, N):
        self.N = N
        self.Dx =  Median_Filter(N)
        self.Dy =  Median_Filter(N)
        self.Dz =  Median_Filter(N)

    def up_and_out(self,new_data):
        Dx_new = self.Dx.up_and_out(new_data[0])
        Dy_new = self.Dy.up_and_out(new_data[1])
        Dz_new = self.Dz.up_and_out(new_data[2])
        return numpy.array([Dx_new,Dy_new,Dz_new])

class Velocity_Filter():
    def __init__(self,N,old_position,old_time):
        self.median_filter = Median_Filter_3D(N)
        self.old_position = old_position
        self.old_time = old_time

    def out(self,new_position,new_time):
        dt = new_time - self.old_time
        vel_estimate =  (new_position - self.old_position)/dt
        self.old_position = new_position
        self.old_time = new_time
        out = self.median_filter.up_and_out(vel_estimate)

        return out



class Mean_Filter():
    # N is order of median filter
    def __init__(self, N):
        self.N = N
        self.data = numpy.zeros(N)
    
    def update_data(self,new_data):
        self.data[:-1] = self.data[1:]
        self.data[-1]  = new_data

    def output(self):
        return numpy.mean(self.data)

    def up_and_out(self,new_data):
        self.update_data(new_data)
        return self.output()


class Mean_Filter_3D():
    # N is order of median filter
    def __init__(self, N):
        self.N = N
        self.Dx =  Mean_Filter(N)
        self.Dy =  Mean_Filter(N)
        self.Dz =  Mean_Filter(N)

    def up_and_out(self,new_data):
        Dx_new = self.Dx.up_and_out(new_data[0])
        Dy_new = self.Dy.up_and_out(new_data[1])
        Dz_new = self.Dz.up_and_out(new_data[2])
        return numpy.array([Dx_new,Dy_new,Dz_new])

class Velocity_Mean_Filter():
    def __init__(self,N,old_position,old_time):
        self.mean_filter = Mean_Filter_3D(N)
        self.old_position = old_position
        self.old_time = old_time

    def out(self,new_position,new_time):
        dt = new_time - self.old_time
        vel_estimate =  (new_position - self.old_position)/dt
        self.old_position = new_position
        self.old_time = new_time
        out = self.mean_filter.up_and_out(vel_estimate)
        return out


#--------------------------------------------------------------------------#
#--------------------------------------------------------------------------#

