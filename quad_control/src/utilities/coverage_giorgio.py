import numpy as np
norm = np.linalg.norm

from math import exp,tan,cos

#from quaternion_utilities import q_mult, qv_mult

from utilities.utility_functions import skew, lat_long_from_unit_vec

import geometry_msgs.msg as gms

import pickle

# Distance functions

D_OPT = 6.

def f(x):
	"""Function that weights the distance"""
	value = x/D_OPT * exp(1-x/D_OPT)
	return value

def d_f(x):
	"""Derivative of f(x)"""
	value = exp(1-x/D_OPT) * (1-x/D_OPT) / D_OPT
	return value

def f_bar(x):
	"""f(x)/x**2"""
	return f(x)/x**2
	
def d_f_bar(x):
	"""Derivative of f_bar"""
	return (x*d_f(x) - 2*f(x)) / (x**3)

# Classes: landmark, list of landmarks and camera

class Camera:

	def __init__(self,p_0 = [0.,0.,1.],v_0 = [0.,-1.,0.]):
		if ( abs(norm(v_0)-1)<1e-5):
			self.p = np.array(p_0)
			self.v = np.array(v_0)
		else:
			print "Error : v vector not unitary"
			# throw exception

	def __str__(self):
		string = ""
		string += "\np = " + str(self.p)
		string += "\nv = " + str(self.v)
		return string
	
	def position(self):
		return self.p

	def orientation_as_vector(self):
		return self.v

	def orientation_as_lat_long(self):
		return lat_long_from_unit_vec(self.v)

	def new_pose(self,cam_pose):
		self.p = np.array([cam_pose.px,cam_pose.py,cam_pose.pz])
		self.v = np.array([cam_pose.vx,cam_pose.vy,cam_pose.vz])

# Test
# cam = Camera([-2.,1.,0.75],[1.,0.,0.])
# print cam
# print cam.position()
# print cam.orientation_as_vector()
# print cam.orientation_as_angles()

class Landmark:

	def __init__(self,q_0,u_0):
		if ( abs(norm(u_0)-1)<1e-5):
			self.q = np.array(q_0)
			self.u = np.array(u_0)
		else:
			print "Error : u vector not unitary"

	def __str__(self):
		string = ""
		string += "\nq = " + str(self.q)
		string += "\nu = " + str(self.u)
		return string
	
	def is_visible(self,cam):
		if (self.vision(cam) > 0.):
			return True
		else:
			return False
	
	def vision(self,cam):
		q = self.q
		u = self.u
		p = cam.p
		v = cam.v
		dist = norm(q-p)
		cos_alpha = (q-p).dot(v)
		cos_beta = (p-q).dot(u)
		if (cos_alpha>0.)and(cos_beta>0.):
			return f_bar(dist) * cos_alpha * cos_beta
		else:
			return 0.

#Test
# from utility_functions import unit_vec
# from math import pi
# lmk = Landmark([-1.,-1.,0.],unit_vec(pi/3, 0))
# cam = Camera([-2.,1.,0.75],[1.,0.,0.])
# print str(lmk)
# print lmk.is_visible(cam)
# print lmk.vision(cam)


class Landmark_list:

	def __init__(self, lst = []):
		if not(all(isinstance(lm,Landmark) for lm in lst)):
			print "Error: ", lst, "contains element that are not of class Landmark"
			# throw exception
		else:
			self.lst = lst

	def __str__(self):
		string = ""
		for i in range(0,len(self.lst)):
			string += "\nLandmark " + str(i) + ":" + str(self.lst[i])
		return string
		
	def append(self,lm_new):
		if (self.remove(lm_new) == lm_new):
			print "Landmark already present in the list"
		self.lst.append(lm_new)
	
	def remove(self, lm_to_remove):
		for lm in self.lst:
			if (lm == lm_to_remove):
				self.lst.remove(lm_to_remove)
				return lm_to_remove
		else:
			return []
	
	def vision(self,cam):
		vis = 0.
		for lm in self.lst:
			vis += lm.vision(cam)	
		return vis
		
	def visible_set(self,cam):
		vis_set = Landmark_list([])
		for lm in self.lst:
			if (lm.is_visible(cam)):
				vis_set.append(lm)
		return vis_set		

# Gradient computation

def linear_velocity(cam, Q):
	Q_v = Q.visible_set(cam)
	dim = len(cam.p)
	p = cam.p
	v = cam.v
	sum = np.zeros((dim,dim))
	for i in range(0,len(Q_v.lst)):
		q = Q_v.lst[i].q
		u = Q_v.lst[i].u
		r = q-p
		dist = norm(r)
		s_1 = d_f_bar(dist)/dist * r.dot(u)
		m_1 = np.transpose([r])*r
		s_2 = f_bar(dist)
		m_2 = np.transpose([u]) * r + (r.dot(u)) * np.identity(dim)
		sum += s_1 * m_1 + s_2 * m_2
	return sum.dot(v)

# # Test
# from utility_functions import unit_vec
# from math import pi
# lmk_0 = Landmark([-3.,-1.,0.],unit_vec(pi/2, pi/6))
# lmk_1 = Landmark([-2.,-1.,0.],unit_vec(pi/3, -pi/4))
# lmk_2 = Landmark([-1.,-1.,0.],unit_vec(pi/3, 0))
# landmarks = Landmark_list([lmk_0, lmk_1, lmk_2])
# p = [-2.,1.,0.75]
# v = unit_vec(-2*pi/3,-3*pi/4)
# cam = Camera(p,v)
# print linear_velocity(cam, landmarks)


def angular_velocity(cam,Q):
	Q_v = Q.visible_set(cam)
	dim = len(cam.p)
	p = cam.p
	v = cam.v
		
	sum = np.zeros(dim)

	for i in range(0,len(Q_v.lst)):
		q = Q_v.lst[i].q
		u = Q_v.lst[i].u
		r = q-p
		dist = norm(r)
		sum += f_bar(dist) * (-r).dot(u) * r
	
	if (dim == 3):
		S_v = skew(v)
	elif (dim == 2):
		S_v = [-cam.v[1], cam.v[0]]
	return S_v.dot(sum)

# # Test
# from utility_functions import unit_vec
# from math import pi
# lmk_0 = Landmark([-3.,-1.,0.],unit_vec(pi/2, pi/6))
# lmk_1 = Landmark([-2.,-1.,0.],unit_vec(pi/3, -pi/4))
# lmk_2 = Landmark([-1.,-1.,0.],unit_vec(pi/3, 0))
# landmarks = Landmark_list([lmk_0, lmk_1, lmk_2])
# p = [-2.,1.,0.75]
# v = unit_vec(-2*pi/3,-3*pi/4)
# cam = Camera(p,v)
# print angular_velocity(cam, landmarks)

def omega_camera_to_ee_rates(omega_xyz,pitch):
	L = np.matrix([[1.,0.,tan(pitch)],[0.,1.,0.],[0.,0.,1./cos(pitch)]])
	omega_rpy = L.dot(omega_xyz)
	return (omega_rpy[0,1], omega_rpy[0,2])

# Test
#omega = np.array([0.5, 1., 0.3])
#pitch = 1.3
#print(omega_camera_to_ee_rates(omega, pitch))


# Reading and writing Landmark objects

def read_lmks_from_file(filename):
	file_object = open(filename)
	lmks = pickle.load(file_object)
	file_object.close()
	return lmks