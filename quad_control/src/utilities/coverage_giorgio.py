import rospy

import numpy as np
norm = np.linalg.norm

from math import exp,tan,sin,cos,pi

#from quaternion_utilities import q_mult, qv_mult

from utilities.utility_functions import skew, lat_long_from_unit_vec, unit_vec_from_lat_long

import geometry_msgs.msg as gms
from quad_control.srv import LandmarksTrade

import pickle

# Distance functions

def f(x, D_OPT):
	"""Function that weights the distance"""
	value = x/D_OPT * exp(1-x/D_OPT)
	return value

def d_f(x, D_OPT):
	"""Derivative of f(x)"""
	value = exp(1-x/D_OPT) * (1-x/D_OPT) / D_OPT
	return value

def f_bar(x, D_OPT):
	"""f(x)/x**2"""
	return f(x, D_OPT)/x**2
	
def d_f_bar(x, D_OPT):
	"""Derivative of f_bar"""
	return (x*d_f(x, D_OPT) - 2*f(x, D_OPT)) / (x**3)

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

	def change_position(self,p_new):
		self.p = p_new

	def change_lat_long(self,lat_new,long_new):
		self.v = unit_vec_from_lat_long(long_new,lat_new)

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
	
	def is_visible(self,cam, D_OPT):
		if (self.vision(cam, D_OPT) > 0.):
			return True
		else:
			return False
	
	def vision(self,cam, D_OPT):
		q = self.q
		u = self.u
		p = cam.p
		v = cam.v
		dist = norm(q-p)
		cos_alpha = (q-p).dot(v)
		cos_beta = (p-q).dot(u)
		if (cos_alpha>0.)and(cos_beta>0.):
			return f_bar(dist, D_OPT) * cos_alpha * cos_beta
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
	
	def vision(self,cam, D_OPT):
		vis = 0.
		for lm in self.lst:
			vis += lm.vision(cam, D_OPT)	
		return vis
		
	def visible_set(self, cam, D_OPT):
		vis_set = Landmark_list([])
		for lm in self.lst:
			if (lm.is_visible(cam, D_OPT)):
				vis_set.append(lm)
		return vis_set

	def not_visible_set(self, cam, D_OPT):
		not_vis_set = Landmark_list([])
		for lm in self.lst:
			if not(lm.is_visible(cam, D_OPT)):
				not_vis_set.append(lm)
		return not_vis_set

	def list(self):
		return self.lst

	def to_lists(self):
		q = []
		u = []
		for lmk in self.lst:
			q.extend(list(lmk.q[:]))
			long_u , lat_u =lat_long_from_unit_vec(lmk.u)
			u.append(long_u)
			u.append(lat_u)
		return q,u

	def from_lists(self,q,u):
		l_q = int(len(q)/3)
		l_u = int(len(u)/2)
		self.lst = []
		if not(l_q == l_u):
			print "Error: lists of different lengths"
		else:
			for i in range(l_q):
				q_0 = q[3*i:3*(i+1)]
				u_0 = unit_vec_from_lat_long(u[2*i],u[2*i+1])
				lmk = Landmark(q_0,u_0)
				self.lst.append(lmk)

# Test
# from utilities.utility_functions import unit_vec
# q_0 = [-3.,-1.,0.]
# q_1 = [-2.,-1.,0.]
# q_2 = [-1.,-1.,0.]
# psi_0 = pi/2.
# theta_0 = pi/6.
# psi_1 = pi/3.
# theta_1 = -pi/4.
# psi_2 = pi/3.
# theta_2 = 0.
# u_0 = unit_vec(psi_0, theta_0)
# u_1 = unit_vec(psi_1, theta_1)
# u_2 = unit_vec(psi_2, theta_2)
# lm_0 = Landmark(q_0,u_0)
# lm_1 = Landmark(q_1,u_1)
# lm_2 = Landmark(q_2,u_2)
# lmk_list = Landmark_list([lm_0,lm_1,lm_2])
# q,u = lmk_list.to_lists()
# print q
# print u
# new_lmk_lst = Landmark_list([])
# new_lmk_lst.from_lists(q,u)
# print str(lmk_list)
# print str(new_lmk_lst)

# Gradient computation

def linear_velocity(cam, Q, D_OPT):
	Q_v = Q.visible_set(cam, D_OPT)
	dim = len(cam.p)
	p = cam.p
	v = cam.v
	sum = np.zeros((dim,dim))
	for i in range(0,len(Q_v.lst)):
		q = Q_v.lst[i].q
		u = Q_v.lst[i].u
		r = q-p
		dist = norm(r)
		s_1 = d_f_bar(dist, D_OPT)/dist * r.dot(u)
		m_1 = np.transpose([r])*r
		s_2 = f_bar(dist, D_OPT)
		m_2 = np.transpose([u]) * r + (r.dot(u)) * np.identity(dim)
		sum += s_1 * m_1 + s_2 * m_2
	return sum.dot(v)

# Test
# from utility_functions import unit_vec
# from math import pi
# lmk_0 = Landmark([-3.,-1.,0.],unit_vec(pi/2, pi/6))
# lmk_1 = Landmark([-2.,-1.,0.],unit_vec(pi/3, -pi/4))
# lmk_2 = Landmark([-1.,-1.,0.],unit_vec(pi/3, 0))
# landmarks = Landmark_list([lmk_0, lmk_1, lmk_2])
# p = [-2.,1.,0.75]
# v = unit_vec(-2*pi/3,-3*pi/4)
# cam = Camera(p,v)
# print linear_velocity(cam, landmarks, D_OPT = 6.)


def angular_velocity(cam,Q, D_OPT):
	Q_v = Q.visible_set(cam, D_OPT)
	dim = len(cam.p)
	p = cam.p
	v = cam.v
		
	sum = np.zeros(dim)

	for i in range(0,len(Q_v.lst)):
		q = Q_v.lst[i].q
		u = Q_v.lst[i].u
		r = q-p
		dist = norm(r)
		sum += f_bar(dist, D_OPT) * (-r).dot(u) * r
	
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

def bls_velocity(cam, Q, p_dot_max, p_dot_star, D_OPT, v_lim=0.3, alpha=1, beta=0.5, tau=0.9, t_s=0.1, v_min=1e-2):
	if not(p_dot_max.any()) or not(p_dot_star.any()):
		return np.zeros(3)
	# alpha = alpha * p_dot_max.dot(1/norm(p_dot_max)).dot(p_dot_star)
	f_now = Q.vision(cam,D_OPT)
	p = cam.position()
	delta_p = p_dot_star.dot(t_s*v_lim)
	p_next = p + delta_p.dot(alpha)
	cam_next = Camera(p_next, cam.orientation_as_vector())
	f_next = Q.vision(cam_next,D_OPT)
	scal = np.asscalar(p_dot_max.dot(delta_p))
	while (f_next < f_now + beta * alpha * scal):
		alpha *= tau
		p_next = p + delta_p.dot(alpha)
		cam_next.change_position(p_next)
		f_next = Q.vision(cam_next,D_OPT)
		if (norm(alpha * delta_p / t_s) < v_min):
			delta_p  = np.zeros(3)
			break
	return delta_p.dot(alpha/t_s)

# Test
# from utility_functions import unit_vec
# from math import pi
# D_OPT = 3
# lmk_0 = Landmark([-3.,-1.,0.],unit_vec(pi/2, pi/6))
# lmk_1 = Landmark([-2.,-1.,0.],unit_vec(pi/3, -pi/4))
# lmk_2 = Landmark([-1.,-1.,0.],unit_vec(pi/3, 0))
# Q = Landmark_list([lmk_0, lmk_1, lmk_2])
# p = np.array([-0.9135, 2.1766, 0.4850])
# v =  unit_vec(-1.2489, 2.6194)
# cam = Camera(p,v)
# p_dot_max = linear_velocity(cam, Q, D_OPT)
# p_dot_star = np.array([0.9196, -0.1812, -0.3486])
# print bls_velocity(cam,Q, p_dot_max, p_dot_star,D_OPT)

def bls_omega(cam, Q, omega_max, D_OPT, omega_lim=0.79, alpha=1, beta=0.5, tau=0.9, t_s=0.1, omega_min=0.02):
	if not(omega_max.any()):
		return np.zeros(3)
	omega_new = omega_max.dot(omega_lim/norm(omega_max))
	longit, lat = cam.orientation_as_lat_long()
	long_rate_new = omega_new[2]
	lat_rate_new = sin(lat)*omega_new[0] - cos(lat)*omega_new[1] 
	longit_next = longit + long_rate_new*t_s
	lat_next = lat + lat_rate_new*t_s
	cam_next = Camera(cam.position(), cam.orientation_as_vector())
	cam_next.change_lat_long(lat_next,longit_next)
	f_now = Q.vision(cam,D_OPT)
	f_next = Q.vision(cam_next,D_OPT)
	scal = np.asscalar(omega_max.dot(omega_new))
	while (f_next < f_now + beta * scal * t_s):
		omega_new = omega_new.dot(tau) 
		long_rate_new = omega_new[2]
		lat_rate_new = sin(lat)*omega_new[0] - cos(lat)*omega_new[1] 
		longit_next = longit + long_rate_new*t_s
		lat_next = lat + lat_rate_new*t_s
		cam_next.change_lat_long(lat_next,longit_next)
		f_next = Q.vision(cam_next,D_OPT)
		scal = np.asscalar(omega_max.dot(omega_new))
		if (norm(omega_new) < omega_min):
			omega_new  = np.zeros(3)
			break
	return omega_new

# Test
# from utility_functions import unit_vec
# from math import pi
# D_OPT = 3
# lmk_0 = Landmark([-3.,-1.,0.],unit_vec(pi/2, pi/6))
# lmk_1 = Landmark([-2.,-1.,0.],unit_vec(pi/3, -pi/4))
# lmk_2 = Landmark([-1.,-1.,0.],unit_vec(pi/3, 0))
# Q = Landmark_list([lmk_0, lmk_1, lmk_2])
# p = np.array([2.2875, 0.0259, 0.9559])
# v =  unit_vec_from_lat_long(-2.8863, -0.2385)
# cam = Camera(p,v)
# omega_max = angular_velocity(cam, Q, D_OPT)
# print bls_omega(cam,Q, omega_max, D_OPT)

def trade_function(cam_i,Q_i,cam_j,Q_j,D_OPT):
	result = False

	list_i = []
	list_j = []

	for lmk in Q_i.list():
		if (lmk.vision(cam_j,D_OPT) > lmk.vision(cam_i,D_OPT)):
			result = True
			list_j.append(lmk)
		else:
			list_i.append(lmk)

	for lmk in Q_j.list():
		if (lmk.vision(cam_i,D_OPT) > lmk.vision(cam_j,D_OPT)):
			result = True
			list_i.append(lmk)
		else:
			list_j.append(lmk)

	return result, Landmark_list(list_i), Landmark_list(list_j)


# # Test

# D_OPT = 3
# p_0 =  np.array([2.1840, -1.8528, 2.0772])
# p_1 =  np.array([-2.2062, 1.8710, 2.1111])
# v_0 = unit_vec_from_lat_long(2.49,-0.53)
# v_1 = unit_vec_from_lat_long(-0.69, -0.54)
# cam_0 = Camera(p_0,v_0)
# cam_1 = Camera(p_1,v_1)
# l = 0.5
# u_l = unit_vec_from_lat_long(-pi/2,0)
# u_r = unit_vec_from_lat_long(pi/2,0)
# u_u = unit_vec_from_lat_long(0,pi/2)
# u_b = unit_vec_from_lat_long(pi,0)
# u_f = unit_vec_from_lat_long(0,0)
# lmk_l_0 = Landmark([0., -l/2, l/2],u_l)
# lmk_l_1 = Landmark([-l/3,-l/2, 3./4.*l],u_l)
# lmk_l_2 = Landmark([-l/4, -l/2,  l/3],u_l)
# lmk_r_0 = Landmark([0., l/2, l/2],u_r)
# lmk_r_1 = Landmark([-l/4, l/2, 3/4*l],u_r)
# lmk_r_2 = Landmark([-l/3, l/2, l/4 ],u_r)
# lmk_u_0 = Landmark([0., 0., l],u_u)
# lmk_u_1 = Landmark([-l/3, -l/3, l],u_u)
# lmk_u_2 = Landmark([-l/4, l/4, l],u_u)
# lmk_b_0 = Landmark([-l/2, 0., l/2],u_b)
# lmk_b_1 = Landmark([-l/2, l/3, l/4],u_b)
# lmk_b_2 = Landmark([-l/2, l/4, 2./3.*l],u_b)
# lmk_f_0 = Landmark([l/2, 0., l/2],u_f)
# lmk_f_1 = Landmark([l/2, l/3, 3./4.*l],u_f)
# lmk_f_2 = Landmark([l/2, l/4, 2./3.*l],u_f)
# Q_0 = Landmark_list([lmk_l_0,lmk_r_0,lmk_u_0,lmk_b_0,lmk_f_0])
# Q_1 = Landmark_list([lmk_l_1,lmk_r_1,lmk_u_1,lmk_b_1,lmk_f_1])
# Q_2 = Landmark_list([lmk_l_2,lmk_r_2,lmk_u_2,lmk_b_2,lmk_f_2])
# result, Q_a, Q_b = trade_function(cam_0,Q_0,cam_1,Q_1,D_OPT)

# for lmk in Q_a.list():
# 	if (lmk.vision(cam_1,D_OPT) > lmk.vision(cam_0,D_OPT)):
# 		print "ERROR"

# for lmk in Q_b.list():
# 	if (lmk.vision(cam_0,D_OPT) > lmk.vision(cam_1,D_OPT)):
# 		print "ERROR"		
