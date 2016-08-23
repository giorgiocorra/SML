import rospy

import numpy as np
norm = np.linalg.norm

import scipy
from scipy import linalg, matrix

from math import sqrt

def null(A, eps = 1e-15):
	u, s, vh = scipy.linalg.svd(A)
	null_mask = (s <= eps)
	null_space = scipy.compress(null_mask, vh, axis = 0)
	return (null_space)

# Test

# A = np.matrix([ [2,3,5],[-4,2,3], [0,0,0]])
# print(null(A))

def projection(x,n):
	n_ort = null(np.matrix([n, n.dot(0), n.dot(0)]))
	proj_coeff = x.dot(n_ort.transpose())
	return n_ort.transpose().dot(proj_coeff)

# Test

# x = np.array([3.,3.,4.])
# n = np.array([0.7071, 0, -0.7071])
# print projection(x,n)

def projection_cone(p,w,l):
	scalar = np.asscalar(w.dot(p))
	if (scalar >= 1.):				# any vector is ok
		rospy.logerr("Errore collision avoidance: p=%f, w=%f, l=%f"%(p,w,l))
	beta_plus = sqrt((1.-l**2)/(1.-scalar**2))
	beta_minus = -beta_plus
	alpha_plus = -beta_plus * scalar + l
	alpha_minus = -beta_minus * scalar + l
	proj_plus = w.dot(alpha_plus) + p.dot(beta_plus)
	proj_minus = w.dot(alpha_minus) + p.dot(beta_minus)
	return True, proj_plus, proj_minus

# Test
# vel = np.array([1,0,0])
# w = np.array([0.7071, 0, -0.7071])
# lambda_cone = -0.3
# print projection_cone(vel,w,lambda_cone)

def intersection_planes(a,b,lambda_1 = 0,lambda_2 = 0):
	u = null(np.array([a,b,a.dot(0)]))[0]
	i = np.argmax(abs(u))

	if (i == 0):
		o = np.array([0, -a[2]*lambda_2+b[2]*lambda_1, -b[1]*lambda_1+a[1]*lambda_2]).dot(1 / (a[1]*b[2] - b[1]*a[2]))
	elif(i == 1):
		o = np.array([-a[2]*lambda_2+b[2]*lambda_1, 0, -b[0]*lambda_1+a[0]*lambda_2]).dot(1 / (a[0]*b[2] - b[0]*a[2]))
	else:
		o = np.array([-a[1]*lambda_2+b[1]*lambda_1, -b[0]*lambda_1+a[0]*lambda_2, 0]).dot(1 / (a[0]*b[1] - b[0]*a[1]))

	return u, o

# Test
# w_1 = np.array([0.7071, 0, -0.7071])
# w_2 = np.array([-0.7071, 0, -0.7071])
# lambda_1 = -0.2
# lambda_2 = -0.5
# print intersection_planes(w_1,w_2, lambda_1, lambda_2)

def intersection_cones(a, b, lambda_1, lambda_2):
	u, o = intersection_planes(a,b,lambda_1,lambda_2)
	scalar = np.asscalar(u.dot(o))
	delta = scalar**2 - (norm(o)**2 -1)
	if (delta < 0):
		return False,[],[]
	else:
		t_plus = -scalar + sqrt(delta)
		t_minus = -scalar - sqrt(delta)
		inters_1 = o + u.dot(t_plus)
		inters_2 = o + u.dot(t_minus)
		return True, inters_1, inters_2

# Test
# w_1 = np.array([ 0.92847669,  0.        , -0.37139068])
# w_2 = np.array([-0.54498835,  0.        , -0.83844362])
# lambda_1 = -0.30813185
# lambda_2 = -0.77074418
# print intersection_cones(w_1,w_2, lambda_1, lambda_2)

def wall_directions(p, p_others, P_MIN = [-3,-3,0], P_MAX = [3,3,4], DELTA_WALLS = [0.5,0.5,0.3], DELTA_OTHERS = 2):
	W = []										# as a list
	# Walls check
	for i in range(len(p)):
		if (p[i] >= P_MAX[i] - DELTA_WALLS[i]):
			w = np.zeros(len(p))
			w[i] = -1
			W.append(w)
		elif (p[i] <= P_MIN[i] + DELTA_WALLS[i]):
			w = np.zeros(len(p))
			w[i] = 1
			W.append(w)
	# Other quads check
	for i in range(len(p_others)):
		if (norm(p-p_others[i]) <= DELTA_OTHERS):
			w = (p-p_others[i]).dot(1/norm(p-p_others[i]))
			W.append(w)
	return np.array(W)

# Test

# p = np.array([0,0,0])
# p_others = np.array([[-1,0,1], [1,0,1]])
# W = wall_directions(p,p_others)
# print(W)

def cone_width(d, delta_distance, delta_worry):
	l = -(d - delta_distance)/delta_worry
	if (l > 0.):
		l = 0.
	return l

def wall_directions_smooth(p, p_others, P_MIN = [-3,-3,0], P_MAX = [3,3,4], DELTA_WALLS = [0.5,0.5,0.3], DELTA_OTHERS = 2, DELTA_WORRY = 0.5):
	W = []										# as a list
	Lambda = []
	# Walls check
	for i in range(len(p)):
		if (p[i] >= P_MAX[i] - (DELTA_WALLS[i] + DELTA_WORRY)):
			w = np.zeros(len(p))
			w[i] = -1
			d = P_MAX[i] - p[i]
			W.append(w)
			Lambda.append(cone_width(d,DELTA_WALLS[i],DELTA_WORRY))
		elif (p[i] <= P_MIN[i] + (DELTA_WALLS[i] + DELTA_WORRY)):
			w = np.zeros(len(p))
			w[i] = 1
			d = p[i] - P_MIN[i]
			W.append(w)
			Lambda.append(cone_width(d,DELTA_WALLS[i],DELTA_WORRY))
	# Other quads check
	for i in range(len(p_others)):
		d = norm(p-p_others[i])
		if (d==0):					# just for when i start the simulators
			continue
		elif(d <= DELTA_OTHERS + DELTA_WORRY):
			w = (p-p_others[i]).dot(1 / d)
			W.append(w)
			Lambda.append(cone_width(d,DELTA_OTHERS,DELTA_WORRY))
	return np.array(W), np.array(Lambda)

# Test

# p = np.array([0,0,0])
# p_others = np.array([[-2,0,0.8], [1.3,0,2]])
# W, Lambda = wall_directions_smooth(p,p_others)
# print (W, Lambda)


def collision_avoidance_vel(p_dot_max, W):
	if (not W.any()) or (all(W.dot(p_dot_max) >= 0)):
		return p_dot_max/norm(p_dot_max)
	max = 0
	p_dot_star = p_dot_max.dot(0)				# just to have the same size
	for i in range(len(W)):
		proj = projection(p_dot_max, W[i])
		if all(W.dot(proj) >= 0) and (proj.dot(p_dot_max) >= max):
			p_dot_star = proj
			max = proj.dot(p_dot_max)
		for j in range(i+1,len(W)):
			intersection = intersection_planes(W[i],W[j])
			if (intersection.dot(p_dot_max) <= 0):
				intersection = -intersection
			if all(W.dot(intersection) >= 0) and (intersection.dot(p_dot_max) >= max):
				p_dot_star = intersection
				max = intersection.dot(p_dot_max)
	return p_dot_star

# Test
# W = np.array([[ 0., 0., 1.], [ 0.70710678,  0., -0.70710678], [-0.70710678,  0., -0.70710678]])
# p_dot_max = np.array([0,0,1])
# print(collision_avoidance_vel(p_dot_max, W))

def collision_avoidance_vel_smooth(p_dot_max, W, Lambda):
	EPSILON = np.array([-1e-4]*len(Lambda))

	if not(p_dot_max.any()):
		return p_dot_max

	if (not W.any()) or (all(W.dot(p_dot_max) - Lambda >= EPSILON)):
		return p_dot_max/norm(p_dot_max)

	max = 0
	p_dot_star = p_dot_max.dot(0)

	for i in range(len(W)):
		if Lambda[i]>0:
			rospy.logerr('Quads are too close!')
			return p_dot_max.dot(0)
		result, proj_1, proj_2 = projection_cone(p_dot_max, W[i], Lambda[i])
		proj_1 = proj_1.dot( 1 / norm(proj_1))
		proj_2 = proj_2.dot( 1 / norm(proj_2))
		if all(W.dot(proj_1) - Lambda >= EPSILON) and (proj_1.dot(p_dot_max) >= max):
			p_dot_star = proj_1
			max = proj_1.dot(p_dot_max)
		if all(W.dot(proj_2) - Lambda >= EPSILON) and (proj_2.dot(p_dot_max) >= max):
			p_dot_star = proj_2
			max = proj_2.dot(p_dot_max)

		for j in range(i+1,len(W)):
			exist, inters_1, inters_2 = intersection_cones(W[i],W[j], Lambda[i], Lambda[j])
			if not (exist):
				continue
			if (inters_1.dot(p_dot_max) <= 0):
				inters_1 = -inters_1
			if (inters_2.dot(p_dot_max) <= 0):
				inters_2 = -inters_2
			if all(W.dot(inters_1) - Lambda >= EPSILON) and (inters_1.dot(p_dot_max) >= max):
				p_dot_star = inters_1
				max = inters_1.dot(p_dot_max)
			if all(W.dot(inters_2) - Lambda >= EPSILON) and (inters_2.dot(p_dot_max) >= max):
				p_dot_star = inters_2
				max = inters_2.dot(p_dot_max)
	return p_dot_star

# Test
# p = np.array([0,0,1])
# p_dot_max = np.array([0,0,1])
# p_others = np.array([[-1.5,0.1,2.5], [1.5,0.1,2.5]])
# W, Lambda = wall_directions_smooth(p,p_others)
# print collision_avoidance_vel_smooth(p_dot_max, W, Lambda)
