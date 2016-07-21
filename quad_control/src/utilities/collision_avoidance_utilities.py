import rospy

import numpy as np
norm = np.linalg.norm

import scipy
from scipy import linalg, matrix

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

def intersection_planes(a,b):
	return null(np.array([a,b,a.dot(0)]))[0]

# Test
# w_1 = np.array([0.7071, 0, -0.7071])
# w_2 = np.array([-0.7071, 0, -0.7071])
# print intersection_planes(w_1,w_2)

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

def collision_avoidance_vel(p_dot_max, W):
	if (not W.any()) or (all(W.dot(p_dot_max) >= 0)):
		return p_dot_max
	max = 0
	p_dot_star = p_dot_max.dot(0)
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

# W = np.array([[ 0., 0., 1.], [ 0.70710678,  0., -0.70710678], [-0.70710678,  0., -0.70710678]])
# p_dot_max = np.array([0,0,1])
# print(collision_avoidance_vel(p_dot_max, W))