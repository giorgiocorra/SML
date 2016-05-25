import matplotlib.pyplot as plt
import numpy as np
import sys

from scipy import signal

filename = sys.argv[1]

print "Plot of "+ filename

h_file = open(filename,"r")


print "Extract data from file..."
print 
data = []
with open(filename,"r") as h_file:
	lines = h_file.readlines()
	for l in lines:
		d = l.split(" ")
		try:
			v = [float(u) for u in d]
			data.append(v)
		except Exception,e:
			print l[0:-1]



data.pop()


   
var = ['time','x','y','z','vx','vy','vz','xd','yd','zd','vxd','vyd','vzd','r','p','w','f1','f2','f3']

def get_data(v):
	id_data = var.index(v)
	return np.array([d[id_data] for d in data])


if True:
	x = get_data('x')
	y = get_data('y')
	xd = get_data('xd')
	yd = get_data('yd')

	plt.plot(x,y)
	plt.plot(xd,yd)
	plt.show()

if True:
	t = get_data('time')
	vx = get_data('vx')
	vxd = get_data('vxd')
	vy = get_data('vy')
	vyd = get_data('vyd')

	vx = signal.medfilt(vx,[9])
	vy = signal.medfilt(vy,[9])


	plt.plot(t,vx,'b')
	plt.plot(t,vxd,'b')


	plt.plot(t,vy,'r')
	plt.plot(t,vyd,'r')
	plt.show()

if True:
	f1 = get_data('f1')
	f2 = get_data('f2')
	f3 = get_data('f3')
	t = get_data('time')

	plt.plot(t,f1)
	plt.plot(t,f2)
	plt.plot(t,f3)
	plt.show()