import matplotlib
import matplotlib.cm as cm
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt



import numpy as np
EXTERNAL_INFLUENCE = False


Xk = np.matrix([0.0, 1.0]).T  # state vector [position, velocity]
Pk = np.matrix([[1, 0.5], [1, 0.5]])  # covariance matrix
delta_t = 1

for i in range(5):
	# ------- Prediction -----------
	Fk = np.matrix([[1, delta_t], [0, 1]])  # prediction step
	# 1,1  So this means on each iteration, the position
	# 0,1  will increase by the velocity * delta_t

	if EXTERNAL_INFLUENCE:
	    uk = [0, 1]  # [acceleration formula for position (pk),
	    # acceleration formula for velocity(vk)]
	    # when [0, 0] it becomes the same as if there was not external_influnece
	    Bk = np.matrix([delta_t**2 / 2, delta_t])  # control matrix,
	    # with the equations of motions regarding time change
	    # additional uncertainty from the environment matrix
	    Qk = np.matrix([[1.0, 0.05], [0.05, 1.0]])
	    # Mean of prediction, given external influence
	    Xk = np.dot(Fk, Xk) + np.dot(Bk, uk)
	    Pk = np.dot(np.dot(Fk, Pk), Fk.T) + Qk
	else:
	    Xk = np.dot(Fk, Xk)  # The mean of the prediction
	    Pk = np.dot(np.dot(Fk, Pk), Fk.T)  # The covariance matrix of the preiction

	# -------- /Prediction ---------



	print "Xk:","\n",Xk
	print "Pk:","\n", Pk
	# print Pk[0,0]
	# print Pk[1,1]
	# print Pk[0,1]
	# print Xk[0,0]
	# print Xk[1,0]

	
	# -------- Display ---------
	delta = 0.025
	x = np.arange(0.0, 50.0, delta)
	y = np.arange(0.0, 10.0, delta)
	X, Y = np.meshgrid(x, y)
	Z1 = mlab.bivariate_normal(X, Y, Pk[0,0], Pk[1,1], Xk[0,0], Xk[1,0], Pk[0,1])
	Z = 10.0 * Z1


	plt.figure()
	CS = plt.contour(X, Y, Z)
	plt.clabel(CS, inline=1, fontsize=10)

	plt.show()
	# -------- /Display ---------
