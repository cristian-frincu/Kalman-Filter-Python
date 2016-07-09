import numpy as np
EXTERNAL_INFLUENCE = False


Xk = np.matrix([0, 1]).T  # state vector [position, velocity]
Pk = np.matrix([[1, 0.5], [0.5, 1]])  # covariance matrix
delta_t = 1

# ------- Prediction -----------
Fk = np.matrix([[1, delta_t], [0, 1]])  # prediction step
# 1,1  So this means on each iteration, the position
# 0,1  will increase by the velocity * delta_t

if EXTERNAL_INFLUENCE:
    uk = [0, 0]  # [acceleration formula for position (pk),
    # acceleration formula for velocity(vk)]
    # when [0, 0] it becomes the same as if there was not external_influnece
    Bk = np.matrix([delta_t**2 / 2, delta_t])  # control matrix,
    # with the equations of motions regarding time change
    Xk = np.dot(Fk, Xk) + np.dot(Bk, uk) # Mean of prediction, given external influence
    Pk
else:
    Xk = np.dot(Fk, Xk)  # The mean of the prediction
    Pk = np.dot(np.dot(Fk, Pk), Fk.T)  # The covariance matrix of the preiction

# -------- /Prediction ---------


print Xk, '\n', Pk
# bel_priori = np.random.normal(bel_mean,bel_cov)
