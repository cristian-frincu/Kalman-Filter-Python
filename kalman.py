import matplotlib
import matplotlib.cm as cm
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv

# For our [positon, velocity] example, this would be applied acceelration
EXTERNAL_INFLUENCE = True
DISPLAY = False  # Want to see a graph of the estiamte?

Xk = np.matrix([1.0, 0.5]).T  # state vector [position, velocity]
Pk = np.matrix([[1.0, 0.25], [0.25, 1.0]])  # covariance matrix
delta_t = 1

ErrorBetweenSensor=0
ErrorBetweenPrediction=0

for i in range(1,1000):
    # ------- Prediction -----------
    Fk = np.matrix([[1.0, delta_t], [0.0, 1.0]])  # prediction step
    # 1,dT  So this means on each iteration, the position
    # 0,1  will increase by the velocity * delta_t

    if EXTERNAL_INFLUENCE:
        uk = [0, 0.25]  # [acceleration formula for position (pk),
        # acceleration formula for velocity(vk)]
        # when [0, 0] it becomes the same as if there was not
        # external_influnece
        Bk = np.matrix([delta_t**2 / 2, delta_t])  # control matrix,
        # with the equations of motions regarding time change
        # additional uncertainty from the environment matrix
        Qk = np.matrix([[1.0, 0.01], [0.01, 1.0]])
        # Mean of prediction, given external influence
        Xk = np.dot(Fk, Xk) + np.dot(Bk, uk)
        Pk = np.dot(np.dot(Fk, Pk), Fk.T) + Qk
    else:
        Xk = np.dot(Fk, Xk)  # The mean of the prediction
        # The covariance matrix of the preiction
        Pk = np.dot(np.dot(Fk, Pk), Fk.T)
    # -------- /Prediction ---------

    # -------- Measurments ---------
    Hk = np.matrix([[1.0, 0.5], [0.5, 1.0]])  # maps the sensor reading values to
    # the state vector values
    # mean of the expected measurment based on the predicted step
    # brings the values from the prediction to the values of the sensor
    Xk = np.dot(Hk, Xk)
    # covariance of the measurment based on the predicted step
    # Brings the values of the prediction to the values of sensor
    Pk = np.dot(np.dot(Hk, Pk), Hk.T)

    # #Sensor Values
    Sensor_reading = np.random.normal(i,i*0.25)
    Zk = np.mean([Sensor_reading, Sensor_reading])  # Mean value of sensors
    # # Covariance matrix of measurments
    Rk = np.matrix([[1.0, 0.25], [0.25, 1.0]])
    # -------- /Measurments ---------


    print "Xk_pre:", Xk[0,0]

    # # -------- Update Step ---------
    inovation = np.dot(np.dot(Hk, Pk), Hk.T) + Rk
    # # print inovation

    K = np.dot(np.dot(Pk, Hk.T), inv(inovation))
    Xk = Xk + np.dot(K, (Zk - np.dot(Hk, Xk)))
    Pk = Pk - np.dot(K, np.dot(Hk, Pk))

    # -------- /Update Step ---------
    print "Sensor:",Sensor_reading
    print "Real Value:",i
    # print "Zk:",Zk
    print "Xk_post:", Xk[0,0]
    print "--------"
    ErrorBetweenPrediction += np.abs(Xk[0,0]-i)
    ErrorBetweenSensor += np.abs(Sensor_reading-i)
    # print "Pk:", "\n", Pk
    # print Pk[0,0]
    # print Pk[1,1]
    # print Pk[0,1]
    # print Xk[0,0]
    # print Xk[1,0]

    # -------- Display ---------
    if DISPLAY:
        delta = 0.25
        x = np.arange(-3.0, 10.0, delta)
        y = np.arange(-3.0, 10.0, delta)
        X, Y = np.meshgrid(x, y)
        Z1 = mlab.bivariate_normal(X, Y, Pk[0, 0], Pk[1, 1], Xk[
                                   0, 0], Xk[1, 0], Pk[0, 1])
        Z = 10.0 * Z1

        plt.figure()
        CS = plt.contour(X, Y, Z)
        # CS = plt.pcolormesh(X, Y, Z)
        # plt.clabel(CS, inline=0, fontsize=10)

        plt.show()
    # -------- /Display ---------
print "Predict Error:",ErrorBetweenPrediction
print "Sensor Error:",ErrorBetweenSensor