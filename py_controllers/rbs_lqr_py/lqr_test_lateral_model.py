import rbs_lqr
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":

    nsim = 500
    Ts = .1

    A = np.array([[1., Ts],
                  [0., 1.]])
    
    B = np.array([[Ts**2/2],
                  [Ts]])

    C = np.array([1., 0.])

    u = np.eye(1)

    x = np.array([[0.],
                  [0.]])

    x_dot = np.array([[20.],
                      [3.]])

    Q = 100*np.eye(2)

    R = 25*np.eye(1)

    tracking_lqr = rbs_lqr.rbs_lqr(A,B,Q,R)
    
    K_lqr = tracking_lqr.dlqr()
    print(K_lqr)
    
    y_results = []
    x_states = []

    for _ in range(nsim):
        
        # The model is not adaptative, so the LQR
        # will only act once.

        # We simulate the model
        x = A@x_dot + B@K_lqr@x_dot
        y = C@x
        x_dot = x

        x_states.append(x)
        y_results.append(y)

    tiempo = [Ts*i for i in range(nsim)]

    plt.plot(tiempo, [x[0] for x in x_states])
    plt.plot(tiempo, [x[1] for x in x_states])
    plt.plot(tiempo, [0 for _ in range(nsim)])
    plt.show()
    plt.savefig("lqr_test_lateral_model.png")    

