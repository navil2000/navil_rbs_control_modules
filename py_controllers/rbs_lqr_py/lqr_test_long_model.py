import rbs_lqr
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":

    Ts = .1

    A = np.array([[1., Ts, Ts**2/2],
                  [0., 1., Ts],
                  [0., 0., 1]])
    
    B = np.array([[Ts**3/6],
                  [Ts**2/2],
                  [Ts]])

    C = np.array([0., 1., 0.])

    u = np.eye(1)

    x = np.array([[0.],
                  [0.],
                  [0.]])

    x_dot = np.array([[20.],
                      [16.],
                      [258.]])

    Q = 100*np.eye(3)

    R = 25*np.eye(1)

    tracking_lqr = rbs_lqr.rbs_lqr(A,B,Q,R)
    
    K_lqr = tracking_lqr.dlqr()
    print(K_lqr)
    
    x_states = []
    y_results = []

    for _ in range(500):
        
        # The model is not adaptative, so the LQR
        # will only act once.

        # We simulate the model
        x = A@x_dot + B@K_lqr@x_dot
        y = C@x
        x_dot = x

        x_states.append(x)
        y_results.append(y)

    plt.plot([x[0] for x in x_states])
    plt.plot([x[1] for x in x_states])
    plt.plot([x[2] for x in x_states])
    plt.show()
    plt.savefig("lqr_test_long_model.png")
