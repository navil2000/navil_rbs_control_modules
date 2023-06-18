import numpy as np

class rbs_lqr:

    def __del__(self):
        print("Turning off the controller...")

    def __init__(self, A_, B_, Q_, R_):
 
        # 1.To initialize the controller we need to define the state space matrix        
        # Model matrix
        self.A = A_
        self.B = B_
        # Weights matrix
        self.Q = Q_
        self.R = R_
        
        # 2. Now we have to check the type and dimensions of the matrix.
        self.update_matrix(A_, B_, Q_, R_, True)

    def update_matrix(self, a_, b_, q_, r_, print_bool=False):
        
        # 1. We may check the types             
        if print_bool: print("Checking matrix types...")
        check_types =   isinstance(a_, np.ndarray) and isinstance(b_, np.ndarray) and isinstance(q_, np.ndarray) and isinstance(r_, np.ndarray)
        if not check_types: raise TypeError("Missmatching matrix types...")
        
        # 2. Now, the matrix dimension should be coherent
        
        # 2.1
        # A number of columns must be the number of states. 
        # As well, the Q matrix must be a square matrix which
        # dimension is equivalent to the number of states of
        # the system.

        # 2.2 
        # B number of columns must be the number of command (u(t)) signals.
        # As well, R matrix must be a square matrix which dimension is equal
        # to the number of columns of B.

        if print_bool: print("Checking matrix dimension...")
        A_Q_check = a_.shape[1] == q_.shape[1]
        B_R_check = b_.shape[1] == r_.shape[1]
        print(b_.shape)
        print(b_.shape[1])
        print(b_.shape[0])
        if not A_Q_check or not B_R_check: 
            print(A_Q_check)
            print(B_R_check)
            raise ValueError("Missmatching matrix dimensions.")
        
        # 3.This is equivalent to the constructor...
        # Model matrix
        self.A = a_
        self.B = b_
        # Weights matrix
        self.Q = q_
        self.R = r_
    
    def dlqr(self, n_it=30):

        # 1. Calculate de Ricatti equation solution
        
        # 1.1 We give an initial value for P_inf
        P_inf = 3.5*np.eye(self.Q.shape[1])
        K_inf = 3.5*np.ones(self.R.shape[1])
        # 1.2 We iterate n_it times until we get an acceptable value of P_inf
        for _ in range(n_it):
            K_inf = -np.linalg.inv(np.transpose(self.B)@P_inf@self.B + self.R)@np.transpose(self.B)@P_inf@self.A
            P_inf = np.transpose(self.A)@P_inf@self.A + np.transpose(self.A)@P_inf@self.B@K_inf + self.Q

        return K_inf
















































