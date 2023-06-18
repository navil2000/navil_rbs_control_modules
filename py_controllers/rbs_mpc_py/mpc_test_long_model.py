from tracemalloc import start
from rbs_mpc import rbs_mpc
import time
import numpy as np
import scipy as sp
from scipy import sparse
import matplotlib.pyplot as plt

if __name__ == "__main__":

  # Simulación
  Ts = 0.1
  nsim = 500

  # Matrices del modelo, restricciones y demás
  Ad = sparse.csc_matrix([[1, Ts, (Ts**2/2)],
                          [0, 1,  Ts],
                          [0, 0,  1]])

  Bd = sparse.csc_matrix([[(Ts**3/6)],
                          [(Ts**2/2)],
                          [Ts]])

  u0 = 0
  umin = np.array([-10])
  umax = np.array([10])

  x0 = np.array([5, 0, 0])
  xmin = np.array([-8, -2, -8])
  xmax = np.array([8,  2,  8])

  xr = np.array([-2, 0.0, 0.0])

  Q = sparse.diags([1000, 5000, 1000])
  R = 500*sparse.eye(1)

  N = 50 

  mpc_lateral = rbs_mpc(A=Ad,B=Bd,u0=u0,umin=umin,umax=umax,x0=x0,xmin=xmin,xmax=xmax,Q=Q,R=R,N=N,xr=xr)

  r_list = []
  u_list = []
  x_list = []

  for _ in range(nsim):

      # Resolvemos el mpc
      ctrl = mpc_lateral.mpc_move(x0)
      # Simulamos la planta
      x0 = Ad.dot(x0) + Bd.dot(ctrl)

      r_list.append(xr)
      u_list.append(ctrl)
      x_list.append(x0)

  tiempo = [Ts*i for i in range(nsim)]

  plt.plot(tiempo, x_list)
  plt.plot(tiempo, u_list)
  plt.plot(tiempo, r_list)
  plt.show()
  plt.savefig("mpc_test_long_model.png")
