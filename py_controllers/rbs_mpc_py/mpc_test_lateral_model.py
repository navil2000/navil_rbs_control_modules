import matplotlib.pyplot as plt
import numpy as np

from rbs_mpc import rbs_mpc 
from scipy import sparse

if __name__ == "__main__":
    
    # Simulación
    nsim = 200
    Ts = 0.1
    # Hiperparámetro
    q11 = 1000
    q22 = 1
    r   = 1000
    # Matrices del modelo, restricciones y demás
    Ad = sparse.csc_matrix([[1, Ts],
                        [0, 1]])

    Bd = sparse.csc_matrix([[(Ts**2/2)],
                        [ Ts]])
    # Condiciones inciales y restricciones
    u0 = 0
    umin = np.array([-10])
    umax = np.array([10])
    x0 = np.array([0, 0])
    xmin = np.array([-8, -4])
    xmax = np.array([8, 4])
    # Referencia de los estados
    xr = np.array([0.0, 0.0])#np.zeros(2)
    # q11 -> Afecta al seguimiento del primer estado
    # q22 -> Afecta al seguimiento del segundo estado
    Q = sparse.diags([q11, q22])  
    # r -> Afecta a las variaciones de consigna.
    R = r*sparse.eye(1)             
    # Horizonte de predicción
    N = 50 
    # Declaramos el controlador
    mpc_lateral = rbs_mpc(A=Ad,B=Bd,
                            u0=u0,umin=umin,umax=umax,
                            x0=x0,xmin=xmin,xmax=xmax,
                            Q=Q,R=R,N=N,xr=xr)
    # Referencia de desplazamiento lateral
    de_ref = 0
    # Listas para hacer los plots
    u_list = []
    x_list = []
    r_list = []

    for i in range(nsim):

        # Probamos a cambiar la referencia
        if i > 70 and i < 140:
            de_ref = 3.5
        else:
            de_ref = 0

        # Metemos la referencia en el mpc. 
        mpc_lateral.set_x_ref(np.array([de_ref, 0.0]))    
        # Resolvemos el mpc
        ctrl = mpc_lateral.mpc_move(x0)
        # Simulamos la planta
        x0 = Ad.dot(x0) + Bd.dot(ctrl)

        r_list.append(de_ref)
        u_list.append(ctrl)
        x_list.append(x0)

    tiempo = [Ts*i for i in range(nsim)]

    plt.plot(tiempo, x_list)
    plt.plot(tiempo, u_list)
    plt.plot(tiempo, r_list)
    plt.ylabel(f'de')
    plt.xlabel(f'tiempo')
    plt.legend([f'Respuesta de', f'Respuesta v_lat', f'Consigna de control', f'Referencia'])
    plt.title(f'q11 = 1000, q22 = 1, r = 1000')
    plt.show()
    plt.savefig("mpc_test_lateral_model.png")
