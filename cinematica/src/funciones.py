import numpy as np
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi


def dh(d, theta, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.
    """
    # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    T = np.round(np.array([[cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta)],
                           [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                           [0, sin(alpha), cos(alpha), d],
                           [0, 0, 0, 1]
                           ]), decimals=5)
    return T
    
    

def fkine_kuka_kr4(q):
    """22.5549
    Calcular la cinematica directa del robot KUKA kr4 AGILUS dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    # Longitudes (en metros)

    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T1 = dh(0.330,q[0], 0, pi/2)
    T2 = dh(0, -q[1]+pi/2, 0.290, 0)
    T3 = dh(0, -q[2], 0.02, pi/2)
    T4 = dh(0.310, q[3]+pi, 0, pi/2)
    T5 = dh(0, q[4]+pi, 0, pi/2)
    T6 = dh(0.075, q[5],0, 0)
    # Efector final con respecto a la base
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
    return T
def jacobian_kr4(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x6
    J = np.zeros((3,6))
    # Transformacion homogenea inicial (usando q)
    x=fkine_kuka_kr4(q)
    x=x[0:3,3]
    
    # Iteracion para la derivada de cada columna
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i] + delta
        # Transformacion homogenea luego del incremento (q+delta)
        dx = fkine_kuka_kr4(dq)
        dx=dx[0:3,3]
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        columna_i = 1/delta * (dx-x)
        J[0:3,i] =columna_i
        
    return J


def ikine_kr4(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo de newton
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001

    q  = copy(q0)
    for i in range(max_iter):
        # Main loop
        J = jacobian_kr4(q)
        f = fkine_kuka_kr4(q)
        f = f[0:3,3]
        # Error
        e = xdes-f
        # Actualización de q (método de Newton)
        q = q + np.dot(np.linalg.pinv(J), e)
        
        # Condición de término
        if (np.linalg.norm(e) < epsilon):
            break
        #if (i==max_iter-1):
        #    print("El algoritmo no llegó al valor deseado")
    
    return q