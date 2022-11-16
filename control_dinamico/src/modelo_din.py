import rbdl
import numpy as np


# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('../../kuka_kr4_description/urdf/kuka_kr4.urdf')
# Grados de libertad
ndof = modelo.q_size

# Configuracion articular
q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5])

# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
g     = np.zeros(ndof)          # Para la gravedad
c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
e     = np.eye(6)               # Vector identidad

# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)

# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics

# vector de gravedad: g = ID(q,0,0)
rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
# vector de coriolis: c = (ID(q,dq,0)-g)/dq
rbdl.InverseDynamics(modelo, q, dq, zeros, c)
c = c-g
# matriz de inercia: M[1,:] = (ID(dq,0,e[1,:]) )/e[1,:]
for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i,:], M[i,:])
    M[i,:]=M[i,:]-g
print('Vector de gravedad')
print(np.round(g,3))
print('\n')
print('Vector de coriolis')
print(np.round(c,3))
print('\n')
print('Matriz de inercia')
print(np.round(M,2))


# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
# en los arreglos llamados M2 y b2
b2 = np.zeros(ndof)          # Para efectos no lineales
M2 = np.zeros([ndof, ndof])  # Para matriz de inercia

rbdl.CompositeRigidBodyAlgorithm(modelo, q, M2)
rbdl.NonlinearEffects(modelo, q, dq, b2)
print("\n Momento de inercia por CompositeRigidBodyAlgorithm:")
print(np.round(M2, 2))

print("\nEfectos no lineales: ")
print(np.round(b2, 2))


# Parte 2: Verificacion de valores

E_Inercia = M2-M
E_Coriolis = b2-(c+g)
# Error de la matriz de inercia
print("\n Error de la matriz de inercia: ")
print(np.round(E_Inercia))
# Error efectos no lineales
print("\nError del vector coriolis: ")
print(E_Coriolis)



# Parte 3: Verificacion de la expresion de la dinamica

tau1 = M.dot(ddq) + c + g
tau2 = M2.dot(ddq) + b2
error = tau1-tau2
print("\n Torque final (Metodo 1): ")
print(np.round(tau1, 1))
print("\n Torque final (Metodo 2): ")
print(np.round(tau2, 1))
print("\n Error:")
print(np.round(error))