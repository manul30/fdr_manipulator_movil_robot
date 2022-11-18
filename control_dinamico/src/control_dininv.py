#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from markers import *
from funciones import *
from roslib import packages

import rbdl


rospy.init_node("control_pdg")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual  = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])
# Archivos donde se almacenara los datos
fqact = open("/tmp/qactual.txt", "w")
fqdes = open("/tmp/qdeseado.txt", "w")
fxact = open("/tmp/xactual.txt", "w")
fxdes = open("/tmp/xdeseado.txt", "w")

# Nombres de las articulaciones
jnames = ['Revolucion1', 'Revolucion2', 'Revolucion3',
          'Revolucion4', 'Revolucion5', 'Revolucion6']
# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# =============================================================
# Configuracion articular inicial (en radianes)
q = np.array([0.0, -1.0, 1.1, -1.2, -1.6, 0.0])
# Velocidad inicial
dq = np.array([0., 0., 0., 0., 0., 0.])
# Aceleracion inicial
ddq = np.array([0., 0., 0., 0., 0., 0.])
# Configuracion articular deseada
qdes = np.array([1.0, 0.0, -0.3, 0.3, 0.5, -0.1])
# Velocidad articular deseada
dqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Aceleracion articular deseada
ddqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
ddxdes=np.array([ 0.0, 0.0, 0.0])
# =============================================================

# Posicion resultante de la configuracion articular deseada
xdes = fkine_kuka_kr4(qdes)[0:3,3]
# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel('../../kuka_kr4_description/urdf/kuka_kr4.urdf')
ndof   = modelo.q_size     # Grados de libertad
zeros = np.zeros(ndof)     # Vector de ceros

# Frecuencia del envio (en Hz)
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)

# Bucle de ejecucion continua
t = 0.0

# Se definen las ganancias del controlador
valores = 0.7*np.array([1.0, 1.0, 1.0])
Kp = np.diag(valores)
Kd = 2*np.sqrt(Kp)

# Inicializar variables
J_pas=jacobian_kr4(q)
e_dot = np.array([0., 0., 0.])
q_pas=q
e_pas=np.array([0., 0., 0.])

while not rospy.is_shutdown():

    # Leer valores del simulador
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Posicion actual del efector final
    x = fkine_kuka_kr4(q)[0:3,3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()
    
    print(q)
    # Almacenamiento de datos
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+'\n ')

    # ----------------------------
    # Control dinamico (COMPLETAR)
    # ----------------------------
    # Arrays numpy
    zeros = np.zeros(ndof) # Vector de ceros
    tau = np.zeros(ndof) # Para torque
    g = np.zeros(ndof) # Para la gravedad
    c = np.zeros(ndof) # Para el vector de Coriolis+centrifuga
    M = np.zeros([ndof, ndof]) # Para la matriz de inercia
    e = np.eye(6) # Vector identidad
    # Calculo de torque
    rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
    # Calculo de fuerzas por gravedad
    rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
    # Calculo de coriolis y centrifuga
    rbdl.InverseDynamics(modelo, q, dq, zeros, c)
    c = c - g
    # Calculo matriz de inercia
    for i in range(ndof):
        rbdl.InverseDynamics(modelo, q, zeros, e[i,:], M[i,:])
        M[i,:] = M[i,:] - g
    # Dinamica de error de segundo orden
    e = xdes - x # Calculo error posicion
    
    J = jacobian_kr4(q)
    invJ = np.linalg.pinv(J)

    
    e_dot=(e-e_pas)/dt
    J_dot=(J-J_pas)/dt
  
    # Calculo del torque
    #u = M.dot(invJ)
    #print(u)
    u = M.dot(invJ).dot(ddxdes - J_dot.dot(dq) + Kd.dot(e_dot) + Kp.dot(e)) + c + g
    # Actualización de variables
    J_pas=J
    e_pas=e

    

    # Simulacion del robot
    robot.send_command(u)

    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t+dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()

fqact.close()
fqdes.close()
fxact.close()
fxdes.close()
