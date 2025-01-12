#!/usr/bin/env python3

import rospy
import smach
import yaml
import os
import random
import actionlib
import time
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = '/scan'

class Inicio(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_detectar_cara'])

    def execute(self, userdata):
        rospy.loginfo("Estado 1: Inicio. Transicionando al estado de detección de cara.")
        return 'to_detectar_cara'

class DetectarCara(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_peticion_objetivo'])
        self.cara_detectada = False
        self.pub_start = rospy.Publisher('/start_detection', String, queue_size=10)
        self.pub_stop = rospy.Publisher('/stop_detection', String, queue_size=10)

    def callback_facedetection(self, msg):
        self.cara_detectada = msg.data
        rospy.loginfo(f"Callback: Cara detectada: {self.cara_detectada}")

    def execute(self, userdata):
        self.facedetection_sub = rospy.Subscriber('/faceDetection', String, self.callback_facedetection)
        rospy.loginfo("Estado 2: Detectando cara...")
        rospy.sleep(1)  # Tiempo para inicializar la detección

        self.pub_start.publish("start")
        rospy.sleep(1)  # Tiempo para inicializar la detección

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.cara_detectada:
                rospy.loginfo("Cara detectada. Transicionando a petición de objetivo.")
                self.pub_stop.publish("stop")
                return 'to_peticion_objetivo'
            rospy.loginfo("Esperando detección de cara...")
            rate.sleep()

class PeticionObjetivo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_navegacion'], output_keys=['objetivo'])
        self.label = None
        self.goal_subscriber = rospy.Subscriber('/destino', String, self.callback)

    def callback(self, msg):
        self.label = msg.data
        rospy.loginfo(f"Etiqueta recibida: {self.label}")

    def execute(self, userdata):
        rospy.loginfo("Estado 4: Petición del objetivo.")

        while not rospy.is_shutdown() and not self.label:
            rospy.loginfo("Esperando a recibir una etiqueta del topic 'destino'...")
            rospy.sleep(1)

        objetivo = PoseStamped()
        if self.label == "salon":
            objetivo.pose.position.x = -2.3
            objetivo.pose.position.y = 4.0
        elif self.label == "comedor":
            objetivo.pose.position.x = 6.0
            objetivo.pose.position.y = -1.2
        else:
            rospy.logwarn(f"Etiqueta desconocida: {self.label}. Volviendo a salon.")
            objetivo.pose.position.x = -2.3
            objetivo.pose.position.y = 4

        userdata.objetivo = objetivo
        rospy.loginfo(f"Objetivo recibido: {objetivo.pose.position}")
        self.label = None
        return 'to_navegacion'

class Navegacion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_decision_usuario','to_verificar_cara'],
                             input_keys=['objetivo'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo("Estado 5: Navegación hacia el objetivo.")
        objetivo = userdata.objetivo
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = objetivo.pose
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Enviando objetivo: x={goal.target_pose.pose.position.x}, y={goal.target_pose.pose.position.y}")
        self.client.send_goal(goal)

        tiempo_inicio = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            state = self.client.get_state()

            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Se alcanzó el objetivo con éxito.")
                return 'to_decision_usuario'

            tiempo_actual = time.time()
            if tiempo_actual - tiempo_inicio >= 20:
                rospy.loginfo("20 segundos transcurridos. Abortando navegación para verificar detección de cara.")
                self.client.cancel_all_goals()
                return 'to_verificar_cara'

            rate.sleep()

# Nuevo Estado: Verificar Cara
class VerificarCara(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_navegacion', 'to_movimiento_aleatorio'],
                             input_keys=['objetivo'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cara_detectada = False
        self.pub_start = rospy.Publisher('/start_detection', String, queue_size=10)
        self.pub_stop = rospy.Publisher('/stop_detection', String, queue_size=10)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.position = 0
        self.orientation = 0

    def amcl_pose_callback(self, msg):
        # Obtener posición del robot
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        #rospy.loginfo("Orientación (cuaternión): x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        #          self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w)

    def callback_facedetection(self, msg):
        self.cara_detectada = True
        rospy.loginfo(f"Cara detectada: {self.cara_detectada}")

    def execute(self, userdata):
        self.cara_detectada = False
        self.facedetection_sub = rospy.Subscriber('/faceDetection', String, self.callback_facedetection)
        rospy.sleep(1)
        rospy.loginfo("Estado VerificarCara: Realizando giro de 360 grados para buscar caras.")
        self.pub_start.publish("start")
        rospy.sleep(1)  # Tiempo para inicializar la detección
        
        self.girar_360()
        if self.cara_detectada:
            rospy.loginfo("Cara detectada durante el giro. Replanificando al mismo objetivo.")
            self.pub_stop.publish("stop")
            return 'to_navegacion'
        else:
            rospy.loginfo("Cara no detectada durante el giro. Cambiando a movimiento aleatorio.")
            self.pub_stop.publish("stop")
            self.client.cancel_all_goals()
            self.client.stop_tracking_goal()
            return 'to_movimiento_aleatorio'

    def girar_360(self):
        rospy.loginfo("Iniciando giro de 360 grados.")
        if self.orientation and self.position:
            # Obtener el ángulo actual del cuaternión
            _, _, yaw_actual = euler_from_quaternion([
                self.orientation.x,
                self.orientation.y,
                self.orientation.z,
                self.orientation.w
            ])
            
            # Realizar 4 giros de 90 grados
            for i in range(4):
                rospy.loginfo(f"Iniciando giro {i + 1} de 4.")
                
                # Añadir 90 grados (pi/2 radianes) al yaw
                yaw_nueva = yaw_actual + math.pi / 2
                
                # Crear el nuevo cuaternión
                nuevo_cuaternio = quaternion_from_euler(0, 0, yaw_nueva)
                
                # Configurar el punto objetivo
                punto = PoseStamped()
                punto.pose.position.x = self.position.x
                punto.pose.position.y = self.position.y
                
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = punto.pose
                goal.target_pose.pose.orientation = Quaternion(*nuevo_cuaternio)
                
                # Enviar el objetivo
                self.client.send_goal(goal)

                # Esperar hasta que se complete el movimiento
                rate = rospy.Rate(10)
                while not rospy.is_shutdown():
                    state = self.client.get_state()
                    if state == actionlib.GoalStatus.SUCCEEDED:
                        rospy.loginfo(f"Giro {i + 1} completado.")
                        break
                
                # Actualizar el yaw actual al nuevo valor
                yaw_actual = yaw_nueva

            rospy.loginfo("El giro de 360 grados se ha completado correctamente.")
        else:
            rospy.loginfo("La posicion y la orientacion no se han obtenido correctamente")

# Estado 6: Movimiento Aleatorio utilizando explore_lite
class MovimientoAleatorio(smach.State):
    def __init__(self):
        # Define los posibles resultados del estado
        smach.State.__init__(self, outcomes=['exit', 'failed'])

        # Aquí inicializa tus variables y lógica específica
        script_dir = os.path.dirname(os.path.realpath(__file__))  # Directorio del script
        map_file = os.path.join(script_dir, 'rooms.yaml')  # Ruta al archivo YAML

        # Cargar configuración de habitaciones desde el archivo YAML
        with open(map_file, 'r') as file:
            self.rooms = yaml.safe_load(file)['rooms']

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cara_detectada = False
        self.current_room = "Desconocido"

        self.pub_start = rospy.Publisher('/start_detection', String, queue_size=10)
        self.pub_stop = rospy.Publisher('/stop_detection', String, queue_size=10)

        rospy.Subscriber('/faceDetection', String, self.callback_facedetection)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        rospy.loginfo("MovimientoAleatorio inicializado.")

    def execute(self, userdata):
        rospy.loginfo("Estado MovimientoAleatorio ejecutándose.")
        self.cara_detectada = False

        # Iniciar la detección de caras
        self.pub_start.publish("start")
        rospy.sleep(1)

        # Esperar a que el servidor de move_base esté disponible
        rospy.loginfo("Esperando a que el servidor move_base esté disponible...")
        if not self.client.wait_for_server(rospy.Duration(5)):
            rospy.logerr("No se pudo conectar al servidor move_base.")
            return 'failed'

        rate = rospy.Rate(1)  # Frecuencia de 1 Hz

        while not rospy.is_shutdown() and not self.cara_detectada:
            # Seleccionar una habitación aleatoria
            target_room = random.choice(list(self.rooms.keys()))
            rospy.loginfo(f"Cambiando a una nueva habitación: {target_room}")

            room_bounds = self.rooms[target_room]

            # Generar un objetivo aleatorio dentro de la nueva habitación
            rospy.loginfo("Generando objetivo aleatorio...")
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = random.uniform(room_bounds['x_min'], room_bounds['x_max'])
            goal.target_pose.pose.position.y = random.uniform(room_bounds['y_min'], room_bounds['y_max'])
            goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, random.uniform(-3.14, 3.14)))

            rospy.loginfo(f"Enviando objetivo aleatorio: x={goal.target_pose.pose.position.x}, y={goal.target_pose.pose.position.y}")
            self.client.send_goal(goal)

            # Esperar a que el robot alcance el objetivo o se detecte una cara
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                if self.cara_detectada:
                    rospy.loginfo("Cara detectada. Terminando estado.")
                    break

                state = self.client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Objetivo alcanzado. Cambiando de habitación.")
                    break
                elif rospy.Time.now() - start_time > rospy.Duration(20):  # Tiempo máximo por objetivo
                    rospy.loginfo("Tiempo máximo alcanzado para el objetivo. Cancelando.")
                    self.client.cancel_goal()
                    break

                rate.sleep()

        # Detener la detección de caras
        self.pub_stop.publish("stop")

        if self.cara_detectada:
            rospy.loginfo(f"Cara detectada. El robot está en la habitación: {self.current_room}")
            self.client.cancel_all_goals()
            self.client.stop_tracking_goal()
            return 'exit'
        else:
            rospy.loginfo("MovimientoAleatorio terminado sin detectar caras.")
            return 'failed'

    def callback_facedetection(self, msg):
        self.cara_detectada = True
        rospy.loginfo("Cara detectada.")

    def pose_callback(self, data):
        """Callback para procesar la posición del robot."""
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        # Determinar en qué habitación se encuentra
        self.current_room = "Desconocido"
        for room_name, bounds in self.rooms.items():
            if (bounds['x_min'] <= x <= bounds['x_max'] and
                    bounds['y_min'] <= y <= bounds['y_max']):
                self.current_room = room_name
                break

        rospy.loginfo(f"El robot está en: {self.current_room}")

# Estado 7: Decisión del Usuario
class DecisionUsuario(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_peticion_objetivo', 'exit'])

    def execute(self, userdata):
        rospy.loginfo("Estado 7: Decisión del usuario.")
        decision = input("¿Deseas elegir un nuevo objetivo? (s/n): ")
        if decision.lower() == 's':
            return 'to_peticion_objetivo'
        else:
            rospy.loginfo("Finalizando el programa.")
            return 'exit'


def main():
    rospy.init_node('turtlebot2_state_machine')

    sm = smach.StateMachine(outcomes=['exit'])

    sm.userdata.objetivo = None

    with sm:
        smach.StateMachine.add('INICIO', Inicio(), transitions={'to_detectar_cara': 'DETECTAR_CARA'})

        smach.StateMachine.add('DETECTAR_CARA', DetectarCara(), 
                               transitions={'to_peticion_objetivo': 'PETICION_OBJETIVO'})
        smach.StateMachine.add('PETICION_OBJETIVO', PeticionObjetivo(), 
                               transitions={'to_navegacion': 'NAVEGACION'},
                               remapping={'objetivo': 'objetivo'})
        smach.StateMachine.add('NAVEGACION', Navegacion(), 
                               transitions={'to_decision_usuario': 'DECISION_USUARIO',
                                            'to_verificar_cara': 'VERIFICAR_CARA'},
                               remapping={'objetivo': 'objetivo'})
        smach.StateMachine.add('VERIFICAR_CARA', VerificarCara(), 
                               transitions={'to_navegacion': 'NAVEGACION',
                                            'to_movimiento_aleatorio': 'MOVIMIENTO_ALEATORIO'},
                               remapping={'objetivo': 'objetivo'})
        smach.StateMachine.add('MOVIMIENTO_ALEATORIO', MovimientoAleatorio(), 
                       transitions={'exit': 'DECISION_USUARIO',
                                    'failed': 'PETICION_OBJETIVO'})
        smach.StateMachine.add('DECISION_USUARIO', DecisionUsuario(), 
                               transitions={'to_peticion_objetivo': 'PETICION_OBJETIVO', 'exit': 'exit'})
        
    outcome = sm.execute()

if __name__ == '__main__':
    main()
