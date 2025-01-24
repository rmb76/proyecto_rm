#!/usr/bin/env python3

# Este codigo implementa una maquina de estados con SMACH que permite el manejo de la logica de
# distintos tipo de movimiento, llamadas a nodos y topicos y sincronizacion de tareas.

import rospy
import smach
import yaml
import os
import random
import actionlib
import time
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = '/scan'

# Estado 1: Se trata unicamente de un estado inicial para configurar el flujo
class Inicio(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_detectar_cara'])

    def execute(self, userdata):
        rospy.loginfo("Estado 1: Inicio. Transicionando al estado de detección de cara.")
        return 'to_detectar_cara'

# Estado 2: Este estado se ejecuta antes de empezar con la navegacion para asegurar
# que se trata de un usuario conocido. Hasta que no se detecta una cara, no transiciona
# al siguiente estado.
class DetectarCara(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_peticion_objetivo'])
        self.cara_detectada = False

        # Al publicar "start" en este topic, se llama al nodo face_detection_node, que abre la camara
        # y espera a reconocer a una persona conocida. Al publicar "stop" se cierra la camara y se para
        # de detectar.
        self.pub_control = rospy.Publisher('/face_recognition_control', String, queue_size=10)

    def callback_facedetection(self, msg):
        self.cara_detectada = msg.data
        rospy.loginfo(f"Callback: Cara detectada: {self.cara_detectada}")

    def execute(self, userdata):
        self.facedetection_sub = rospy.Subscriber('/faceDetection', String, self.callback_facedetection)
        rospy.loginfo("Estado 2: Detectando cara...")
        rospy.sleep(1)

        # Comienza el reconocimiento de un usuario conocido (persona 1, persona 2 o persona 3)
        self.pub_control.publish("start")
        rospy.sleep(1)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.cara_detectada:
                rospy.loginfo("Cara detectada. Transicionando a petición de objetivo.")
                self.pub_control.publish("stop")
                return 'to_peticion_objetivo'
            rospy.loginfo("Esperando detección de cara...")
            rate.sleep()

# Estado 4: Una vez se reconoce al usuario, se habilita su interfaz permitiendole elegir una
# habitacion destino de la casa. Asi pues, asigna un objetivo con unas coordenadas concretas.
# El punto final se envia como output para poder enviarlo al siguiente estado.
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

        # Dependiendo del objetivo recibido, se recogen las coordenadas que presentaba el turtlebot en
        # el topic /amcl_pose en el punto deseado de cada sala y se listan.
        objetivo = PoseStamped()
        if self.label == "salon":
            objetivo.pose.position.x = -2.3
            objetivo.pose.position.y = 4.0
        elif self.label == "comedor":
            objetivo.pose.position.x = 6.0
            objetivo.pose.position.y = -1.2
        elif self.label == "entrada":
            objetivo.pose.position.x = -6.9
            objetivo.pose.position.y = 3.36
        elif self.label == "cocina":
            objetivo.pose.position.x = -6.3
            objetivo.pose.position.y = 0.67
        elif self.label == "aseo":
            objetivo.pose.position.x = 2.0
            objetivo.pose.position.y = 4.44
        elif self.label == "habitacion":
            objetivo.pose.position.x = 4.7
            objetivo.pose.position.y = 1.0
        elif self.label == "pasillo":
            objetivo.pose.position.x = 1.2
            objetivo.pose.position.y = 0.29
        else:
            rospy.logwarn(f"Etiqueta desconocida: {self.label}. Volviendo a salon.")
            objetivo.pose.position.x = -2.3
            objetivo.pose.position.y = 4

        userdata.objetivo = objetivo
        rospy.loginfo(f"Objetivo recibido: {objetivo.pose.position}")
        self.label = None
        return 'to_navegacion'

# Estado 5: este estado recoge el punto del destino y comienza una trayectoria desde el punto actual
# del robot hasta el especificado. Para este movimiento se emplea el planificador del paquete move_goal.
# 
# Esta trayectoria se sigue durante 20 segundos y, una vez ha pasado ese tiempo, se aborta y transiciona al
# siguiente estado.
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

# Estado 3: Este estado se repite cada cierto tiempo para asegurar que la persona conocida que esta siguiendo al robot se encuentra
# a su alrededor todavia. En caso afirmativo, vuelve al estado de navegacion y sigue yendo al objetivo. En caso contrario, transiciona
# al siguiente estado con el objetivo de encontrar al usuario.
class VerificarCara(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_navegacion', 'to_movimiento_aleatorio'],
                             input_keys=['objetivo'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cara_detectada = False

        # Se vuelve a necesitar el nodo de reconocimiento de personas
        self.pub_control = rospy.Publisher('/face_recognition_control', String, queue_size=10)
        self.pub_lost = rospy.Publisher('/personaperdida', String, queue_size=10)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.position = 0
        self.orientation = 0

    def amcl_pose_callback(self, msg):
        # Para obtener posicion del robot
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
       
    def callback_facedetection(self, msg):
        self.cara_detectada = True
        rospy.loginfo(f"Cara detectada: {self.cara_detectada}")

    def execute(self, userdata):
        self.cara_detectada = False
        self.facedetection_sub = rospy.Subscriber('/faceDetection', String, self.callback_facedetection)
        rospy.sleep(1)
        rospy.loginfo("Estado VerificarCara: Realizando giro de 360 grados para buscar caras.")
        self.pub_control.publish("start")
        rospy.sleep(1)
        
        self.girar_360()
        if self.cara_detectada:
            rospy.loginfo("Cara detectada durante el giro. Replanificando al mismo objetivo.")
            self.pub_control.publish("stop")
            return 'to_navegacion'
        else:
            rospy.loginfo("Cara no detectada durante el giro. Cambiando a movimiento aleatorio.")
            self.pub_control.publish("stop")
            self.pub_lost.publish("perdida")
            self.client.cancel_all_goals()
            self.client.stop_tracking_goal()
            return 'to_movimiento_aleatorio'

    def girar_360(self):
        rospy.loginfo("Iniciando giro de 360 grados.")
        if self.orientation and self.position:
            # En primer lugar se obtiene el angulo actual
            _, _, yaw_actual = euler_from_quaternion([
                self.orientation.x,
                self.orientation.y,
                self.orientation.z,
                self.orientation.w
            ])
            
            # Entonces, se divide el giro en 4 giros de 90 grados para mayor precision
            for i in range(4):
                rospy.loginfo(f"Iniciando giro {i + 1} de 4.")
                
                # Se suman 90 grados (pi/2 radianes) al yaw
                yaw_nueva = yaw_actual + math.pi / 2
                
                # Se crea un nuevo cuaternnio y se configura como nuevo objetivo
                nuevo_cuaternio = quaternion_from_euler(0, 0, yaw_nueva)
                punto = PoseStamped()
                punto.pose.position.x = self.position.x
                punto.pose.position.y = self.position.y
                
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = punto.pose
                goal.target_pose.pose.orientation = Quaternion(*nuevo_cuaternio)
                self.client.send_goal(goal)

                # Esperar hasta que se complete el movimiento con move_goal de nuevo
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

# Estado 6: En caso de que el usuario se haya perdido, se emplea move_base para ir buscando a la persona
# por toda la casa. Para ello se activa el nodo de reconocimiento de cara durante la busqueda y se van
# asignando objetivos aleatorios para que navegue hacia ellos. Ademas, si transcurre mucho tiempo, se modifica
# este objetivo para que busque por una mayor area.
class MovimientoAleatorio(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['exit', 'failed'])

        script_dir = os.path.dirname(os.path.realpath(__file__))
        map_file = os.path.join(script_dir, 'rooms.yaml')  # Ruta al archivo YAML con las coordenadas limites de las salas

        # Se carga la configuracion de habitaciones desde el archivo YAML
        with open(map_file, 'r') as file:
            self.rooms = yaml.safe_load(file)['rooms']

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cara_detectada = False
        self.current_room = "Desconocido"

        self.pub_control = rospy.Publisher('/face_recognition_control', String, queue_size=10)
        self.pub_lost = rospy.Publisher('/personaperdida', String, queue_size=10)

        rospy.Subscriber('/faceDetection', String, self.callback_facedetection)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        rospy.loginfo("MovimientoAleatorio inicializado.")

    def execute(self, userdata):
        
        rospy.loginfo("Estado MovimientoAleatorio ejecutandose.")
        self.cara_detectada = False
        self.pub_control.publish("start")
        rospy.sleep(1)
        rospy.loginfo("Esperando a que el servidor move_base este disponible...")
        if not self.client.wait_for_server(rospy.Duration(5)):
            rospy.logerr("No se pudo conectar al servidor move_base.")
            return 'failed'

        rate = rospy.Rate(1)

        # En este bucle se asigna en primer lugar una habitacion aleatoria, que esta mapeada con sus coordenadas limite y
        # posteriormente se elige una coordenada aleatoria dentro de este limite. Se sigue esta logica debido a que, al no ser
        # un mapa rectangular, se asegura que el punto elegido no se encuentre fuera del mapa.

        while not rospy.is_shutdown() and not self.cara_detectada:
            # Seleccionar una habitacion aleatoria
            target_room = random.choice(list(self.rooms.keys()))
            rospy.loginfo(f"Cambiando a una nueva habitacion: {target_room}")

            room_bounds = self.rooms[target_room]

            # Generar un objetivo aleatorio dentro de la nueva habitacion
            rospy.loginfo("Generando objetivo aleatorio...")
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = random.uniform(room_bounds['x_min'], room_bounds['x_max'])
            goal.target_pose.pose.position.y = random.uniform(room_bounds['y_min'], room_bounds['y_max'])
            goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, random.uniform(-3.14, 3.14)))

            rospy.loginfo(f"Enviando objetivo aleatorio: x={goal.target_pose.pose.position.x}, y={goal.target_pose.pose.position.y}")
            self.client.send_goal(goal)

            # Se espera a que el robot alcance el objetivo o se detecte una cara
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                if self.cara_detectada:
                    rospy.loginfo("Cara detectada. Terminando estado.")

                    break

                state = self.client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Objetivo alcanzado. Cambiando de habitacion.")
                    break
                elif rospy.Time.now() - start_time > rospy.Duration(20):  # Tiempo maximo por objetivo
                    rospy.loginfo("Tiempo maximo alcanzado para el objetivo. Cancelando.")
                    self.client.cancel_goal()
                    break

                rate.sleep()

        # Se detiene la deteccion de caras
        self.pub_control.publish("stop")

        if self.cara_detectada:
            rospy.loginfo(f"Cara detectada. El robot esta en la habitacion: {self.current_room}")
            self.client.cancel_all_goals()
            self.client.stop_tracking_goal()
            self.pub_lost.publish(self.current_room)
            return 'exit'
        else:
            rospy.loginfo("MovimientoAleatorio terminado sin detectar caras.")
            return 'failed'

    def callback_facedetection(self, msg):
        self.cara_detectada = True
        rospy.loginfo("Cara detectada.")

    # Con este callback se conoce en todo momento el robot, asi como cuando encuentra al usuario.
    def pose_callback(self, data):

        """Callback para procesar la posicion del robot."""
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        # Determinar en que habitacion se encuentra
        self.current_room = "Desconocido"
        for room_name, bounds in self.rooms.items():
            if (bounds['x_min'] <= x <= bounds['x_max'] and
                    bounds['y_min'] <= y <= bounds['y_max']):
                self.current_room = room_name
                break

        rospy.loginfo(f"El robot esta en: {self.current_room}")

# Estado 7: En este ultimo estado, una vez se ha llegado al destino o se ha encontrado a una persona
# permite volver a repetir el proceso y elegir un nuevo destino. Con esto se consigue que el programa
# se pueda ejecutar cuantas veces se desee, siendo mas realista.
class DecisionUsuario(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_peticion_objetivo', 'exit'])

    def execute(self, userdata):
        rospy.loginfo("Estado 7: Decision del usuario.")
        decision = input("Deseas elegir un nuevo objetivo? (s/n): ")
        if decision.lower() == 's':
            return 'to_peticion_objetivo'
        else:
            rospy.loginfo("Finalizando el programa.")
            return 'exit'

# En el main unicamente se define la maquina de estados y se agregan los propios estados, sus transiciones y se
# definen en caso de haber variables de entrada o salida.
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
