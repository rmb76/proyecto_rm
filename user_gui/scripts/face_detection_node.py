#!/usr/bin/env python3

# Este codigo lanza el nodo que abre la camara para reconocer a un usuario conocido. En el caso real deberia 
# emplear la camara del turtlebot (usada en la interfaz del operador) y tener una base de datos mayor.
# Dado que esto es una simulacion, hemos recogido dos fotos de cada uno de los integrantes del proyecto y se detecta mediante
# la camara del ordenador personal.

import rospy
import face_recognition
import cv2
import numpy as np
from scipy.spatial import distance
from std_msgs.msg import String


# Para el roconociento de cara se ha empleado la biblioteca face_recognition. Esta carga imagenes de personas conocidas
# y cada imagen se convierte en un vector de caracteristicas. Se han empleado dos imagenes por persona para no incrementar 
# mucho el tiempo de procesamiento; asi pues, se hace la media de ambos vectores y su etiqueta asociada.
# Por ultimo se compara cada fotograma de la camara del dispositivo y solo en caso de detectar una persona y que se conocida, 
# envia un mensaje al topic /faceDetection, empleado en la maquina de estados base.

class FaceRecognitionNode:
    def __init__(self):
        # Inicializar nodo ROS
        rospy.init_node('face_recognition_node', anonymous=True)

        self.control_subscriber = rospy.Subscriber('/face_recognition_control', String, self.control_callback)
        self.face_publisher = rospy.Publisher('/faceDetection', String, queue_size=10)

        self.running = False

        # Configurar vectores
        self.known_face_encodings = []
        self.known_face_names = []

        self.load_known_faces()

        # Configurar captura de video
        self.video_capture = None

    def load_known_faces(self):
        # Cargar imagenes y calcular embeddings (vectores de caracteristicas de la imagen)
        personas = {
            # En caso de probar en otro ordenador, habria que modificar el directorio de estas imagenes
            "Persona 1": ["/home/vil/robots_moviles_ws/src/user_gui/data/persona1_1.jpg", "/home/vil/robots_moviles_ws/src/user_gui/data/persona1_2.jpg"],
            "Persona 2": ["/home/vil/robots_moviles_ws/src/user_gui/data/persona2_1.jpg", "/home/vil/robots_moviles_ws/src/user_gui/data/persona2_2.jpg"],
            "Persona 3": ["/home/vil/robots_moviles_ws/src/user_gui/data/persona3_1.jpg", "/home/vil/robots_moviles_ws/src/user_gui/data/persona3_2.jpg"],
        }

        for name, images in personas.items():
            encodings = []
            for image_path in images:
                try:
                    img = face_recognition.load_image_file(image_path)
                    img_encodings = face_recognition.face_encodings(img)
                    if img_encodings:
                        encodings.append(img_encodings[0])
                    else:
                        rospy.logwarn(f"No se detectó ninguna cara en {image_path}.")
                except Exception as e:
                    rospy.logerr(f"Error al cargar la imagen {image_path}: {e}")

            if encodings:
                mean_encoding = np.mean(encodings, axis=0)
                self.known_face_encodings.append(mean_encoding)
                self.known_face_names.append(name)
            else:
                rospy.logwarn(f"No se pudieron cargar caras para {name}.")

    # Este nodo unicamente se ejecuta cuando se le envia desde la maquina de estados, un mensaje de "start" al topic /face_recognition_control
    # y se detiene cuando se le envia un mensaje "stop" a este mismo topic. Esto se hace para que no haya problemas con el flujo de la maquina
    # de estados
    def control_callback(self, msg):
        command = msg.data.lower()
        if command == "start":
            if not self.running:
                self.running = True
                rospy.loginfo("Comando recibido: iniciar deteccion facial.")
        elif command == "stop":
            if self.running:
                self.running = False
                rospy.loginfo("Comando recibido: detener deteccion facial.")
                
    def start_recognition(self):
        self.video_capture = cv2.VideoCapture(0)
        rospy.loginfo("Ventana de deteccion facial abierta.")
        while not rospy.is_shutdown():
            if self.running:
                ret, frame = self.video_capture.read()
                if not ret:
                    rospy.logwarn("No se pudo capturar el video.")
                    continue

                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # Detectar y codificar caras
                face_locations = face_recognition.face_locations(rgb_frame)
                face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

                for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                    distances = [distance.euclidean(face_encoding, known) for known in self.known_face_encodings]
                    min_distance = min(distances) if distances else float("inf")
                    
                    if min_distance < 0.5:  # Umbral ajustado
                        name = self.known_face_names[distances.index(min_distance)]
                        self.face_publisher.publish(name)
                    else:
                        name = "Desconocido"

                    # Dibujar rectangulo y nombre para mejorar visualizacion 
                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                    cv2.putText(frame, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

                # Mostrar el video
                cv2.imshow("Video", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.loginfo("Cerrando ventana manualmente.")
                    break
            else:
                rospy.loginfo_throttle(5, "Esperando comando para detectar...")

        self.stop_recognition()
    
    def stop_recognition(self):
        if self.video_capture:
            self.video_capture.release()
            self.video_capture = None
        cv2.destroyAllWindows()
        rospy.loginfo("Ventana de deteccion facial cerrada.")

    def run(self):
        rospy.loginfo("Nodo de reconocimiento facial en ejecucion. Esperando comandos...")
        self.start_recognition()

if __name__ == "__main__":
    try:
        node = FaceRecognitionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass