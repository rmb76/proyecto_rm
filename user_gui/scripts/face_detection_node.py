#!/usr/bin/env python3

import rospy
import face_recognition
import cv2
import numpy as np
from scipy.spatial import distance
from std_msgs.msg import String

class FaceRecognitionNode:
    def __init__(self):
        # Inicializar nodo ROS
        rospy.init_node('face_recognition_node', anonymous=True)

        # Suscriptor al tópico para control
        self.control_subscriber = rospy.Subscriber('/face_recognition_control', String, self.control_callback)

        # Publicador de nombres detectados
        self.face_publisher = rospy.Publisher('/faceDetection', String, queue_size=10)

        # Estado del reconocimiento facial
        self.running = False

        # Configurar embeddings conocidos
        self.known_face_encodings = []
        self.known_face_names = []

        self.load_known_faces()

        # Configurar captura de video
        self.video_capture = None

    def load_known_faces(self):
        # Cargar imágenes y calcular embeddings
        personas = {
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

    def control_callback(self, msg):
        command = msg.data.lower()
        if command == "start":
            if not self.running:
                self.running = True
                rospy.loginfo("Comando recibido: iniciar detección facial.")
        elif command == "stop":
            if self.running:
                self.running = False
                rospy.loginfo("Comando recibido: detener detección facial.")
                
    def start_recognition(self):
        self.video_capture = cv2.VideoCapture(0)
        rospy.loginfo("Ventana de detección facial abierta.")
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

                    # Dibujar rectángulo y nombre
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
        rospy.loginfo("Ventana de detección facial cerrada.")

    def run(self):
        rospy.loginfo("Nodo de reconocimiento facial en ejecución. Esperando comandos...")
        self.start_recognition()

if __name__ == "__main__":
    try:
        node = FaceRecognitionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass