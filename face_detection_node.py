#!/usr/bin/env python3

import face_recognition
import cv2
import numpy as np
from scipy.spatial import distance
import rospy
from std_msgs.msg import String

# Cargar imágenes de cada persona
persona1_images = [
    face_recognition.load_image_file("persona1_1.jpg"),
    face_recognition.load_image_file("persona1_2.jpg"),
    face_recognition.load_image_file("persona1_3.jpg")
]

persona2_images = [
    face_recognition.load_image_file("persona2_1.jpg"),
    face_recognition.load_image_file("persona2_2.jpg"),
    face_recognition.load_image_file("persona2_3.jpg")
]

persona3_images = [
    face_recognition.load_image_file("persona3_1.jpg"),
    face_recognition.load_image_file("persona3_2.jpg"),
    face_recognition.load_image_file("persona3_3.jpg")
]

# Generar embeddings promedio para cada persona
def calcular_encoding_promedio(images):
    encodings = []
    for img in images:
        encoding = face_recognition.face_encodings(img)
        if len(encoding) > 0:
            encodings.append(encoding[0])
    if len(encodings) > 0:
        return np.mean(encodings, axis=0)
    else:
        return None

persona1_mean_encoding = calcular_encoding_promedio(persona1_images)
persona2_mean_encoding = calcular_encoding_promedio(persona2_images)
persona3_mean_encoding = calcular_encoding_promedio(persona3_images)

# Lista de embeddings y nombres conocidos
known_face_encodings = [persona1_mean_encoding, persona2_mean_encoding, persona3_mean_encoding]
known_face_names = ["Persona 1", "Persona 2", "Persona 3"]

def main():
    # Inicializar el nodo de ROS
    rospy.init_node('face_detection_node', anonymous=True)

    # Crear el publicador para el tópico /faceDetection
    pub = rospy.Publisher('/faceDetection', String, queue_size=10)

    # Captura de video desde la cámara
    video_capture = cv2.VideoCapture(0)

    if not video_capture.isOpened():
        rospy.logerr("No se pudo abrir la cámara del ordenador.")
        return

    rospy.loginfo("Nodo de detección facial iniciado.")

    while not rospy.is_shutdown():
        ret, frame = video_capture.read()
        if not ret:
            rospy.logwarn("No se pudo leer el frame de la cámara.")
            continue

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Detectar y codificar caras
        face_locations = face_recognition.face_locations(rgb_frame)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

        detected = "No Face Detected"
        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            distances = [distance.euclidean(face_encoding, known) for known in known_face_encodings]
            min_distance = min(distances)

            if min_distance < 0.5:
                name = known_face_names[distances.index(min_distance)]
                detected = f"Face Detected: {name}"
            else:
                name = "Desconocido"

            # Dibujar rectángulo y nombre
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(frame, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        pub.publish(detected)
        rospy.loginfo(detected)

        # Mostrar el frame (opcional para depuración)
        cv2.imshow("Detección Facial", frame)

        # Salir con la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
