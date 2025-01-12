import face_recognition
import cv2
import numpy as np
from scipy.spatial import distance

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

# Captura de video
video_capture = cv2.VideoCapture(0)

while True:
    ret, frame = video_capture.read()
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Detectar y codificar caras
    face_locations = face_recognition.face_locations(rgb_frame)
    face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
        # Calcular distancias entre la cara detectada y los embeddings conocidos
        distances = [distance.euclidean(face_encoding, known) for known in known_face_encodings]
        min_distance = min(distances)

        if min_distance < 0.5:  # Umbral para reconocer una cara conocida
            name = known_face_names[distances.index(min_distance)]
        else:
            name = "Desconocido"

        # Dibujar rectángulo y nombre
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(frame, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    cv2.imshow("Video", frame)

    # Salir con la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()
