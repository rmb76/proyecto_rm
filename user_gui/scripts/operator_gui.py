#!/usr/bin/env python3

# Este codigo lanza la interfaz que veria el operador desde la recepcion de la residencia o un familiar que supervise
# al usuario de avanzada edad. En esta interfaz se puede observar la camara del turtlebot para poder comprobar donde se encuentra,
# el estado del mismo (parado o en movimiento) y el objetivo al que se dirige en cada momento.
# Tambien conserva la logica de los dos LEDs donde el LED verde informa de un buen comportamiento y el LED rojo parpadeando 
# informa de que se encuentra buscando al usuario en caso de perdida.

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError
import tkinter as tk
from PIL import Image as PILImage, ImageTk
import cv2


class TurtleBotCameraInterface:
    def __init__(self, root):
        # Inicializar nodo ROS
        rospy.init_node('turtlebot_camera_interface', anonymous=True)

        # Publicadores y suscriptores
        self.pub_action = rospy.Publisher('/accion_usuario', String, queue_size=10)
        rospy.Subscriber('/camera/rgb/image_raw', ROSImage, self.camera_callback)
        rospy.Subscriber('/personaperdida', String, self.person_lost_callback)
        rospy.Subscriber('/destino', String, self.destination_callback)
        rospy.Subscriber('/goalReached', String, self.goal_reached_callback)

        # Variables de control
        self.bridge = CvBridge()
        self.current_frame = None
        self.movement_status = "Parado"
        self.destination = "Sin destino"

        # Crear la interfaz grafica
        self.root = root
        self.root.title("Interfaz TurtleBot3 - Cámara")
        self.root.geometry("900x550")
        self.root.configure(bg="#f4f4f4")

        # En este caso, el titulo de la interfaz es tortuga 1 ya que en el uso en aplicacion real tndria que tener distintas ventanas
        # para los distintos Turtlebots
        self.title_label = tk.Label(self.root, text="Tortuga Uno", font=("Arial", 16, "bold"), bg="#f4f4f4")
        self.title_label.place(x=10, y=10, width=880, height=30)

        # Seccion de la camara (izquierda)
        self.camera_label = tk.Label(self.root, bg="black")
        self.camera_label.place(x=10, y=50, width=500, height=400)

        # Cuadro de estado del movimiento
        self.movement_frame = tk.LabelFrame(self.root, text="Estado del Movimiento", font=("Arial", 12), bg="#f4f4f4")
        self.movement_frame.place(x=530, y=50, width=350, height=100)
        self.movement_status_label = tk.Label(self.movement_frame, text=self.movement_status, font=("Arial", 14), bg="white", relief="ridge")
        self.movement_status_label.place(x=10, y=10, width=320, height=60)

        # Cuadro de destino
        self.destination_frame = tk.LabelFrame(self.root, text="Destino", font=("Arial", 12), bg="#f4f4f4")
        self.destination_frame.place(x=530, y=160, width=350, height=100)
        self.destination_status_label = tk.Label(self.destination_frame, text=self.destination, font=("Arial", 14), bg="white", relief="ridge")
        self.destination_status_label.place(x=10, y=10, width=320, height=60)

        # LEDs redondos
        self.led_green = tk.Canvas(self.root, width=30, height=30, bg="#f4f4f4", highlightthickness=0)
        self.led_green.place(x=660, y=270)
        self.led_green.create_oval(2, 2, 28, 28, fill="gray", outline="black", tags="led")

        self.led_red = tk.Canvas(self.root, width=30, height=30, bg="#f4f4f4", highlightthickness=0)
        self.led_red.place(x=710, y=270)
        self.led_red.create_oval(2, 2, 28, 28, fill="gray", outline="black", tags="led")

        # Botones dinamicos
        self.button_return = tk.Button(self.root, text="Vuelta al inicio", font=("Arial", 12), command=self.return_to_start)
        self.button_return.place(x=545, y=320, width=150, height=40)
        self.button_return.config(state=tk.DISABLED)

        self.button_continue = tk.Button(self.root, text="Continuar", font=("Arial", 12), command=self.continue_moving)
        self.button_continue.place(x=705, y=320, width=150, height=40)
        self.button_continue.config(state=tk.DISABLED)

        # Actualizacion del video
        self.update_camera()

    def camera_callback(self, data):
        """Recibe imagenes del topico de la camara y las almacena."""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Error en la conversion de la imagen: {e}")

    def update_camera(self):
        """Actualiza la vista de la camara en la interfaz."""
        if self.current_frame is not None:
            # Convertir a formato compatible con Tkinter
            frame_rgb = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
            resized_frame = cv2.resize(frame_rgb, (500, 400))  # Redimensionar la imagen a 500x400
            img = PILImage.fromarray(resized_frame)
            imgtk = ImageTk.PhotoImage(image=img)
            self.camera_label.imgtk = imgtk
            self.camera_label.configure(image=imgtk)
        self.root.after(33, self.update_camera)  # Aproximadamente 30 FPS

    # En caso de que la persona se pierda, el Turtlebot informa de si lo encuentra o permanece buscando y gestiona el funcionamiento
    # de los LEDs
    def person_lost_callback(self, msg):
        """Gestiona el estado de persona perdida."""
        if msg.data != "perdida":
            self.button_return.config(state=tk.NORMAL)
            self.button_continue.config(state=tk.NORMAL)
            self.destination = "Encontrado en " + msg.data
            self.destination_status_label.config(text=self.destination)
            self.movement_status = "Parado"
            self.movement_status_label.config(text=self.movement_status)
            self.activate_green_led()
        else:
            self.movement_status = "buscando persona"
            self.movement_status_label.config(text=self.movement_status)
            self.activate_red_led()

    def destination_callback(self, msg):
        """Actualiza el destino del robot."""
        self.destination = msg.data
        self.destination_status_label.config(text=self.destination)
        self.movement_status = "Moviendose"
        self.movement_status_label.config(text=self.movement_status)
        self.activate_green_led()

    def goal_reached_callback(self, msg):
        """Callback que maneja la llegada al destino."""
        self.movement_status = "Meta alcanzada"
        self.destination = "Sin destino"
        self.movement_status_label.config(text=self.movement_status)
        self.destination_status_label.config(text=self.destination)
        self.deactivate_leds()

    def return_to_start(self):
        """Accion para volver al inicio."""
        self.pub_action.publish("vuelta_inicio")
        self.reset_interface("Volviendo al inicio...")

    def continue_moving(self):
        """Accion para continuar movimiento."""
        self.pub_action.publish("continuar")
        self.reset_interface("Continuando...")

    def reset_interface(self, status_text):
        """Reinicia la interfaz después de una accion."""
        self.movement_status_label.config(text=status_text)
        self.button_return.config(state=tk.DISABLED)
        self.button_continue.config(state=tk.DISABLED)
        self.activate_green_led()

    def activate_green_led(self):
        """Enciende el LED verde y apaga el rojo."""
        self.led_green.itemconfig("led", fill="green")
        self.led_red.itemconfig("led", fill="gray")

    def activate_red_led(self):
        """Enciende el LED rojo y apaga el verde."""
        self.led_red.itemconfig("led", fill="red")
        self.led_green.itemconfig("led", fill="gray")

    def deactivate_leds(self):
        """Apaga ambos LEDs."""
        self.led_red.itemconfig("led", fill="gray")
        self.led_green.itemconfig("led", fill="gray")


def main():
    root = tk.Tk()
    app = TurtleBotCameraInterface(root)
    try:
        root.mainloop()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
