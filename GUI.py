#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import tkinter as tk
from PIL import Image, ImageTk  # Para cargar imágenes en Tkinter

class TurtleBotInterface:
    def __init__(self, root):
        # Inicializar nodo ROS
        rospy.init_node('turtlebot_interface', anonymous=True)

        # Publicadores y suscriptores
        self.pub_decision = rospy.Publisher('/destino', String, queue_size=10)
        rospy.Subscriber('/faceDetection', String, self.face_detection_callback)
        rospy.Subscriber('/personaperdida', String, self.person_lost_callback)

        # Crear la interfaz gráfica
        self.root = root
        self.root.title("Interfaz TurtleBot2")
        self.root.geometry("600x500")  # Ajustar tamaño de la ventana

        # Cargar el mapa como fondo
        map_path = "/home/vil/robots_moviles_ws/src/user_gui/resources/map.png"        
        self.canvas = tk.Canvas(self.root, width=600, height=500)
        self.canvas.pack(fill="both", expand=True)

        # Cargar y mostrar el mapa
        self.map_image = Image.open(map_path)
        self.map_image = self.map_image.resize((600, 400))  # Ajustar tamaño
        self.map_tk = ImageTk.PhotoImage(self.map_image)
        self.canvas.create_image(0, 0, image=self.map_tk, anchor="nw")

        # Etiqueta para mensajes
        self.label = tk.Label(self.root, text="Esperando detección facial...", font=("Arial", 14), bg="white")
        self.label.place(x=150, y=20)

        # Botones para seleccionar destino
        self.button_salon = tk.Button(self.root, text="Salón", font=("Arial", 14), command=lambda: self.send_decision("salon"))
        self.button_salon.place(x=150, y=420)
        self.button_salon.config(state=tk.DISABLED)

        self.button_comedor = tk.Button(self.root, text="Comedor", font=("Arial", 14), command=lambda: self.send_decision("comedor"))
        self.button_comedor.place(x=350, y=420)
        self.button_comedor.config(state=tk.DISABLED)

        # LEDs (verde y rojo)
        self.led_green = self.canvas.create_oval(50, 420, 70, 440, fill="gray", outline="black")  # LED verde
        self.led_red = self.canvas.create_oval(100, 420, 120, 440, fill="gray", outline="black")  # LED rojo

        self.is_red_blinking = False  # Estado del parpadeo rojo

    def face_detection_callback(self, msg):
        """Actualiza la interfaz cuando se detecta una cara."""
        self.label.config(text="Cara detectada. Selecciona punto destino.")
        self.button_salon.config(state=tk.NORMAL)  # Habilitar botones
        self.button_comedor.config(state=tk.NORMAL)

    def send_decision(self, destination):
        """Publica el destino seleccionado y actualiza la interfaz."""
        self.pub_decision.publish(destination)
        self.label.config(text=f"Destino seleccionado: {destination}")
        self.button_salon.config(state=tk.DISABLED)
        self.button_comedor.config(state=tk.DISABLED)
        self.activate_green_led()

    def person_lost_callback(self, msg):
        """Callback para gestionar el estado de persona perdida."""
        if msg.data == "encontrada":
            self.activate_green_led()
        elif msg.data:
            self.activate_red_led_blinking()

    def activate_green_led(self):
        """Activa el LED verde y apaga el LED rojo."""
        self.canvas.itemconfig(self.led_green, fill="green")
        self.canvas.itemconfig(self.led_red, fill="gray")
        self.is_red_blinking = False  # Detener parpadeo del rojo

    def activate_red_led_blinking(self):
        """Activa el parpadeo del LED rojo y apaga el LED verde."""
        self.canvas.itemconfig(self.led_green, fill="gray")
        self.is_red_blinking = True
        self.blink_red_led()

    def blink_red_led(self):
        """Hace que el LED rojo parpadee."""
        if self.is_red_blinking:
            current_color = self.canvas.itemcget(self.led_red, "fill")
            new_color = "red" if current_color == "gray" else "gray"
            self.canvas.itemconfig(self.led_red, fill=new_color)
            self.root.after(500, self.blink_red_led)  # Cambia de color cada 500ms

def main():
    root = tk.Tk()
    app = TurtleBotInterface(root)
    try:
        root.mainloop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
