#!/usr/bin/env python3

# Este codigo lanza la interfaz del usuario; es decir, la interfaz que tendria el turtlebot 
# y permite al usuario decidir a donde quiere ir. 

# Al estar orientado a personas ancianas, se ha decidido emplear una estructura simple donde se muestra
# el mapa de la casa y botones con el nombre de cada sala para que unicamente tenga que decidir el destino.
# Tambien se han insertado dos LEDs, uno verde en caso de un buen comportamiento y uno rojo en caso de que 
# el usuario de haya perdido y el turtlebot se encuentre buscandolo.

import rospy
from std_msgs.msg import String
import tkinter as tk
from PIL import Image, ImageTk  # Para cargar imagenes en Tkinter

class TurtleBotInterface:
    def __init__(self, root):

        rospy.init_node('turtlebot_interface', anonymous=True)

        # Publicadores y suscriptores
        self.pub_decision = rospy.Publisher('/destino', String, queue_size=10)
        rospy.Subscriber('/faceDetection', String, self.face_detection_callback)
        rospy.Subscriber('/personaperdida', String, self.person_lost_callback)
        rospy.Subscriber('/goalReached', String, self.destination_reached_callback)  # Nuevo suscriptor

        # Crear la interfaz gráfica
        self.root = root
        self.root.title("Interfaz TurtleBot")
        self.root.geometry("1024x768")  # Ajustar tamaño de la ventana para que todo se vea directamente

        # Cargar el mapa como fondo
        map_path = "/home/vil/robots_moviles_ws/src/user_gui/resources/map.png"        
        self.canvas = tk.Canvas(self.root, width=1024, height=600)
        self.canvas.pack(fill="both", expand=True)

        # Cargar y mostrar el mapa
        self.map_image = Image.open(map_path)
        self.map_image = self.map_image.resize((1024, 600))  # Ajustar tamaño
        self.map_tk = ImageTk.PhotoImage(self.map_image)
        self.canvas.create_image(0, 0, image=self.map_tk, anchor="nw")

        # Etiqueta para mensajes
        self.label = tk.Label(self.root, text="Esperando detección facial...", font=("Arial", 14), bg="white")
        self.label.place(relx=0.5, rely=0.05, anchor="center")

        # Contenedor para los botones
        button_frame = tk.Frame(self.root, bg="lightgray")
        button_frame.pack(fill="x", side="bottom")

        # Botones para seleccionar destino
        self.button_salon = tk.Button(button_frame, text="Salon", font=("Arial", 12), command=lambda: self.send_decision("salon"))
        self.button_salon.grid(row=0, column=1, padx=20, pady=20)
        self.button_salon.config(state=tk.DISABLED)

        self.button_comedor = tk.Button(button_frame, text="Comedor", font=("Arial", 12), command=lambda: self.send_decision("comedor"))
        self.button_comedor.grid(row=0, column=2, padx=20, pady=20)
        self.button_comedor.config(state=tk.DISABLED)

        self.button_entrada = tk.Button(button_frame, text="Entrada", font=("Arial", 12), command=lambda: self.send_decision("entrada"))
        self.button_entrada.grid(row=1, column=0, padx=20, pady=20)
        self.button_entrada.config(state=tk.DISABLED)

        self.button_aseo = tk.Button(button_frame, text="Aseo", font=("Arial", 12), command=lambda: self.send_decision("aseo"))
        self.button_aseo.grid(row=1, column=1, padx=20, pady=20)
        self.button_aseo.config(state=tk.DISABLED)

        self.button_cocina = tk.Button(button_frame, text="Cocina", font=("Arial", 12), command=lambda: self.send_decision("cocina"))
        self.button_cocina.grid(row=1, column=2, padx=20, pady=20)
        self.button_cocina.config(state=tk.DISABLED)

        self.button_habitacion = tk.Button(button_frame, text="Habitacion", font=("Arial", 12), command=lambda: self.send_decision("habitacion"))
        self.button_habitacion.grid(row=1, column=3, padx=20, pady=20)
        self.button_habitacion.config(state=tk.DISABLED)

        self.button_pasillo = tk.Button(button_frame, text="Pasillo", font=("Arial", 12), command=lambda: self.send_decision("pasillo"))
        self.button_pasillo.grid(row=1, column=4, padx=20, pady=20)
        self.button_pasillo.config(state=tk.DISABLED)

        # LEDs (verde y rojo) para comprobar funcionamiento
        self.led_green = self.canvas.create_oval(50, 620, 70, 640, fill="gray", outline="black")  # LED verde
        self.led_red = self.canvas.create_oval(100, 620, 120, 640, fill="gray", outline="black")  # LED rojo

        self.is_red_blinking = False  # Estado del parpadeo rojo

    # Los botones se habilitan unicamente cuando se ha reconocido una cara para seguir con la logica de la
    # maquina de estados
    def face_detection_callback(self, msg):
        """Actualiza la interfaz cuando se detecta una cara."""
        self.label.config(text="Cara detectada. Selecciona punto destino.")
        self.button_salon.config(state=tk.NORMAL)  # Habilitar botones
        self.button_comedor.config(state=tk.NORMAL)
        self.button_entrada.config(state=tk.NORMAL)
        self.button_aseo.config(state=tk.NORMAL)
        self.button_cocina.config(state=tk.NORMAL)
        self.button_habitacion.config(state=tk.NORMAL)
        self.button_pasillo.config(state=tk.NORMAL)

    # Envia las coordenadas del destino seleccionado a un topic que lee el estado la mquina de estados para
    # guardar el punto objetivo. Despues se deshabilitan de nuevo todos los botones para asegurar un buen comportamiento.
    def send_decision(self, destination):
        """Publica el destino seleccionado y actualiza la interfaz."""
        self.pub_decision.publish(destination)
        self.label.config(text=f"Destino seleccionado: {destination}")
        self.button_salon.config(state=tk.DISABLED)
        self.button_comedor.config(state=tk.DISABLED)
        self.button_entrada.config(state=tk.DISABLED)
        self.button_aseo.config(state=tk.DISABLED)
        self.button_cocina.config(state=tk.DISABLED)
        self.button_habitacion.config(state=tk.DISABLED)
        self.button_pasillo.config(state=tk.DISABLED)
        self.activate_green_led()

    # Lee el topic y si se ha perdido el usuario, cambia de LED.
    def person_lost_callback(self, msg):
        """Callback para gestionar el estado de persona perdida."""
        if msg.data != "perdida":
            self.activate_green_led()
        else:
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

    # Cuando se alcanza el objetivo asignado, se vuelven a habilitar los botones para elegir uno nuevo.
    def destination_reached_callback(self, msg):
        """Callback para cuando el robot llega al destino."""
        self.label.config(text="Destino alcanzado. Selecciona un nuevo destino.")
        self.button_salon.config(state=tk.NORMAL)  # Reactivar botones
        self.button_comedor.config(state=tk.NORMAL)
        self.button_entrada.config(state=tk.NORMAL)
        self.button_aseo.config(state=tk.NORMAL)
        self.button_cocina.config(state=tk.NORMAL)
        self.button_habitacion.config(state=tk.NORMAL)
        self.button_pasillo.config(state=tk.NORMAL)
        self.canvas.itemconfig(self.led_green, fill="gray")  # Apagar LED verde
        self.canvas.itemconfig(self.led_red, fill="gray")    # Apagar LED rojo
        self.is_red_blinking = False  # Asegurarse de que el parpadeo se detenga

def main():
    root = tk.Tk()
    app = TurtleBotInterface(root)
    try:
        root.mainloop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
