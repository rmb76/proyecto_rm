#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import os

class BeepOnMessage:
    def __init__(self, topic_name):
        rospy.init_node('beep_on_message', anonymous=True)
        rospy.Subscriber(topic_name, String, self.callback)
        rospy.loginfo(f"Escuchando el tópico: {topic_name}")

    def callback(self, msg):
        """Callback que emite un pitido cuando se recibe un mensaje."""
        rospy.loginfo(f"Mensaje recibido: {msg.data}")
        self.beep()

    def beep(self):
        """Genera un pitido en el ordenador."""
        try:
            # Método 1: Usar `os` para un pitido del sistema
            print("\a")  # Pitido en la consola
            os.system('play -nq -t alsa synth 1 sine 440')  # Necesita `sox`
        except Exception as e:
            rospy.logwarn(f"Error al generar el pitido: {e}")

def main():
    topic_name = "/mi_topico"  # Cambia esto al tópico que desees
    BeepOnMessage(topic_name)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
