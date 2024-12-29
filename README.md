# proyecto_rm
Proyecto de Robots Móviles 2024/25. Adriana Rodríguez. Andrés Vilches. Raquel Mollá
Este proyecto utiliza un TurtleBot para asistir en la movilidad de personas mayores en un entorno simulado. El robot emplea navegación autónoma y reconocimiento facial para guiar al usuario desde un punto inicial hasta un destino seleccionado, verificando continuamente su presencia y alertando en caso de pérdida.

Configuración Inicial: Modelo del TurtleBot. Antes de comenzar, asegúrate de configurar el modelo del robot: export TURTLEBOT_MODEL="waffle"

Archivos de Lanzamiento:
Configura el entorno inicial del robot en Gazebo:
catkin_make
source devel/setup.bash
roslaunch inicio.launch
Posicionamiento del Robot: Utiliza la flecha verde en Gazebo para posicionar el robot en el lugar inicial.
Ejecuta todas las tareas necesarias para la simulación completa: roslaunch todo.launch


Instalación y Uso de face_recognition
Para habilitar el reconocimiento facial en el proyecto, sigue los pasos a continuación:
1. Actualiza los paquetes del sistema: sudo apt update
2. Instala las dependencias necesarias:
sudo apt install build-essential cmake libboost-all-dev
sudo apt install python3-dev python3-pip
sudo apt install libopencv-dev
3. Instala las bibliotecas requeridas: pip3 install dlib
                                       pip3 install face_recognition
4. Ejecuta el script de reconocimiento facial si lo quieres para ver cómo funciona por su cuenta: python3 reconocimiento_facial.py
