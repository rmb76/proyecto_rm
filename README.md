# proyecto_rm
Proyecto de Robots Móviles 2024/25. Adriana Rodríguez. Andrés Vilches. Raquel Mollá
Este proyecto utiliza un TurtleBot para asistir en la movilidad de personas mayores en un entorno simulado. El robot emplea navegación autónoma y reconocimiento facial para guiar al usuario desde un punto inicial hasta un destino seleccionado, verificando continuamente su presencia y alertando en caso de pérdida.

Configuración Inicial: Modelo del TurtleBot. Antes de comenzar, asegúrate de configurar el modelo del robot: export TURTLEBOT_MODEL="waffle"

Archivos de Lanzamiento:
Configura el entorno inicial del robot en Gazebo:
catkin_make | 
source devel/setup.bash | 
roslaunch inicio.launch | 
Posicionamiento del Robot: Utiliza la flecha verde en Gazebo para posicionar el robot en el lugar inicial.
Ejecuta todas las tareas necesarias para la simulación completa: roslaunch todo.launch


Instalación y Uso de face_recognition
PARA UTILIZAR FACE_RECOGNITION, instalar lo siguiente:

1. Actualizar el sistema:
----------------------------------------------------------------------
sudo apt update

2. Instalar las dependencias esenciales:
----------------------------------------------------------------------
sudo apt install build-essential cmake libboost-all-dev

3. Instalar las herramientas para Python:
----------------------------------------------------------------------
sudo apt install python3-dev python3-pip

4. Instalar las bibliotecas de OpenCV:
----------------------------------------------------------------------
sudo apt install libopencv-dev

5. Instalar las bibliotecas de Python necesarias:
----------------------------------------------------------------------
pip3 install dlib
pip3 install face_recognition

6. Preparar el entorno ROS (opcional si usas un nodo ROS):
----------------------------------------------------------------------
- Verifica que el archivo sea ejecutable:
chmod +x reconocimiento_facial.py

- Coloca el archivo en la carpeta `scripts` de tu paquete ROS si estás trabajando en un proyecto ROS.

7. Ejecutar el archivo de reconocimiento facial:
----------------------------------------------------------------------
python3 reconocimiento_facial.py


MÁS FÁCIL: Para instalar todas las bibliotecas de Python que se necesitan en un solo comando: 
----------------------------------------------------------------------
pip3 install face_recognition opencv-python numpy scipy
