import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi
from PyQt5.QtCore import QTimer
import numpy as np
import sim
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        loadUi("qtUi.ui", self)

        self.numero = 0
        self.velocity = 0
        self.centerSensor = 0
        self.leftSensor = 0
        self.rightSensor = 0
        self.clientID = 0
        self.stop = True
        self.ruedaDerecha = 0
        self.ruedaIzquierda = 0

        self.timer = QTimer()
        self.timer.timeout.connect(self.timerEvent)
        self.timer.start(100)
        self.timerGraph = QTimer()
        self.timerGraph.timeout.connect(self.actualizar_grafica)

        # Crear la figura y el eje para la gráfica
        self.graphic = False
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('Tiempo')
        self.ax.set_ylabel('Valor del sensor')
        self.ax.set_title('Datos de los sensores')

        # Configurar la precisión de los ejes
        self.ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        self.ax.yaxis.set_major_locator(MaxNLocator(integer=True))

        # Listas para almacenar los datos de los sensores
        self.tiempo = []
        self.data_sensor_center = []
        self.data_sensor_left = []
        self.data_sensor_right = []

        # Líneas de la gráfica
        self.linea_central, = self.ax.plot([], [], label='Central')
        self.linea_izquierda, = self.ax.plot([], [], label='Izquierda')
        self.linea_derecha, = self.ax.plot([], [], label='Derecha')
        self.ax.legend()

        #LOGICA DIFUSA  
        self.set_point = 0.272
        # Definir el rango de las variables de entrada y salida
        self.error_range = np.arange(-2.5, 2.5, 0.1)
        self.vel_range = np.arange(0, 1.5, 0.1)

        # Definir las variables lingüísticas para el error de posición y las velocidades de los motores
        self.error_pos = ctrl.Antecedent(self.error_range, 'error_pos')
        self.vel_motor_derecho = ctrl.Consequent(self.vel_range, 'vel_motor_derecho')
        self.vel_motor_izquierdo = ctrl.Consequent(self.vel_range, 'vel_motor_izquierdo')

        # Definir las funciones de membresía para el error de posición
        self.error_pos['negativo_grande'] = fuzz.trapmf(self.error_range, [-2.5, -2.0, -1.0, -0.5])
        self.error_pos['negativo_mediano'] = fuzz.trimf(self.error_range, [-1.0, -0.5, 0])
        self.error_pos['cero'] = fuzz.trimf(self.error_range, [-0.3, 0, 0.3])  # Se ajusta para que comience donde termina 'negativo_mediano'
        self.error_pos['positivo_mediano'] = fuzz.trimf(self.error_range, [0, 0.5, 1.0])
        self.error_pos['positivo_grande'] = fuzz.trapmf(self.error_range, [0.5, 1.0, 2.0, 2.5])

        # Definir las funciones de membresía para las velocidades de los motores derecho e izquierdo
        self.vel_motor_derecho['muy_lenta'] = fuzz.trimf(self.vel_range, [0, 0.1, 0.3])
        self.vel_motor_derecho['lenta'] = fuzz.trimf(self.vel_range, [0.3, 0.4, 0.6])
        self.vel_motor_derecho['media'] = fuzz.trimf(self.vel_range, [0.6, 0.8, 1.0])
        self.vel_motor_derecho['rapida'] = fuzz.trimf(self.vel_range, [0.8, 1.0, 1.2])
        self.vel_motor_derecho['muy_rapida'] = fuzz.trimf(self.vel_range, [1.0, 1.2, 1.5])

        self.vel_motor_izquierdo['muy_lenta'] = fuzz.trimf(self.vel_range, [0, 0.1, 0.3])
        self.vel_motor_izquierdo['lenta'] = fuzz.trimf(self.vel_range, [0.3, 0.4, 0.6])
        self.vel_motor_izquierdo['media'] = fuzz.trimf(self.vel_range, [0.6, 0.8, 1.0])
        self.vel_motor_izquierdo['rapida'] = fuzz.trimf(self.vel_range, [0.8, 1.0, 1.2])
        self.vel_motor_izquierdo['muy_rapida'] = fuzz.trimf(self.vel_range, [1.0, 1.2, 1.5])

        # Reglas difusas para el control del robot móvil
        rule1 = ctrl.Rule(self.error_pos['negativo_grande'], (self.vel_motor_derecho['muy_lenta'], self.vel_motor_izquierdo['muy_rapida']))
        rule2 = ctrl.Rule(self.error_pos['negativo_mediano'], (self.vel_motor_derecho['lenta'], self.vel_motor_izquierdo['rapida']))
        rule3 = ctrl.Rule(self.error_pos['cero'], (self.vel_motor_derecho['media'], self.vel_motor_izquierdo['media']))
        rule4 = ctrl.Rule(self.error_pos['positivo_mediano'], (self.vel_motor_derecho['rapida'], self.vel_motor_izquierdo['lenta']))
        rule5 = ctrl.Rule(self.error_pos['positivo_grande'], (self.vel_motor_derecho['muy_rapida'], self.vel_motor_izquierdo['muy_lenta']))

        # Asignar reglas al controlador
        self.controlador = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])

        self.simulacion = ctrl.ControlSystemSimulation(self.controlador)

        self.simulacion.input['error_pos'] = 0.4
        self.simulacion.compute()

        print("Vel Der: ",self.simulacion.output['vel_motor_derecho'])
        print("Vel Izq: ",self.simulacion.output['vel_motor_izquierdo'])

        self.pushButton.clicked.connect(self.start_stop)
        self.pushButton_start_coppelia.clicked.connect(self.iniciar_comunicacion_coppelia)
        self.pushButton_lecture.clicked.connect(self.iniciar_timer_grafica)


    def start_stop(self):
        if self.velocity == 0:
            self.velocity = 0.2
            sim.simxSetJointTargetVelocity(self.clientID, self.ruedaDerecha, self.velocity, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.ruedaIzquierda, self.velocity, sim.simx_opmode_oneshot)
            self.stop = False
        else:
            self.velocity = 0
            sim.simxSetJointTargetVelocity(self.clientID, self.ruedaDerecha, self.velocity, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.ruedaIzquierda, self.velocity, sim.simx_opmode_oneshot)
            self.stop = True

    def iniciar_comunicacion_coppelia(self):
        #Conectar con coppelia sim
        port = 19999 #Puerto de conexion de CoppeliaSim
        self.clientID = sim.simxStart("127.0.0.1",port,True,True,2000,5) #Insertar conector
        if self.clientID != -1:
            print("Conexion establecida con CoppeliaSim (ID de cliente: ", self.clientID,")")

            retCode, self.ruedaDerecha = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_blocking)
            retCode, self.ruedaIzquierda = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_blocking)
            
            retCode, self.centerSensor = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_CenterSensor1", sim.simx_opmode_blocking)
            retCode, self.leftSensor = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_LeftSensor2", sim.simx_opmode_blocking)
            retCode, self.rightSensor = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_RightSensor3", sim.simx_opmode_blocking)
        else:
            print("No se ha podido establecer conexion")
    
    def timerEvent(self):
        if self.stop == False:
            sensorCenter,sensorLeft,SensorRight = self.readSensor()

            if self.controlador is not None:
                error = sensorCenter - self.set_point + SensorRight - sensorLeft
                self.simulacion.input['error_pos'] = error
                self.simulacion.compute()
                vel_derecho = self.simulacion.output['vel_motor_derecho']
                vel_izquierdo = self.simulacion.output['vel_motor_izquierdo']
                # Aquí aplicas las velocidades calculadas a los motores
                sim.simxSetJointTargetVelocity(self.clientID, self.ruedaDerecha, vel_derecho, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(self.clientID, self.ruedaIzquierda, vel_izquierdo, sim.simx_opmode_oneshot)
                print("Velocidad motor derecho:", vel_derecho)
                print("Velocidad motor izquierdo:", vel_izquierdo)

    def readSensor(self):
        aux = sim.simxReadVisionSensor(self.clientID, self.centerSensor, sim.simx_opmode_streaming)
        aux1 = aux[2]
        if aux1:
            aux = np.mean(aux1)
            sensorCenter = aux
        else:
            sensorCenter = 0
        aux = sim.simxReadVisionSensor(self.clientID, self.leftSensor, sim.simx_opmode_streaming)
        aux1 = aux[2]
        if aux1:
            aux = np.mean(aux1)
            sensorLeft = aux
        else:
            sensorLeft = 0
        aux = sim.simxReadVisionSensor(self.clientID, self.rightSensor, sim.simx_opmode_streaming)
        aux1 = aux[2]
        if aux1:
            aux = np.mean(aux1)
            sensorRight = aux
        else:
            sensorRight = 0

        return sensorCenter,sensorLeft,sensorRight
    
    def iniciar_timer_grafica(self):
        if self.clientID != -1:
            if self.graphic == False:
                self.timerGraph.start(1000)  # Actualizar cada 1000 milisegundos (1 segundo)
                self.graphic = True
                plt.show()
            else:
                self.graphic = False
                self.detener_timer_grafica()
                plt.close()
        else:
            print("Primero conecte a CoppeliaSim")
            

    def detener_timer_grafica(self):
        self.timerGraph.stop()

    def actualizar_grafica(self):
        sensorCenter, sensorLeft, sensorRight = self.readSensor()

        # Agregar datos a las listas
        self.tiempo.append(self.numero)
        self.data_sensor_center.append(sensorCenter)
        self.data_sensor_left.append(sensorLeft)
        self.data_sensor_right.append(sensorRight)

        # Actualizar las líneas de la gráfica con los nuevos datos
        self.linea_central.set_data(self.tiempo, self.data_sensor_center)
        self.linea_izquierda.set_data(self.tiempo, self.data_sensor_left)
        self.linea_derecha.set_data(self.tiempo, self.data_sensor_right)

        # Redibujar la gráfica
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw_idle()

        self.numero += 1

if __name__ == "__main__" :
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())