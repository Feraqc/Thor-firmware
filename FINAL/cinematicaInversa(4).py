import numpy as np
import serial, time
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

SIMULACION = True # Acciona los mensajes de Debug y desactiva la comunicacion Serial
GRAFICAR = False # Activa los graficos ve visualizacion de trayectoria previa
#interpolated_points = []
OK_POINT = 0
Q1_OUT_RANGE = "Q1 FUERA DE RANGO"
Q2_OUT_RANGE = "Q2 FUERA DE RANGO"
Q3_OUT_RANGE = "Q3 FUERA DE RANGO"
Q4_OUT_RANGE = "Q4 FUERA DE RANGO"
Q5_OUT_RANGE = "Q5 FUERA DE RANGO"
Q6_OUT_RANGE = "Q6 FUERA DE RANGO"
OUT_WORK_SPACE = "FUERA DEL ESPACIO DE TRABAJO"

if SIMULACION == True:
    print("ESTAS EN MODO SIMULACION!!!!!!")
    opcion = input("CAMBIAR A MODO RUN?????, Y / -")
    if opcion == "y" or opcion == "Y":
        SIMULACION = False
    
opcion = input("DESEA PODER GRAFICAR LAS TRAYECTORIAS???? Y / -")
if opcion == "Y" or opcion == "y":
    GRAFICAR = True

      

COM = 'COM6'
##DESCOMENTAR LA LINEA DE ABAJO PARA LA COMUNICACION SERIAL CON EL ARDUINO
if SIMULACION == False:
    arduino = serial.Serial(COM,115200)
    time.sleep(2)

l1 = 20.2 #cm
l2 = 16 #cm
l3 = 19.5 #cm
l4 = 6.715 #cm
d = np.array([0, 0, l4])

A01 = lambda q1: np.array([ [np.cos(q1),    0, np.sin(q1),     0],
                            [np.sin(q1),    0, -np.cos(q1),    0],
                            [     0,        1,       0,       l1],
                            [     0,        0,       0,        1]])
A12 = lambda q2: np.array([ [-np.sin(q2), -np.cos(q2),   0,  -l2*np.sin(q2)],
                            [np.cos(q2), -np.sin(q2),    0,  l2*np.cos(q2)],
                            [       0,                  0,              1,                      0],
                            [       0,                  0,              0,                      1]])
A23 = lambda q3: np.array([ [np.sin(q3), 0, np.cos(q3),    0],
                            [-np.cos(q3), 0,  np.sin(q3),    0],
                            [     0,            -1,       0,  0],
                            [     0,             0,       0,  1]])
A34 = lambda q4: np.array([ [np.cos(q4),    0,  np.sin(q4), 0],
                            [np.sin(q4),    0, -np.cos(q4), 0],
                            [     0,        1,       0,    l3],
                            [     0,        0,       0,     1]])
A45 = lambda q5: np.array([ [np.cos(q5),    0,  -np.sin(q5), 0],
                            [np.sin(q5),    0, np.cos(q5), 0],
                            [     0,                -1,          0,          0],
                            [     0,                0,          0,          1]])
A56 = lambda q6: np.array([ [np.cos(q6), -np.sin(q6), 0,        0],
                            [np.sin(q6),  np.cos(q6), 0,        0],
                            [     0,         0,       1,       l4],
                            [     0,         0,       0,        1]])
T = lambda phi,theta,thi: np.array([[np.cos(phi)*np.cos(theta), np.cos(phi)*np.sin(theta)*np.sin(thi)-np.sin(phi)*np.sin(thi), np.cos(phi)*np.sin(theta)*np.cos(thi)+np.sin(phi)*np.sin(thi), 0],
                                    [np.sin(phi)*np.cos(theta), np.sin(phi)*np.sin(theta)*np.sin(thi)+np.cos(phi)*np.cos(thi), np.sin(phi)*np.sin(theta)*np.cos(thi)-np.cos(phi)*np.sin(thi), 0],
                                    [       -np.sin(theta),                 np.cos(theta)*np.sin(thi),                              np.cos(theta)*np.cos(thi),                                0],
                                    [             0,                                     0,                                                      0,                                           1]])
#phi = pmz = alfa
# theta = pmy = beta
# thi = pmx = gama

##########################################################
def direct_kinematics(q1,q2,q3,q4,q5,q6):
    _0A2 = np.dot(A01(q1),A12(q2))
    _0A3 = np.dot(_0A2,A23(q3))
    _0A4 = np.dot(_0A3,A34(q4))
    _0A5 = np.dot(_0A4,A45(q5))
    _0A6 = np.dot(_0A5,A56(q6))
    return _0A6     

########################################################
def inverse_kinematics_P1(x, y,z, a, b, g):
    signoQ3=-1
    codo = 1
    
    #Calculo del vector de orientacion del eje de la herramienta con respecto al eje de la base Angulos de Euler
    alpha=a*np.pi/180.0
    beta=b*np.pi/180.0
    gamma=g*np.pi/180.0
    Transform = T(alpha, beta, gamma) #Matripmz de tranformacion directa
    
    n = Transform[0,0:3]
    o = Transform[1,0:3]
    a = Transform[2,0:3]
    
    #Calculo de punto muñeca
    pmx = x - l4*a[0]
    pmy = y - l4*a[1]
    pmz = z - l4*a[2] -l1
    print("Punto Muñeca Calculado: (Altura desde Q2)")       
    print(pmx,pmy,pmz) #debug
    
    # Halla q1
    q1 = np.arctan2(pmy,pmx)
    if q1 > np.pi or q1 < -np.pi:
        return Q1_OUT_RANGE
            
    #Hallar q3
    cosq3=(pmx**2+pmy**2+pmz**2-l2**2-l3**2)/(2*l2*l3)
    if cosq3 < -1:
        cosq3 = int(cosq3)
    if cosq3 > 1:
        return OUT_WORK_SPACE
    sinq3=(signoQ3)*np.sqrt(1-cosq3**2)
    q3=np.arctan2(sinq3,cosq3)
    if q3 > np.pi/2 or q3 < -np.pi/2:
        signoQ3*=-1
        sinq3=(signoQ3)*np.sqrt(1-cosq3**2)
        q3=np.arctan2(sinq3,cosq3)
        if q3 > np.pi/2 or q3 < -np.pi/2:
            return Q3_OUT_RANGE
    #Hallar q2
    q2=(np.arctan2(pmz,(codo*np.sqrt(pmx**2+pmy**2))) - np.arctan2((l3*sinq3),(l2+l3*cosq3)))-np.pi/2
    if q2 > np.pi/2 or q2 < -np.pi/2:
        codo*=-1
        q2=(np.arctan2(pmz,(codo*np.sqrt(pmx**2+pmy**2))) - np.arctan2((l3*sinq3),(l2+l3*cosq3)))-np.pi/2
        if q2 > np.pi/2 or q2 < -np.pi/2:
            return Q2_OUT_RANGE
        
    cosq1=np.cos(q1)
    cosq2=np.cos(q2)
    sinq1=np.sin(q1)
    sinq2=np.sin(q2) 
    sinq3=np.sin(q3) 
    
    """ A03 = np.dot(A01(q1),A12(q2))# los angulos aca deben estar en radianes
    A03 = np.dot(A03,A23(q3))
    #print(A03)#debug
    R03_inv = A03[0:3,0:3].T
    #print(R03_inv)#debug
    R36 = np.dot(R03_inv,Transform[0:3,0:3]) 
    
    q5 = -np.arccos(R36[2,2]) 
    q4 = np.arccos(R36[1,2]/np.sin(q5))
    q6 = -np.arcsin(R36[2,1]/np.sin(q5))
    q4 = np.degrees(q4)
    q5 = np.degrees(q5)
    q6 = np.degrees(q6) """
    
    #q5=np.arccos(-a[0]*cosq1*(cosq2*sinq3+cosq3*sinq2)-a[1]*sinq1*(cosq2*sinq3+cosq3*sinq2)+a[2]*(cosq2*cosq3-sinq2*sinq3))
    q5=np.arccos(-a[0]*cosq1*(cosq2*sinq3+cosq3*sinq2)-a[1]*sinq1*(cosq2*sinq3+cosq3*sinq2)+a[2]*(cosq2*cosq3-sinq2*sinq3))
    if q5 > np.pi or q5 < 0:
        print("q5:",q5)
        return Q5_OUT_RANGE
        #pass
    sinq5=np.sin(q5)

    q4=np.arctan2((cosq1*a[1]-sinq1*a[0]),abs(a[2]*(cosq2*sinq3+cosq3*sinq2)+(cosq2*cosq3-sinq2*sinq3)*(a[1]*sinq1+a[0]*cosq1)))

    sinq4=np.sin(q4)
    cosq4=np.cos(q4)

    q6=np.arcsin(n[0]*(-2*cosq4*sinq1+cosq1*sinq4*(-cosq2*cosq3+sinq2*sinq3))+n[1]*(cosq1*cosq4+sinq1*sinq4*(sinq2*sinq3-cosq2*cosq3))-n[2]*sinq4*(cosq2*sinq3+cosq3*sinq2))
    
    # Convertir a grados
    q1 = np.degrees(q1).round(decimals=3)
    q2 = np.degrees(q2).round(decimals=3)
    q3 = np.degrees(q3).round(decimals=3)
    q4 = np.degrees(q4).round(decimals=3)
    q5 = np.degrees(q5).round(decimals=3)
    q6 = np.degrees(q6).round(decimals=3)
    #print(str(q1) + str(q2) + str(q3) + str(q4) + str(q5) + str(q6))
   
    return q1, q2, q3, q4, q5, q6

##############################################################
def inverse_kinematics_G1(x, y, z, corregir = False):
    signoQ3=-1
    codo = 1
    
    #print("Corregir", corregir)

    #Calculo de punto muñeca
    pmx = x
    pmy = y
    pmz = z - l1
    
    if SIMULACION == True:    
        print("--------------------------------")
        print("Punto Muñeca Calculado: ")       
        print(pmx,pmy,pmz) #debug
        print("--------------------------------")
    
    # Halla q1
    q1 = np.arctan2(pmy,pmx)
    if q1 < 0:
        q1 = np.pi*2 + q1
    """ if q1 > np.pi or q1 < -np.pi:
        if corregir == False:
            return Q1_OUT_RANGE
        else:
            if q1 < 0:
                q1 = np.pi """
    
    #q3
    cosq3=(pmx**2+pmy**2+pmz**2-l2**2-l3**2)/(2*l2*l3)
    if cosq3 < -1:
        cosq3 = int(cosq3)
    if cosq3 > 1 or cosq3 < 0.0871:
        if corregir == False:
            return OUT_WORK_SPACE
        else:
            cosq3 = 1
    sinq3=(signoQ3)*np.sqrt(1-cosq3**2)
    q3=np.arctan2(sinq3,cosq3)
    if q3 > np.pi/2.1 or q3 < -np.pi/2.1:
        signoQ3*=-1
        sinq3=(signoQ3)*np.sqrt(1-cosq3**2)
        q3=np.arctan2(sinq3,cosq3)
        if q3 > np.pi/2 or q3 < -np.pi/2:
            print("q3:",q3,cosq3)
            return Q3_OUT_RANGE
    
    #q2
    q2=(np.arctan2(pmz,(codo*np.sqrt(pmx**2+pmy**2))) - np.arctan2((l3*sinq3),(l2+l3*cosq3)))-np.pi/2
    if q2 > np.pi/2.1 or q2 < -np.pi/2.1:
        codo*=-1
        q2=(np.arctan2(pmz,(codo*np.sqrt(pmx**2+pmy**2))) - np.arctan2((l3*sinq3),(l2+l3*cosq3)))-np.pi/2
        if q2 > np.pi/2.1 or q2 < -np.pi/2.1:
            if corregir == False:
                return Q2_OUT_RANGE
            else:
                if q2 < 0:
                    q2 = -np.pi/2.1
                else:
                    q2 = np.pi/2.1
    #print(q2)
    
    q1 = np.degrees(q1).round(decimals=3)
    q2 = np.degrees(q2).round(decimals=3)
    q3 = np.degrees(q3).round(decimals=3)
    print("q1: ", str(q1))
    print("q2: ", str(q2))
    print("q3: ", str(q3))

    return q1, q2, q3

############################################################
def cartesian2polar(x,y,z):
    r = np.sqrt(x**2 + y**2 + z**2)
    Q = np.degrees(np.arctan2(y,x)).round(decimals=3) 
    phi = np.degrees(np.arccos(z/r)).round(decimals=3) 
    return r,Q,phi

##########################################################
def polar2cartesian(r,q,phi):
    q = q*np.pi/180
    phi = phi*np.pi/180
    x = r*np.sin(phi)*np.cos(q)
    y = r*np.sin(phi)*np.sin(q)
    z = r*np.cos(phi)
    return x, y, z

#####################INTERPOLACION EN EL ESPACIO DE TRABAJO#################################
def Interpolacion_workSpace(points):
    #print(points)
    # Extraer los puntos individuales
    x_points = points[:, 0]
    y_points = points[:, 1]
    z_points = points[:, 2]

    # Crear una secuencia de valores t para los puntos originales
    t_points = np.linspace(0, 1, len(points))

    # Crear una secuencia de valores t para la interpolación (30 puntos)
    t_interpolated = np.linspace(0, 1, 30)

    # Realizar la interpolación cúbica
    cs_x = CubicSpline(t_points, x_points,bc_type = 'clamped')
    cs_y = CubicSpline(t_points, y_points,bc_type = 'clamped')
    cs_z = CubicSpline(t_points, z_points,bc_type = 'clamped')

    # Generar los puntos interpolados
    x_interpolated = cs_x(t_interpolated)
    y_interpolated = cs_y(t_interpolated)
    z_interpolated = cs_z(t_interpolated)

    # Guardar los puntos interpolados en una matriz
    interpolated_points = np.vstack((x_interpolated, y_interpolated, z_interpolated)).T

    x_corregido = []
    y_corregido = []
    z_corregido = []
    for k in interpolated_points:
        k[2] = k[2] - 20.2
        r,q,phi = cartesian2polar(k[0], k[1], k[2]) #Max = 40.415 Min = 25.224 
        #print(r,q,phi)
        if phi < -49:
            phi = -48
        if r < 26.2:
            r = 26.5
            x, y, z = polar2cartesian(r,q,phi)
            x_corregido.append(x)
            y_corregido.append(y)
            z_corregido.append(z + 20.2)
        elif r > 35.4:
            r = 35
            x, y, z = polar2cartesian(r,q,phi)
            x_corregido.append(x)
            y_corregido.append(y)
            z_corregido.append(z + 20.2)
        else:
            x, y, z = polar2cartesian(r,q,phi)
            x_corregido.append(x)
            y_corregido.append(y)
            z_corregido.append(z + 20.2)
        
    interpolated_points = np.vstack((x_corregido, y_corregido, z_corregido)).T
    if SIMULACION == True:
        pass
        print("Puntos de la interpolacion:")
        print(interpolated_points)
        
    # Visualizar la trayectoria original e interpolada
    if GRAFICAR == True:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x_points, y_points, z_points, '*', label='Puntos originales')
        ax.plot(x_interpolated, y_interpolated, z_interpolated, 'o', label='Puntos Interpolados')
        ax.plot(x_interpolated, y_interpolated, z_interpolated, '-', label='Trayectoria interpolada')
        ax.plot(x_corregido, y_corregido, z_corregido, '-', label='Trayectoria corregida')

        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_title('Trayectoria Interpolada del Brazo Robótico Thor')
        ax.legend()
        plt.show()
        plt.show()
    return interpolated_points

#####################INTERPOLACION EN EL ESPACIO ARTICULAR####################
def interpolacion_QsSpace(points):
    q1_points_corregidos = []
    q3_points_corregidos = []
    q2_points_corregidos = []

    try:
        for k in points:
            q1, q2, q3 = inverse_kinematics_G1(k[0],k[1],k[2],corregir=True)
            # Extraer los puntos individuales
            q1_points_corregidos.append(int(q1))
            q2_points_corregidos.append(int(q2))
            q3_points_corregidos.append(int(q3))
    except:
        ERROR = inverse_kinematics_G1(k[0],k[1],k[2],corregir=True)
        print(ERROR)
    q1_points_corregidos = np.array(q1_points_corregidos)
    q2_points_corregidos = np.array(q2_points_corregidos)
    q3_points_corregidos = np.array(q3_points_corregidos)

    # Crear una secuencia de valores t para los puntos originales
    t_points = np.linspace(0, 1, len(points))

    # Crear una secuencia de valores t para la interpolación (30 puntos)
    t_interpolated = np.linspace(0, 1, 30)

    # Realizar la interpolación cúbica
    cs_x = CubicSpline(t_points, q1_points_corregidos,bc_type = 'clamped')
    cs_y = CubicSpline(t_points, q2_points_corregidos,bc_type = 'clamped')
    cs_z = CubicSpline(t_points, q3_points_corregidos,bc_type = 'clamped')

    # Generar los puntos interpolados
    q1_interpolated = cs_x(t_interpolated)
    q2_interpolated = cs_y(t_interpolated)
    q3_interpolated = cs_z(t_interpolated)

    # Guardar los puntos interpolados en una matriz
    interpolated_points = np.vstack((q1_interpolated, q2_interpolated, q3_interpolated)).T
    if SIMULACION == True:
        print("Puntos de la interpolacion:")
        print(interpolated_points)
        pass
    
    q1_corregido = []
    q2_corregido = []
    q3_corregido = []
    for k in interpolated_points: 
        q1 = k[0]
        q2 = k[1]
        q3 = k[2]
        if q2 > 85:
            q2 = 85
        if q3 > 85:
            q3 = 85
        q1_corregido.append(q1)
        q2_corregido.append(q2)
        q3_corregido.append(q3)
    #interpolated_points = np.vstack((q1_corregido, q2_corregido, q3_corregido)).T
    
    arrayQs = []
    for k in range(0,len(q1_corregido)):
        arrayQs.append([int(q1_corregido[k]),int(q2_corregido[k]),int(q3_corregido[k])])

    # Visualizar la trayectoria original e interpolada
    if GRAFICAR == True:
        fig = plt.figure()
        ax = fig.add_subplot(311)
        ax.plot( t_interpolated,q1_interpolated, '-', label='Puntos Interpolados y corregidos')
        ax.plot( t_points,q1_points_corregidos, '-', label='Puntos Corregidos')
        ax = fig.add_subplot(312)
        ax.plot( t_interpolated,q2_interpolated, '-', label='Puntos Interpolados y corregidos')
        ax.plot( t_points,q2_points_corregidos, '-', label='Puntos Corregidos')
        ax = fig.add_subplot(313)
        ax.plot( t_interpolated,q3_interpolated, '-', label='Puntos Interpolados y corregidos')
        ax.plot( t_points,q3_points_corregidos, '-', label='Puntos Corregidos')
        ax.legend()
        plt.show()
        
    return arrayQs
#######################################################
def Envio_mensaje(msg):
    #Arriba hay que descomentar la definicion del objeto arduino
    #Arduino en tiempo de debugger es NONE
    if SIMULACION == False:
        arduino.write(msg.encode())
        response = None
        while response == None:
            response = arduino.readline().decode().strip()
            print(response)
        response = None

#######################################################################
def Envio_Interpolacion(array):
    cont = 0
    #avisoInicio = arduino.readline().decode().strip()
    #print(avisoInicio)
    msg = "STR"
    respondio = False
    for k in array:
        q1 = k[0]
        q2 = k[1]
        q3 = k[2]
        cont += 1
        if np.isnan(q2):
            q2 = 0
        if np.isnan(q3):
            q3 = 0
        msg = msg + " " + str(q1) + " " + str(q2) + " " +str(q3)
    Envio_mensaje(msg)
    print("[Mensaje enviado] => " + msg)
    if SIMULACION == False:
        while respondio == False:
            response = "ESPERANDO..."
            response = arduino.readline().decode().strip()
            print(response)
            if response == "Listo":
                #print(response)
                respondio = True
                print("BUFFER CARGADO CORRECTAMENTE")
                print("CONTROLADOR LISTO PARA RECIBIR SIGUIENTE COMANDO")
                
##################### Graficador en tiempo real ################
def create_3d_animation(num_points=30, interval=50):
    # Create figure and 3D axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set the viewing angle
    ax.view_init(30, 30)

    # Create data for a 3D scatter plot
    #t = np.linspace(0, 4 * np.pi, num_points)
    x = []
    y = []
    z = []
    for k in interpolated_points:
        x.append(k[0])
        y.append(k[1])
        z.append(k[2])
    
    scatter = ax.scatter([], [], [], c='r', marker='o')
    line, = ax.plot([], [], [], 'b-', lw=2)
    
    # Set axis limits
    """ ax.set_xlim((-100, 100))
    ax.set_ylim((-100, 100))
    ax.set_zlim((0, 70)) """

    # State variable
    data = {'puntoRealizado': True, 'index': 0}

    # Initialization function
    def init():
        scatter._offsets3d = ([], [], [])
        ax.plot(x,y,z)
        return scatter, line

    # Animation function
    def animate(i):
        ##LEER SERIAL AQUI
        #response = arduino.read()
        respondio = False
        while respondio == False:
            response = arduino.readline().decode().strip()
            if response == "1":
                respondio = True          
        data['puntoRealizado'] = True
        respondio = False
            
       
        scatter._offsets3d = (x[:data['index']], y[:data['index']], z[:data['index']])
        data['index'] += 1
        if data['puntoRealizado']:
            pass
            #data['num'] = False
        else:
            pass
            #scatter._offsets3d = (x[:data['index']], y[:data['index']], z[:data['index']])
        return scatter
            #data['num'] = True
 
    # Create the animation
    anim = FuncAnimation(
        fig,               # The figure object
        animate,           # The function to update the plot
        init_func=init,    # The initialization function
        frames=num_points, # Number of frames
        interval=interval, # Delay between frames in milliseconds
        blit=False         # Do not use blitting
    )

    # Display the animation
    plt.show()

#create_3d_animation(num_points=30, interval=10)

        
##################### PROGRAMA PRINCIPAL #####################
while(True):
    #STR XA YA ZA XB YB ZB XC YC ZC XD YD ZD XE YE ZE XF YF ZF
    msg = input("[Escriba el comando] => ")
    msg = msg.strip()
    msg = msg.split()
    try:
        for k in range(1,len(msg)):
            msg[k] = float(msg[k])
        funcion = msg[0]
    except:
        print("MENSAJE NO VALIDO")
        continue
    
    type = 1 # Valor por defecto
    
    if funcion == "P1":
        if len(msg) < 8 or len(msg) > 8:
            print("NO SE INCLUYERON TODOS LOS PARAMETROS: [G# x y x a b g type]")
        else:
            if msg[-1] == 2 or msg[-1] == 1:
                type = msg[-1]
            pr = msg[1:4]
            orient = msg[4:7]
            try:
                q1, q2, q3, q4, q5, q6 = inverse_kinematics_P1(pr[0], pr[1], pr[2], orient[0], orient[1], orient[2])  
                msg =  funcion +" "+ str(q1) + " " + str(q2) + " " + str(q3) + " " + str(q4) + " " + str(q5) + " " + str(q6) + " " + str(type)         
                print("[Mensaje enviado] => " + msg)
                Envio_mensaje(msg)
            except:
                ERROR = inverse_kinematics_P1(pr[0], pr[1], pr[2], orient[0], orient[1], orient[2])
                print(ERROR)
        continue
    
    if funcion == "S00":
        if len(msg) < 1 or len(msg) > 1:
            print("NO SE INCLUYERON TODOS LOS PARAMETROS")
        else:
            msg = funcion
            print("[Mensaje enviado] => " + msg)
            Envio_mensaje(msg)
        continue
    
    if funcion == "G00":
        if len(msg) < 1 or len(msg) > 1:
            print("NO SE INCLUYERON TODOS LOS PARAMETROS")
        else:
            msg = funcion
            print("[Mensaje enviado] => " + msg)
            Envio_mensaje(msg)
        continue
    if funcion == "G1":
        if len(msg) < 5 or len(msg) > 5:
            print("NO SE INCLUYERON TODOS LOS PARAMETROS: [G# x y z type]")
        else:
            if msg[-1] == 2 or msg[-1] == 1:
                type = msg[-1]
            pm = msg[1:4]
            print(pm)
            try: 
                q1, q2, q3 = inverse_kinematics_G1(pm[0], pm[1], pm[2])
                msg =  funcion +" "+ str(q1) + " " + str(q2) + " " + str(q3) + " " + str(type)
                #msg = str(q1) + "," + str(q2) + "," + str(q3) + "," + str(type)         
                print("[Mensaje enviado] => " + msg)
                Envio_mensaje(msg)
            except:
                ERROR = inverse_kinematics_G1(pm[0], pm[1], pm[2])
                print(ERROR)
        continue
    if funcion == "G2":
        if len(msg) < 4 or len(msg) > 4:
            print("NO SE INCLUYERON TODOS LOS PARAMETROS: [G# # a vel]")
        else:
            q = msg[1]
            alfa = msg[2]
            velocidad = msg[-1]
            if velocidad <= 0 or velocidad > 20:
                print("VALORES DE VELOCIDAD FUERA DE RANGO!, VALORES ACEOTADOS: [ 0 < vel <= 20 ]")
            else:
                if len(msg) < 4 or len(msg) > 4:
                    print("CANTIDAD DE PARAMETROS INCORRECTOS: [G# # a vel]")
                else:
                    if  q != 1 and q != 4:
                        if abs(alfa) > 90:
                            print("EL ANGULO ESTA FUERA DE RANGO")
                            continue 
                    msg =  funcion +" "+ str(q) + " " + str(alfa) + " " + str(velocidad)
                    #msg =  str(q) + "," + str(alfa) + "," + str(velocidad)         
                    print("[Mensaje enviado] => " + msg)
                    Envio_mensaje(msg)
            continue
        continue
    if funcion == "G13":
        if len(msg) < 6 or len(msg) > 6:
            print("NO SE INCLUYERON TODOS LOS PARAMETROS: [G# # a vel b1 b2]")
        else:
            q = msg[1]
            alfa = msg[2]
            velocidad = msg[3]
            beta1 = msg[4]
            beta2 = msg[5]
            if velocidad <= 0 or velocidad > 20:
                print("VALORES DE VELOCIDAD FUERA DE RANGO, VALORES ACEPTADOS: [ 0 < vel <= 20 ]")
            if  q != 1:
                if abs(alfa) > 90 or abs(beta1) > 90 or abs(beta2) > 90:
                    print("UNO DE LOS ANGULOS ESTA FUERA DE RANGO")
                    continue
            msg = funcion + " " + str(q) + " " + str(alfa) + " " + str(velocidad) + " " + str(beta1) + " " + str(beta2)
            #msg = str(q) + "," + str(alfa) + "," + str(velocidad) + "," + str(beta1) + "," + str(beta2)
            print("[Mensaje enviado] => " + msg)
            Envio_mensaje(msg)
            continue
        continue
    if funcion == "STR":
        """ points = np.array([ [0, 0, 0],        # Punto 1
                            [0, 0, 0],  # Punto 2
                            [0, 0, 0],  # Punto 3
                            [0, 0, 0],  # Punto 4
                            [0, 0, 0],
                            [0, 0, 0]])
        points[0,0:3] = msg[1:4]
        points[1,0:3] = msg[4:7]
        points[2,0:3] = msg[7:10]
        points[3,0:3] = msg[10:13]
        points[4,0:3] = msg[13:16]
        points[5,0:3] = msg[16:19] """
        
        points = []
        for k in range(1,len(msg)-2,3):
            points.append(msg[k:k+3])
        points = np.array(points)
        if SIMULACION == True:
            print(points)

        try:
            for k in points:
                q1, q2, q3 = inverse_kinematics_G1(k[0],k[1],k[2],corregir= True)
        except:
            print("UNO DE LOS PUNTOS ESTA FUERA DEL ESPACIO DE TRABAJO")
            opcion = input("CONTINUAR Y CORREGIR DE FORMA AUTOMATICA? Y / -")
            if opcion == "y" or opcion == "Y":
                pass
            else:
                continue
        
        #Verificar si los puntos estan en el espacio de la tarea
        arrayQs = []
        interpolated_points = Interpolacion_workSpace(points)
        try: 
            for k in interpolated_points:
                if SIMULACION == True:
                    print(k)
                q1, q2, q3 = inverse_kinematics_G1(k[0],k[1],k[2],corregir = True)
                arrayQs.append([int(q1),int(q2),int(q3)])
            Envio_Interpolacion(arrayQs)
        except:
            ERROR = inverse_kinematics_G1(k[0],k[1],k[2])
            print("UNO DE LOS PUNTOS INTERPOLADOS ESTA FUERA DEL ESPACIO DE TRABAJO")
            print(ERROR)
        continue
    if funcion == "GTR":
        if len(msg) > 1 or len(msg) < 1:
            print("CANTIDAD DE PARAMETROS INCORRECTO")
        else:
            msg = funcion
            print("[Mensaje enviado] => " + msg)
            Envio_mensaje(msg)
        #create_3d_animation(num_points=30, interval=500)
        respondio = False
        while respondio == False:
            response = arduino.readline().decode().strip()
            print(response)
            if response == "WAITING COMMAND":
                print(response)
                respondio = True
        continue
    if funcion == "STRQ":
        points = []
        for k in range(1,len(msg)-2,3):
            points.append(msg[k:k+3])
        points = np.array(points)
        
        try:
            for k in points:
                q1, q2, q3 = inverse_kinematics_G1(k[0],k[1],k[2],corregir= True)
        except:
            print("UNO DE LOS PUNTOS ESTA FUERA DEL ESPACIO DE TRABAJO")
            opcion = input("CONTINUAR Y CORREGIR DE FORMA AUTOMATICA? Y / -")
            if opcion == "y" or opcion == "Y":
                pass
            else:
                continue
        
        if SIMULACION == True:
            print(points)    
            
        interpolated_points = interpolacion_QsSpace(points)
        Envio_Interpolacion(interpolated_points)
        continue
    if funcion == "F00":
        msg = funcion
        Envio_mensaje(msg)
        continue
    print("NO CORRESPONDE A NINGUN COMANDO")
    
#Funciones
# STR 15 15 35 21.7 13.2 37.7 23.9 12.9 43.5 10.2 14.5 44.2 -8 16.5 45.9 -12.4 15.7 45.7  Movimiento en S en el espacio
# STR -20 20 20.2 -20 20 30 -20 20 40.2 -20 10 40.2 -10 20 40.2 -10 20 30.2 cuadrilatero
# STR 15 25 28.2 0 25 43.2 -15 25 28.2 0 25 13.2 15 25 28.2 0 25 43.2 Movimiendo de circulo en el plano Y
# STR 21 21.3 28.2 30 0 28.2 0 30 28.2 -30 0 28.2 0 -30 28.2 30 0 28.2  Movimiendo de circulo en el plano X
# STR 16 0 44.415 0 16 44.415 -16 0 44.415 0 -16 44.415 16 0 44.415 0 16 44.415 Movimiendo de circulo en el plano X
# STRQ 15 15 35 21.7 13.2 37.7 23.9 12.9 43.5 10.2 14.5 44.2
############################
# STR 19.5 0 36.2 0 19.2 36.2 -19.5 0 36.2 0 -19.5 36.2 19.5 0 36.2
# STR 15 25 28.2 0 25 43.2 -15 25 28.2 0 25 13.2 15 25 28.2 0 25 43.2
#
# G1 23.09 19.38 36.56 1
#G1 12 15 45.2
#G1 13 -15 404
#G1 -15 12 45 1
#G1 13 16 46
#G2 1 
# G13 1 360 20 90 270
# P1 15 15 40 0 -90 0 1
# P1 -26.215 0 36.2 0 90 0 1