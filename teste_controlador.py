#as demais entradas do módulo são só pra suportar o RemoteAPIClient
from coppeliasim_zmqremoteapi_client import *
import math
from Controlador import ControladorPID

zerar = input("Criar a cena do zero? [\"N\" para cancelar]") != "N"
#zerar = False

cliente = RemoteAPIClient()
sim = cliente.require('sim')

zerar and sim.closeScene()
sim.addLog(sim.verbosity_default, "Olá mundo!")

if zerar:
    #aqui se considera o path relativo à pasta do Coppelia
    robo_handle = sim.loadModel("./models/robots/mobile/pioneer p3dx.ttm")
    x_positivo = sim.generateTextShape("X eq 2", [255,255,0], 0.2)
    y_positivo = sim.generateTextShape("y eq 2", [255,255,0], 0.2)
    alvo = sim.generateTextShape("alvo", [255,255,0], 0.2)
    sim.setObjectPosition(x_positivo, [2,0,0])
    sim.setObjectPosition(y_positivo, [0,2,0])
else:
    #aqui se considera o 'path' da cena
    robo_handle = sim.getObject("/PioneerP3DX")
    alvo = sim.getObject("/alvo")

roda_esquerda = sim.getObject("/PioneerP3DX/leftMotor")
roda_direita = sim.getObject("/PioneerP3DX/rightMotor")


calc_deltas = lambda pos_robo, pos_alvo : (pos_alvo[0]-pos_robo[0], pos_alvo[1]-pos_robo[1])
calc_erro_distancia = lambda deltas : math.sqrt(deltas[0]**2 + deltas[1]**2)
calc_erro_angulo = lambda deltas : math.atan2(deltas[1],deltas[0]) #tem salto aqui entre -pi e pi

def alimentar_controladores(controlador_distancia:ControladorPID, controlador_angulo:ControladorPID, pos_alvo,
    pos_robo, angulo_robo):
    
    deltas = calc_deltas(pos_robo, pos_alvo)
    erro_distancia = calc_erro_distancia(deltas)
    erro_angulo = calc_erro_angulo(deltas)

    controlador_angulo.entrada = (erro_angulo - angulo_robo)
    controlador_distancia.entrada = -erro_distancia*math.cos(controlador_angulo.entrada)

def setar_velocidades(vel_esq, vel_dir):
    sim.setJointTargetVelocity(roda_esquerda, vel_esq)
    sim.setJointTargetVelocity(roda_direita, vel_dir)

parametros_distancia = dict(
    k_p=0.5,
    k_i=0.1,
    k_d=0
)

k_c_angulo = 1
parametros_angulo = dict(
    k_p=k_c_angulo,
    k_i=k_c_angulo/20,
    k_d=k_c_angulo*1
)

controlador_distancia = ControladorPID(0, 0, dt=sim.getSimulationTimeStep()*2, **parametros_distancia)
controlador_angulo = ControladorPID(0, 0, dt=sim.getSimulationTimeStep(), **parametros_angulo)


def calc_pos_alvo(t):
    if t <= 20:
        return [2, 2, 0]
    if t <= 40:
        return [-2, -2, 0]
    return [2, 0, 0]

pos_alvo = calc_pos_alvo(0)

sim.setStepping(True)
sim.startSimulation()

div_freq=10
contador=0

pos_robo = sim.getObjectPosition(robo_handle)
angulo_robo = sim.getObjectOrientation(robo_handle)[2]
alimentar_controladores(controlador_distancia, controlador_angulo, pos_alvo, pos_robo, angulo_robo)
u_v:float = controlador_distancia.calc_saida()
u_w:float = controlador_angulo.calc_saida()

while (t := sim.getSimulationTime()) < 60:
    pos_alvo = calc_pos_alvo(t)
    sim.setObjectPosition(alvo, pos_alvo)

    pos_robo = sim.getObjectPosition(robo_handle)
    angulo_robo = sim.getObjectOrientation(robo_handle)[2]
    alimentar_controladores(controlador_distancia, controlador_angulo, pos_alvo, pos_robo, angulo_robo)
    
    
    if (contador%div_freq == 0):
        u_v = controlador_distancia.calc_saida()
        u_w = controlador_angulo.calc_saida()
    else:
        u_w = controlador_angulo.calc_saida()
    
    velocidades = (u_v+u_w, u_v-u_w)

    setar_velocidades(*velocidades)
    sim.step()
    contador +=1
    if t >= 30: pos_alvo[0] = -1; pos_alvo[1] = -1

sim.stopSimulation()