#as demais entradas do módulo são só pra suportar o RemoteAPIClient
from coppeliasim_zmqremoteapi_client import *
import math
from Controlador import ControladorPID
from Interpolador import PathGenerator

zerar = input("Criar a cena do zero? [\"N\" para cancelar]") != "N"
#zerar = False

cliente = RemoteAPIClient()
sim = cliente.require('sim')

zerar and sim.closeScene()
sim.addLog(sim.verbosity_default, "Olá mundo!")

tolerancia = 0.1
num_pontos = 10
q_inicial = [-1., 0., 0]
q_final = [1., 0., 0.]

def get_pontos_caminho(interpolador, num_pontos, altura_path=0.1):
    pontos = interpolador(num_pontos)
    poses = []
    for ponto in pontos:
        poses.extend(sim.buildPose([*ponto[:-1], altura_path], [0,0,ponto[-1]]))
    return poses

if zerar:
    #aqui se considera o path relativo à pasta do Coppelia
    robo_handle = sim.loadModel("./models/robots/mobile/pioneer p3dx.ttm")
    x_positivo = sim.generateTextShape("X eq 2", [255,255,0], 0.2)
    y_positivo = sim.generateTextShape("y eq 2", [255,255,0], 0.2)
    ponto_inicial = sim.generateTextShape("Inicio", [255,255,0], 0.2)
    ponto_final = sim.generateTextShape("Fim", [255,255,0], 0.2)
    alvo = sim.generateTextShape("alvo", [255,255,0], 0.2)
    sim.setObjectPosition(x_positivo, [2,0,0])
    sim.setObjectPosition(y_positivo, [0,2,0])
    sim.setObjectPosition(ponto_inicial, [*q_inicial,0])
    sim.setObjectPosition(ponto_final, [*q_final,0])
else:
    #aqui se considera o 'path' da cena
    robo_handle = sim.getObject("/PioneerP3DX")
    alvo = sim.getObject("/alvo")
    ponto_inicial = sim.getObject("/Inicio")
    ponto_final = sim.getObject("/Fim")
    q_inicial = [*sim.getObjectPosition(ponto_inicial)[:-1], q_inicial[-1]]
    q_final = [*sim.getObjectPosition(ponto_final)[:-1], q_final[-1]]

sim.setObjectPosition(robo_handle, [*q_inicial[:-1],0.14])
caminho = PathGenerator(q_inicial, q_final)
pontos_caminho = caminho.getGeneratedPath(num_pontos)
caminho_handle = sim.createPath(get_pontos_caminho(caminho.getGeneratedPath, num_pontos), 0b00000)

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

def calc_dist_l2(ponto1, ponto2):
    return math.sqrt((ponto2[0]-ponto1[0])**2 + (ponto2[1]-ponto1[1])**2)

def calc_pos_alvo(lista_pontos, index, pos_robo, tolerancia):
    res_index = index
    print(abs(calc_dist_l2(pos_robo, lista_pontos[index])))
    if abs(calc_dist_l2(pos_robo, lista_pontos[index])) < tolerancia:
        res_index = index+1 if index < len(lista_pontos)-1 else index
        print(res_index)
    return res_index, [*lista_pontos[res_index][:-1],0.2]


sim.setStepping(True)
sim.startSimulation()

div_freq=2
contador=0

pos_robo = sim.getObjectPosition(robo_handle)
angulo_robo = sim.getObjectOrientation(robo_handle)[2]

path_index, pos_alvo = calc_pos_alvo(pontos_caminho, 0, pos_robo, tolerancia)

alimentar_controladores(controlador_distancia, controlador_angulo, pos_alvo, pos_robo, angulo_robo)
u_v:float = controlador_distancia.calc_saida()
u_w:float = controlador_angulo.calc_saida()

while (t := sim.getSimulationTime()) < 60:
    path_index, pos_alvo = calc_pos_alvo(pontos_caminho, path_index, pos_robo, tolerancia)
    sim.setObjectPosition(alvo, pos_alvo)

    pos_robo = sim.getObjectPosition(robo_handle)
    angulo_robo = sim.getObjectOrientation(robo_handle)[2]
    alimentar_controladores(controlador_distancia, controlador_angulo, pos_alvo, pos_robo, angulo_robo)
    
    if (contador%div_freq == 0):
        u_v = controlador_distancia.calc_saida()
        u_w = controlador_angulo.calc_saida()
    else:
        u_w = controlador_angulo.calc_saida()
    if abs(calc_dist_l2(pos_robo, pos_alvo)) < tolerancia:
        u_v = u_v/10
        u_w = u_w/10
    
    velocidades = (u_v+u_w, u_v-u_w)

    setar_velocidades(*velocidades)
    sim.step()
    contador +=1

sim.stopSimulation()
sim.removeObjects(sim.getObjectsInTree(caminho_handle))