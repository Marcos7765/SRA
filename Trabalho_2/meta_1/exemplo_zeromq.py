#as demais entradas do módulo são só pra suportar o RemoteAPIClient
from coppeliasim_zmqremoteapi_client import *
import math
import Robot_Map as robm
import sys

RemoteAPIClient_parametros = dict(
    host='localhost', #IP do servidor, por padrão o da máquina local
    port=23000, #porta que o servidor está usando, por padrão 23000
    cntport=None, #não tem descrição e, no código fonte, não tem uso em momento
        #algum. Na versão assíncrona o padrão é port+1 (mas continua sem outra
        #ocorrência no código)
    verbose=None #flag de verbosidade, por padrão lê da variável de ambiente
        #VERBOSE (int(os.environ.get('VERBOSE', '0')))
)

#zerar = input("Criar cena do zero? [\"N\" para cancelar]") != "N"

zerar = len(sys.argv) > 1 and sys.argv[1] == "zerar"

cliente = RemoteAPIClient(**RemoteAPIClient_parametros)

sim = cliente.require('sim')

zerar and sim.closeScene()
sim.addLog(sim.verbosity_default, "Olá mundo!")

if zerar:
    #aqui se considera o path relativo à pasta do Coppelia
    robo_handle = sim.loadModel("./models/robots/mobile/pioneer p3dx.ttm")
else:
    #aqui se considera o 'path' da cena
    robo_handle = sim.getObject("/PioneerP3DX")

roda_esquerda = sim.getObject("/PioneerP3DX/leftMotor")
roda_direita = sim.getObject("/PioneerP3DX/rightMotor")

sim.setStepping(True)

filter_obstacle_list = [robo_handle, sim.getObject("/Floor")]

#adicionar obstáculos antes daqui, como objetos com tipo shape, alguns rotacionados no eixo Z
obs_list = robm.collect_obstacles(sim, filter_obstacle_list)
rec_robo = robm.calc_robot_square_handle(sim, robo_handle)
print(rec_robo)
rec_teste = obs_list[0]
print(obs_list[0])
print(80*"-")
print(robm.rectangle_poly_to_obst(rec_robo, rec_teste))

'''
sim.startSimulation()
periodo_aprox = (2*math.pi)/0.5889007082610693

def setar_velocidades(t):
    global periodo_aprox
    if t < periodo_aprox:
        sim.setJointTargetVelocity(roda_esquerda, 3)
        sim.setJointTargetVelocity(roda_direita, 5)
    else:
        sim.setJointTargetVelocity(roda_esquerda, 5)
        sim.setJointTargetVelocity(roda_direita, 3)

while (t := sim.getSimulationTime()) < 2*periodo_aprox:
    setar_velocidades(t)
    sim.addLog(sim.verbosity_default,
        f"Tempo: {t}\tPosicao: {sim.getObjectPosition(robo_handle)}\tVelocidades: {sim.getVelocity(robo_handle)}")
    sim.step()

sim.stopSimulation()
'''