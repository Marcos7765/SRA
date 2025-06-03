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

obs_list = robm.collect_obstacles(sim, filter_obstacle_list)
rec_robo = robm.calc_robot_square_handle(sim, robo_handle)

obs_inflated = [robm.rectangle_poly_to_obst(rec_robo, rect) for rect in obs_list]
robot_abs_rect = robm.robot_rect_to_absolute(sim, robo_handle, rec_robo)

robm.plot_both_environments(sim, obs_list, obs_inflated, robot_abs_rect, robo_handle)