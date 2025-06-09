#as demais entradas do módulo são só pra suportar o RemoteAPIClient
from coppeliasim_zmqremoteapi_client import *
import math
import sys
import planning as plan
import moving as mv

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

if zerar:
    #aqui se considera o path relativo à pasta do Coppelia
    robo_handle = sim.loadModel("./models/robots/mobile/pioneer p3dx.ttm")
    x_positivo = sim.generateTextShape("X eq 2", [255,255,0], 0.2)
    y_positivo = sim.generateTextShape("y eq 2", [255,255,0], 0.2)
    alvo = sim.generateTextShape("alvo", [255,255,0], 0.2)
    sim.setObjectPosition(x_positivo, [2,0,0])
    sim.setObjectPosition(y_positivo, [0,2,0])
    exit(0)
else:
    #aqui se considera o 'path' da cena
    robo_handle = sim.getObject("/PioneerP3DX")

roda_esquerda = sim.getObject("/PioneerP3DX/leftMotor")
roda_direita = sim.getObject("/PioneerP3DX/rightMotor")

sim.setStepping(True)

filter_list = [sim.getObject("/Floor")]

plan.load_sim(sim)
gr, caminho = plan.cabou_os_nomes_eh_um_plot_de_tudo([-2.5, 2.5], [2.5, -2.5], 10, 10, filter_list, block=False)

caminho_real = [gr.gr_to_cartesian(ponto) for ponto in caminho]


def get_pontos_caminho(lista_pontos, altura_path=0.1):
    poses = []
    ang = 0
    for i, ponto in enumerate(lista_pontos):
        if i != len(lista_pontos)-1:
            prox = lista_pontos[i+1]
            dist = (prox[0] - ponto[0], prox[1]-ponto[1])
            ang = math.atan2(dist[1], dist[0])
        poses.extend(sim.buildPose([*ponto, altura_path], [0,0,ang]))
    return poses

caminho_handle = sim.createPath(get_pontos_caminho(caminho_real), 0b00000)

input("De enter pra seguir com a simulacao")

ganho_base = 0.4

k_c_distancia = ganho_base*0.5
parametros_distancia = dict(
    dt=sim.getSimulationTimeStep(),
    k_p=k_c_distancia,
    k_i=k_c_distancia/10,
    k_d=k_c_distancia*0
)

k_c_angulo = ganho_base
parametros_angulo = dict(
    dt=sim.getSimulationTimeStep(),
    k_p=k_c_angulo,
    k_i=k_c_angulo/30,
    k_d=k_c_angulo*1
)

mv.load_sim(sim)
mv.configControladores(parametros_distancia, parametros_angulo, caminho_real, robo_handle, [roda_esquerda, roda_direita])

sim.setStepping(True)
sim.startSimulation()

while (t := sim.getSimulationTime()) < 180:
    mv.loop_work(t)
    sim.step()

sim.stopSimulation()

sim.removeObjects(sim.getObjectsInTree(caminho_handle))