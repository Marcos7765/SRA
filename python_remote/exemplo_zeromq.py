#as demais entradas do módulo são só pra suportar o RemoteAPIClient
from coppeliasim_zmqremoteapi_client import *
import math

RemoteAPIClient_parametros = dict(
    host='localhost', #IP do servidor, por padrão o da máquina local
    port=23000, #porta que o servidor está usando, por padrão 23000
    cntport=None, #não tem descrição e, no código fonte, não tem uso em momento
        #algum. Na versão assíncrona o padrão é port+1 (mas continua sem outra
        #ocorrência no código)
    verbose=None #flag de verbosidade, por padrão lê da variável de ambiente
        #VERBOSE (int(os.environ.get('VERBOSE', '0')))
)

zerar = input("Criar cena do zero? [\"N\" para cancelar]") != "N"

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

sim.startSimulation()

#o movimento do exemplo é pra parecer um 8, então catei uma amotra de
#velocidade ângular do robô no coppelia no primeiro círculo do
#setar_velocidades pra calcular o período
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