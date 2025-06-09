import math
from Controlador import ControladorPID

_sim = None

_robo_handle = None
_roda_esquerda = None
_roda_direita = None

_controlador_pos:ControladorPID = None
_controlador_angulo:ControladorPID = None

_tolerancia = 0.2

calc_deltas = lambda pos_robo, pos_alvo : (pos_alvo[0]-pos_robo[0], pos_alvo[1]-pos_robo[1])
calc_erro_distancia = lambda deltas : math.sqrt(deltas[0]**2 + deltas[1]**2)
calc_erro_angulo = lambda deltas : math.atan2(deltas[1],deltas[0]) #tem salto aqui entre -pi e pi

_caminho = None
_index_caminho = None

_pos_robo = None
_angulo_robo = None
_pos_alvo = None
_contadores = None

def load_sim(sim):
    global _sim
    _sim = sim

def alimentar_controladores():
    deltas = calc_deltas(_pos_robo, _pos_alvo)
    erro_distancia = calc_erro_distancia(deltas)
    erro_angulo = calc_erro_angulo(deltas)

    
    #eu nn soube resolver esse problema do atan2 e apelei pro chatgpt
    #aqui ta ajustando pro menor angulo replementar
    diff = (erro_angulo - _angulo_robo)
    diff = (diff + math.pi) % (math.tau) - math.pi
    #e aqui ta limitando o erro
    max_turn = math.pi/2
    diff = min(max_turn, diff)
    diff = max(-max_turn, diff)

    _controlador_angulo.entrada = diff
    _controlador_pos.entrada = -erro_distancia*math.cos(_controlador_angulo.entrada)

def setar_velocidades(vel_esq, vel_dir):
    _sim.setJointTargetVelocity(_roda_esquerda, vel_esq)
    _sim.setJointTargetVelocity(_roda_direita, vel_dir)

def update_robo_data():
    global _pos_robo, _angulo_robo
    _pos_robo = _sim.getObjectPosition(_robo_handle)
    _angulo_robo = _sim.getObjectOrientation(_robo_handle)[2]

def configControladores(parametros_posicao, parametros_angulo, caminho, robo_handle, rodas):
    '''
        parametros_{posicao, angulo}: dicionario com os parametros do controlador especifico, contendo dt, k_p, k_i e
            k_d.
        caminho: lista de pontos (onde cada ponto eh uma lista [x,y] no espaço de trabalho)
        robo_handle: handle do robo
        rodas: lista de handle das rodas na forma [roda_esquerda_handle, roda_direita_handle]
    '''
    global _controlador_angulo, _controlador_pos, _caminho, _index_caminho, _robo_handle, _roda_esquerda, _roda_direita
    global _pos_alvo, _contadores

    _controlador_pos = ControladorPID(0,0, **parametros_posicao)
    _controlador_angulo = ControladorPID(0,0, **parametros_angulo)
    _caminho = caminho
    _index_caminho = 0
    _pos_alvo = _caminho[_index_caminho]
    _robo_handle = robo_handle
    [_roda_esquerda, _roda_direita] = rodas
    update_robo_data()

    alimentar_controladores()
    _controlador_pos.calc_saida()
    _controlador_angulo.calc_saida()
    _contadores = [0, 0]

def set_tolerancia(tol):
    global _tolerancia
    _tolerancia = tol

def calc_dist_l2(ponto1, ponto2):
    return math.sqrt((ponto2[0]-ponto1[0])**2 + (ponto2[1]-ponto1[1])**2)

def calc_pos_alvo():
    global _index_caminho, _pos_alvo
    index = _index_caminho
    update_robo_data()
    res_index = index
    if calc_dist_l2(_pos_robo, _caminho[index]) < _tolerancia:
        res_index = index+1 if index < len(_caminho)-1 else index

    if res_index != index:
        print(f"Alvo {res_index} de {len(_caminho)-1} : {list(map(float, _caminho[res_index]))}")
        if res_index == len(_caminho)-1:
            print("Alvo final alcancado")
    _index_caminho = res_index
    _pos_alvo = [*_caminho[res_index],0.2]
    return _index_caminho, _pos_alvo

def loop_work(tempo):
    calc_pos_alvo()
    update_robo_data()
    alimentar_controladores()
    controladores = [_controlador_pos, _controlador_angulo]
    divs = [_controlador_pos.get_dt(), _controlador_angulo.get_dt()]
    
    for i, div in enumerate(divs):
        cand = tempo//div
        if cand == _contadores[i]:
            continue
        controladores[i].calc_saida()

    u_v = _controlador_pos.saidas[0]
    u_w = _controlador_angulo.saidas[0]
    if calc_dist_l2(_pos_robo, _pos_alvo) < _tolerancia:
        u_v = u_v/10
        u_w = u_w/10
    
    setar_velocidades(*(u_v+u_w, u_v-u_w))