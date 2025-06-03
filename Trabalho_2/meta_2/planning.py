import numpy as np
import shapely as sp
import functools
import Robot_Map

__all__ = ["load_sim", "collect_objects"] #descobri que pra isso daqui funcionar msm eu teria que criar outro modulo
__sim = None #tentativa de fazer um singleton sem classe e sem singleton so pra tirar 1 arg das chamadas

def load_sim(_sim):
    global __sim
    __sim = _sim

def __verify_sim_wrapper(orig_func, name):
    #puxei esse decorator so porque queria ver a signature com help() no repl
    @functools.wraps(orig_func)
    def wrapper(*args, **kwargs):
        if __sim is None:
            print("Por favor chame load_sim(__sim) antes de usar " +name)
            raise BaseException("sim não carregado pro planner")
        return orig_func(*args, **kwargs)
    return wrapper

#ideia: deixar a filtragem por parte de usuario, com cada filter_func sendo uma f: obj_handle, container -> exit_code
#um exit_code de -1 sinaliza para seguir imediatamente com o proximo objeto
def collect_objects(filter_funcs):
    object_list = __sim.getObjectsInTree(__sim.handle_scene, __sim.handle_all, 0b00000010)
    results = [[] for _ in filter_funcs]
    for obj_handle in object_list:
        temp = obj_handle
        for i, func in enumerate(filter_funcs):
            if func(temp, results[i]) == -1: break
    return results

def obstacle_filter_func(obj_handle, output_list):
    if __sim.getObjectType(obj_handle) == __sim.sceneobject_shape:
        output_list.append(obj_handle)
        return -1
    return obj_handle

def goal_filter_func(obj_handle, output_list):
    if __sim.getObjectAlias(obj_handle).startswith("Alvo"):
        output_list.append(obj_handle)
        return -1
    return obj_handle

def robot_filter_func(obj_handle, output_list):
    if __sim.getObjectAlias(obj_handle).startswith("PioneerP3DX"):
        output_list.append(obj_handle)
        return -1
    return obj_handle

def collect_all():
    return collect_objects([robot_filter_func, goal_filter_func, obstacle_filter_func])


def convert_obstacles_to_cSpace(robot_handle, obstacle_handle_list):
    '''
        Use da seguinte forma:
        1. obtenha o handle do robo e dos obstaculos com:
            robo, obstaculos, objetivo = collect_all()
            robo = robo[0]
        2. chame a funcao com as handles obtidas:
            obstaculos_vertices = convert_obstacles_to_cSpace(robo, obstaculos)
    '''
    obs_vertex = [Robot_Map.get_rectangle_vertices(__sim, obj_handle) for obj_handle in obstacle_handle_list]
    robot_vertex = Robot_Map.calc_robot_square_handle(__sim, robot_handle)
    return [Robot_Map.rectangle_poly_to_obst(robot_vertex, obs_rect) for obs_rect in obs_vertex]


'''
def distance_point_to_segment(point, line_start, line_end):
    line_seg = shapely.LineString([tuple(line_start), tuple(line_end)])
    return line_seg.distance(shapely.Point(tuple(point)))
'''


#def grade_retangular(x_range, y_range, x_pts, y_pts)
    


class GR_Manhattan:
    def __init__(self, x_lims, y_lims, x_pts, y_pts, obstacle_list, goal):
        '''
        goal -> coordenadas do objetivo
        obstacle_list -> lista de obstaculos, com cada obstaculo sendo uma lista de vertices. veja convert_obstacles_to_cSpace
        x_pts -> numero de colunas da grade
        y_pts -> numero de linhas da grade
        x_lims -> limites horizontais da grade na forma [inferior, superior]
        y_lims -> limites verticais da grade na forma [inferior, superior]
        '''
        self.x_lims = x_lims
        self.y_lims = y_lims
        self.x_pts = x_pts
        self.y_pts = y_pts
        grade_discreta = np.zeros(shape=(y_pts, x_pts))
        
        x_temp = np.linspace(x_lims[0], x_lims[1], y_pts+1)[:-1]
        x_step = (x_temp[1]-x_temp[0])/2
        x_temp += x_step
        
        y_temp = np.linspace(y_lims[0], y_lims[1], x_pts+1)[:-1]
        y_step = (y_temp[1]-y_temp[0])/2
        y_temp += y_step

        cell_to_rect = lambda x_cell, y_cell : sp.Polygon(
            (
                (x_cell + x_step, y_cell + y_step), (x_cell - x_step, y_cell + y_step),
                (x_cell - x_step, y_cell - y_step),(x_cell + x_step, y_cell - y_step)
            )
        )

        caminho = []

        obstacle_polygons = [sp.Polygon(obs_vertex) for obs_vertex in obstacle_list]
        for (i, j), _ in np.ndenumerate(grade_discreta):
            cell_rect = cell_to_rect(x_temp[j], y_temp[i])
            for obstacle in obstacle_polygons:
                if obstacle.intersects(cell_rect):
                    grade_discreta[i,j] = np.inf
                    break
                grade_discreta[i,j] = -np.inf
            if cell_rect.contains(sp.Point(goal)):
                grade_discreta[i,j] = 0
                caminho.append([(i,j)])

        def mark_cell(i_cell, j_cell, next_i, next_level_list):
            cfgs = []
            if i_cell < y_pts-1:
                cfgs.append((i_cell+1, j_cell))
            if i_cell > 0:
                cfgs.append((i_cell-1, j_cell))
            if j_cell < x_pts-1:
                cfgs.append((i_cell, j_cell+1))
            if j_cell > 0:
                cfgs.append((i_cell, j_cell-1))
            for i, j in cfgs:
                if grade_discreta[i, j] == -np.inf:
                    grade_discreta[i, j] = next_i
                    next_level_list.append((i,j))

        i = 0
        print(f"oxe oh o caminho: {caminho}")
        has_goal = len(caminho) > 0
        while has_goal:
            level = caminho[i]
            if len(level) == 0: break
            cand = []
            for lcell in level:
                mark_cell(lcell[0], lcell[1], i+1, cand)
            caminho.append(cand)
            print(i)
            i += 1

        self.grade_discreta = grade_discreta

__exclusion_list = ["load_sim","__verify_sim_wrapper", "__clean"]

def __clean():
    for name, obj in globals().items():
        if callable(obj) and not name in __exclusion_list:
            globals()[name] = __verify_sim_wrapper(obj, name)

__clean()

load_sim(10) #carregando um valor qualquer pra nn encher o saco


#FALTA:
# - um gerador de caminhos que comece na celula inicial e navegue para a celula de menor valor
# - testar no coppelia

#limpem isso aqui depois que se convencerem que o codigo funciona
circulo_exemplo = [(1.5,2.5), (3.5,2.5),(3.5,4.5), (1.5, 4.5)]
triangulo_exemplo = [[6.5, 3.5], [7.5, 3.5], [7, 5.5]]

teste = GR_Manhattan([0,8], [8,0], 8, 8, [circulo_exemplo, triangulo_exemplo], (1.5, 6.5))

print("exemplo da pag 28 do slide Planejamento_Guiado_por_Potencial.pdf")
print(f"teste.grade_discreta=\n{teste.grade_discreta}")