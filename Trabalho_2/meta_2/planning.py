import numpy as np
import shapely as sp
import functools
import Robot_Map
import heapq
import math
import matplotlib.pyplot as plt


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

def obstacle_filter_generator(filter_list):
    def filter_func(obj_handle, output_list):
        if (__sim.getObjectType(obj_handle) == __sim.sceneobject_shape) and not obj_handle in filter_list:
            output_list.append(obj_handle)
            return -1
        return obj_handle
    return filter_func

def goal_filter_func(obj_handle, output_list):
    if __sim.getObjectAlias(obj_handle).startswith("alvo"):
        output_list.append(obj_handle)
        return -1
    return obj_handle

def robot_filter_func(obj_handle, output_list):
    if __sim.getObjectAlias(obj_handle).startswith("PioneerP3DX"):
        output_list.append(obj_handle)
        return -1
    return obj_handle

def collect_all(filter_list):
    return collect_objects([robot_filter_func, goal_filter_func, obstacle_filter_generator(filter_list)])


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

class GR_Manhattan:
    def __init__(self, x_lims, y_lims, x_pts, y_pts, obstacle_list, goal_real):
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
        grade_discreta = np.full(shape=(y_pts, x_pts), fill_value=-np.inf)
        
        x_temp = np.linspace(x_lims[0], x_lims[1], y_pts+1)[:-1]
        x_step = (x_temp[1]-x_temp[0])/2
        x_temp += x_step
        
        y_temp = np.linspace(y_lims[0], y_lims[1], x_pts+1)[:-1]
        y_step = (y_temp[1]-y_temp[0])/2
        y_temp += y_step

        self.x_step = x_step
        self.y_step = y_step
        self.goal_real = goal_real

        # Convertendo goal_real para índice no grid
        gx, gy = goal_real
        self.gx_idx = None
        self.gy_idx = None
        #self.gx_idx = int((gx - x_lims[0]) / ((x_lims[1] - x_lims[0]) / x_pts))
        #self.gy_idx = int((y_lims[0] - gy) / ((y_lims[0] - y_lims[1]) / y_pts))


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
            if cell_rect.contains(sp.Point(goal_real)):
                grade_discreta[i,j] = 0
                self.gx_idx = i
                self.gy_idx = j
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
        #print(f"oxe oh o caminho: {caminho}")
        has_goal = len(caminho) > 0
        while has_goal:
            level = caminho[i]
            if len(level) == 0: break
            cand = []
            for lcell in level:
                mark_cell(lcell[0], lcell[1], i+1, cand)
            caminho.append(cand)
            #print(i)
            i += 1

        self.grade_discreta = grade_discreta

        if (self.gx_idx is None or self.gy_idx is None):
            raise Exception("Erro na localizacao do alvo na grade")

    def gr_to_cartesian(self, gr_coord:list):
        '''
            gr_coord: indice na grade
        '''
        x = self.x_lims[0] + (2*gr_coord[1] + 1)*(self.x_step)
        y = self.y_lims[0] + (2*gr_coord[0] + 1)*(self.y_step)
        return [x, y]


    #   Abaixo, tem as funções para usar o gerador de caminho A*, já que temos
    def heuristica(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        D = 1
        D2 = math.sqrt(2)
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

    def is_move_diagonal_allowed(self, current, neighbor):
        i, j = current
        ni, nj = neighbor
        di = ni - i
        dj = nj - j

        if di != 0 and dj != 0:  # movimento diagonal
            # verifica os vizinhos ortogonais para evitar passar por cantos
            if self.grade_discreta[i + di, j] == np.inf or self.grade_discreta[i, j + dj] == np.inf:
                return False
        return True

    def gerar_caminho(self, start_real):
        x, y = start_real

        x_idx = int((x - self.x_lims[0]) / ((self.x_lims[1] - self.x_lims[0]) / self.x_pts))
        y_idx = int((self.y_lims[0] - y) / ((self.y_lims[0] - self.y_lims[1]) / self.y_pts))

        start = (y_idx, x_idx)
        goal = (self.gx_idx, self.gy_idx)

        if self.grade_discreta[start] == np.inf or self.grade_discreta[goal] == np.inf:
            print("Start ou goal está em obstáculo ou fora do mapa.")
            return []

        movimentos = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        open_set = []
        heapq.heappush(open_set, (0, 0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, g_current, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            i, j = current
            for di, dj in movimentos:
                ni, nj = i + di, j + dj
                if 0 <= ni < self.y_pts and 0 <= nj < self.x_pts:
                    if self.grade_discreta[ni, nj] == np.inf:
                        continue

                    neighbor = (ni, nj)
                    custo_mov = 1

                    tentative_g = g_current + custo_mov
                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + self.heuristica(neighbor,
                                                                         goal)  # heurística ainda ok pra ortogonal
                        heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        print("Caminho não encontrado.")
        return []



__exclusion_list = ["load_sim","__verify_sim_wrapper", "__clean"]

def __clean():
    for name, obj in globals().items():
        if callable(obj) and not name in __exclusion_list:
            globals()[name] = __verify_sim_wrapper(obj, name)

__clean()

def heatmap_alpha_path(GR:GR_Manhattan, caminho=None, _ax=None):
    grade = GR.grade_discreta
    heatmap = np.copy(grade)
    heatmap[np.isinf(heatmap)] = np.nan  # Obstáculos invisíveis no mapa

    cmap = plt.cm.viridis
    cmap.set_bad(color='gray')  # Obstáculos = preto

    
    if _ax is None:
        fig, ax = plt.subplots(figsize=(8, 8))
    else:
        ax = _ax
    im = ax.imshow(heatmap, cmap=cmap, origin='lower',
        extent=[GR.x_lims[0], GR.x_lims[1], GR.y_lims[0], GR.y_lims[1],])

    def y_index_to_coord(y_index):
        return GR.y_lims[0] + (2*y_index + 1)*(GR.y_step)
    def x_index_to_coord(x_index):
        return GR.x_lims[0] + (2*x_index + 1)*(GR.x_step)

    # Mostrar valores nas células com alta visibilidade
    for i in range(grade.shape[0]):
        for j in range(grade.shape[1]):
            val = grade[j, i]
            y_pos = y_index_to_coord(j)
            x_pos = x_index_to_coord(i)
            if not np.isinf(val):
                ax.text(x_pos, y_pos, str(int(val)), ha='center', va='center',
                        color='white', fontsize=12, fontweight='bold')
            else:
                ax.text(x_pos, y_pos, "X", ha='center', va='center',
                        color='white', fontsize=12, fontweight='bold')

    # Sobrepor caminho com opacidade
    if caminho:
        y_coords = [y_index_to_coord(p[0])  for p in caminho]
        x_coords = [x_index_to_coord(p[1])  for p in caminho]
        ax.plot(x_coords, y_coords, marker='o', color='red', linewidth=2, markersize=6, alpha=0.8, label='Caminho')
        ax.scatter(x_coords[0], y_coords[0], color='black', marker='*',label='Objetivo', zorder=5, s=100, alpha=0.7)
        ax.scatter(x_coords[-1], y_coords[-1], color='black', marker='x',label='Início', zorder=5, s=100, alpha=0.7)

    ax.set_title("Grade com Caminho (opacidade) e Custos")
    ax.set_xlim(tuple(GR.x_lims))
    ax.set_ylim(tuple(GR.y_lims))
    ax.set_xticks([x_index_to_coord(x)-GR.x_step for x in range(GR.x_pts+1)])
    ax.set_yticks([y_index_to_coord(y)-GR.y_step for y in range(GR.y_pts+1)])
    ax.grid(color='black', linestyle='-', linewidth=0.5)
    ax.invert_yaxis()
    ax.legend()
    plt.colorbar(im, ax=ax, label='Custo da célula')
    plt.tight_layout()

def cabou_os_nomes_eh_um_plot_de_tudo(x_lims, y_lims, x_pts, y_pts, filter_list, block=True):
    robo, objetivo, obstaculos = collect_all(filter_list)
    robo = robo[0]
    objetivo = objetivo[0]
    
    obstaculos = convert_obstacles_to_cSpace(robo, obstaculos)
    robo_pos = __sim.getObjectPosition(robo)[:2]
    objetivo_pos = __sim.getObjectPosition(objetivo)[:2]

    grade_mh = GR_Manhattan(x_lims, y_lims, x_pts, y_pts, obstaculos, objetivo_pos)
    caminho = grade_mh.gerar_caminho(robo_pos)
    fig, ax = plt.subplots(1,1, figsize=(8,8))
    heatmap_alpha_path(grade_mh, caminho, ax)
    Robot_Map.plot_environment(obstaculos, [robo_pos]*4, "Campo de Potencial Manhattan",_ax=ax)
    ax.set_xlabel("X real")
    ax.set_ylabel("Y real")
    plt.tight_layout()
    plt.show(block=block)

    return grade_mh, caminho