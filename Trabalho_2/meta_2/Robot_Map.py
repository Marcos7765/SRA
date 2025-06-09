import math
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def rotatedVec2d(vec_in, angle):
    vec_out = [
        math.cos(angle)*vec_in[0] - math.sin(angle)*vec_in[1],
        math.cos(angle)*vec_in[1] + math.sin(angle)*vec_in[0]
    ]
    return vec_out

def get_rectangle_vertices(sim, obj_handle):
    sizes, _ = sim.getShapeBB(obj_handle) #vou assumir que o obj seja um prisma e pos esteja no seu centroide
    
    vertices_rel_dist = [[x/2, y/2] for y in [sizes[1], -sizes[1]] for x in [sizes[0], -sizes[0]]]
    #vertices_rel_dist esta segurando +lx,+ly; -lx,+ly; +lx,-ly e -lx,-ly, ent precisamos reordenar
    vertices_rel_dist[1:] = vertices_rel_dist[2:] + [vertices_rel_dist[1]] #reordenamento pra manter anti-horario
    #independentemente da rotacao em Z o acesso nessa ordem fica anti-horario
    
    pos = sim.getObjectPosition(obj_handle)
    gamma = sim.getObjectOrientation(obj_handle)[-1]
    vertices_rel_dist = [rotatedVec2d(vec, gamma) for vec in vertices_rel_dist]
    return [[vec[0]+pos[0], vec[1]+pos[1]] for vec in vertices_rel_dist]

def collect_obstacles(sim, filter_list):
    object_list = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 0b00000010)
    obstacles = [get_rectangle_vertices(sim, obj_handle) for obj_handle in object_list if not obj_handle in filter_list]
    return obstacles

def calc_robot_square_handle(sim, robot_handle):
    sizes, _ = sim.getShapeBB(robot_handle)
    side = math.sqrt(sizes[0]**2 + sizes[1]**2) # hip >= max(size_x, size_y)

    vertices_rel_dist = [[x/2, y/2] for y in [side, -side] for x in [side, -side]]
    vertices_rel_dist[1:] = vertices_rel_dist[2:] + [vertices_rel_dist[1]]

    return vertices_rel_dist

normalize = lambda vec : [x/math.sqrt(sum([x**2 for x in vec])) for x in vec]

def dist_vec(start, end):
    return [end[0]-start[0],end[1]-start[1]]

def dot_prod(vec_1, vec_2):
    return vec_1[0]*vec_2[0] + vec_1[1]*vec_2[1]

def external_norm(vec):
    return [-vec[1], vec[0]]

def internal_norm(vec):
    return [vec[1], -vec[0]]

#as duas func de contato nao precisaram ser usadas, talvez precisemos dps adaptadas pra checagem de colisao
def contact_type_a_cond(a_1, a_2, b_0, b_1, b_2):
    E_i_a_norm = normalize(dist_vec(a_1, a_2))
    V_i_a = [-E_i_a_norm[1], E_i_a_norm[0]] #rot por 90, sempre eh externo porque os pontos sao anti-horarios
    res = dot_prod(V_i_a,dist_vec(b_1, b_2)) >= 0 and dot_prod(V_i_a,dist_vec(b_1, b_0)) >= 0
    return res

def contact_type_b_cond(b_1, b_2, a_0, a_1, a_2):
    E_i_b_norm = normalize(dist_vec(b_1, b_2))
    V_i_b = [-E_i_b_norm[1], E_i_b_norm[0]] #rot por 90, sempre eh externo porque os pontos sao anti-horarios
    res = dot_prod(V_i_b,dist_vec(a_1, a_2)) >= 0 and dot_prod(V_i_b,dist_vec(a_1, a_0)) >= 0
    return res

def get_sides_vecs(polygon):
    return [dist_vec(polygon[i], polygon[(i+1)%len(polygon)]) for i in range(len(polygon))]

def rectangle_poly_to_obst(robot_vertices, rect_vertices):
    new_vertices = []

    robot_norms = [internal_norm(side) for side in get_sides_vecs(robot_vertices)]
    rec_norms = [external_norm(side) for side in get_sides_vecs(rect_vertices)]

    def adjusted_atan(vec):
        angle = math.atan2(vec[1], vec[0])
        return angle if angle >= 0 else angle+math.tau
    
    A_NORM=0
    B_NORM=1
    
    def angle_keyfunc(index, norm_type):
        source = robot_norms if norm_type==A_NORM else rec_norms
        return adjusted_atan(source[index])
    
    def find_next_of_type(S_list, index, norm_type):
        for i in range(index+1, index+len(S_list)):
            cand = S_list[i%len(S_list)]
            if cand[1] == norm_type:
                return cand
    def find_prev_of_type(S_list, index, norm_type):
        for i in range(1, len(S_list)+1):
            cand = S_list[(index-i)%len(S_list)]
            if cand[1] == norm_type:
                return cand
    
    S_list = sorted([(i, A_NORM) for i,_ in enumerate(robot_norms)] + 
        [(i, B_NORM) for i,_ in enumerate(rec_norms)], key=lambda x : angle_keyfunc(*x))
    for i, vec_tup in enumerate(S_list):
        if vec_tup[1] == A_NORM:
            i_a = vec_tup[0]
            j2, _ = find_next_of_type(S_list, i, B_NORM)
            j1, _ = find_prev_of_type(S_list, i, B_NORM)
            if robot_norms[i_a] == rec_norms[j2]: #so precisa checar o da frente porque o de traz ou ja detectou
                #ou vai detectar quando chegar na vez dele
                new_vertices.extend(
                    [dist_vec(robot_vertices[i_a],rect_vertices[j2]),
                        dist_vec(robot_vertices[(i_a+1)%len(robot_vertices)],rect_vertices[(j2+1)%len(rect_vertices)])
                        #o slide tem um erro de digitacao e nn da pra ter certeza se deveria ser b_{j+1} - a_{i+1}
                        #ou seja la o que o , significasse. se isso aqui nn funcionar tente o de baixo no lugar
                        #[robot_vertices[(i_a+1)%len(robot_vertices)][0] + rect_vertices[(j2+1)%len(rect_vertices)][0],
                            #robot_vertices[(i_a+1)%len(robot_vertices)][1] + rect_vertices[(j2+1)%len(rect_vertices)][1]]
                    ]
                )
                continue
            new_vertices.append(dist_vec(robot_vertices[i_a], rect_vertices[j2]))
        if vec_tup[1] == B_NORM:
            j_b = vec_tup[0]
            i2, _ = find_next_of_type(S_list, i, A_NORM)
            i1, _ = find_prev_of_type(S_list, i, A_NORM)
            if robot_norms[i2] == rec_norms[j_b]: 
                new_vertices.extend(
                    [dist_vec(robot_vertices[i2],rect_vertices[j_b]),
                        dist_vec(robot_vertices[(i2+1)%len(robot_vertices)],rect_vertices[(j_b+1)%len(rect_vertices)])
                    ]
                )
                continue
            new_vertices.append(dist_vec(robot_vertices[i2], rect_vertices[j_b]))
    return new_vertices

def plot_rectangle(ax, vertices, edgecolor='black', facecolor='lightgray', label=None, show_vertices=True):

    polygon = Polygon(vertices, closed=True, edgecolor=edgecolor, facecolor=facecolor, linewidth=2, label=label)
    ax.add_patch(polygon)

    if show_vertices:
        xs, ys = zip(*vertices)
        ax.plot(xs, ys, 'o', color=edgecolor, markersize=5)  # 'o' para marcar os pontos dos vértices

def plot_environment(obstacles, robot_rect, title="Ambiente no CoppeliaSim", _ax=None):
    if _ax is None:
        fig, ax = plt.subplots()
    else:
        ax = _ax
    ax.set_aspect('equal')


    for i, rect in enumerate(obstacles):
        plot_rectangle(ax, rect, edgecolor='black', facecolor='lightgray', label='Obstáculo' if i == 0 else None)


    plot_rectangle(ax, robot_rect, edgecolor='green', facecolor='lightgreen', label='Robô')


    all_points = [pt for rect in obstacles + [robot_rect] for pt in rect]

    plt.title(title)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True)
    ax.legend()
    plt.show(block=False)

def robot_rect_to_absolute(sim, robot_handle, rel_vertices):
    pos = sim.getObjectPosition(robot_handle)
    gamma = sim.getObjectOrientation(robot_handle)[-1]
    abs_vertices = [rotatedVec2d(vec, gamma) for vec in rel_vertices]
    return [[v[0]+pos[0], v[1]+pos[1]] for v in abs_vertices]

def plot_both_environments(sim, obstacles_real, obstacles_inflated, robot_rect, robot_handle):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7))
    ax1.set_aspect('equal')
    ax2.set_aspect('equal')

    # Ambiente Real
    for i, rect in enumerate(obstacles_real):
        plot_rectangle(ax1, rect, edgecolor='black', facecolor='lightgray', label='Obstáculo' if i == 0 else None)
    plot_rectangle(ax1, robot_rect, edgecolor='green', facecolor='lightgreen', label='Robô')
    all_points_1 = [pt for rect in obstacles_real + [robot_rect] for pt in rect]
    xs1, ys1 = zip(*all_points_1)
    ax1.set_xlim(min(xs1) - 0.5, max(xs1) + 0.5)
    ax1.set_ylim(min(ys1) - 0.5, max(ys1) + 0.5)
    ax1.set_title('Espaço de Trabalho')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.grid(True)
    ax1.legend()

    # Ambiente Inflado
    for i, rect in enumerate(obstacles_inflated):
        plot_rectangle(ax2, rect, edgecolor='red', facecolor='salmon', label='Obstáculo Inflado' if i == 0 else None)
    robot_pos = sim.getObjectPosition(robot_handle)
    ax2.plot(robot_pos[0], robot_pos[1], 'go', label='Robô (ponto)', markersize=8)
    all_points_2 = [pt for rect in obstacles_inflated for pt in rect]
    all_points_2.append((robot_pos[0], robot_pos[1]))
    xs2, ys2 = zip(*all_points_2)
    ax2.set_xlim(min(xs2) - 0.5, max(xs2) + 0.5)
    ax2.set_ylim(min(ys2) - 0.5, max(ys2) + 0.5)
    ax2.set_title('Espaço de Configuração')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()