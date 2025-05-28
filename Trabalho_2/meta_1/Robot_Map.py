import enum
import math

'''
def get_object_hit_box(self, obj_handle):
    min_x = sim.GetObjectFloatParameter( obj_handle, sim.sim_objfloatparam_modelbbox_min_x)

    max_x = sim.GetObjectFloatParameter( obj_handle, sim.sim_objfloatparam_modelbbox_max_x)

    min_y = sim.GetObjectFloatParameter( obj_handle, sim.sim_objfloatparam_modelbbox_min_y)

    max_y = sim.GetObjectFloatParameter( obj_handle, sim.sim_objfloatparam_modelbbox_max_y)


    width = max_x - min_x
    height = max_y - min_y

    return width, height


def is_within_area(self, x, y, x_center, y_center, distance):
    return abs(x - x_center) <= distance and abs(y - y_center) <= distance
'''

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

class ContactType(enum.Enum):
    A = 0
    B = 1

normalize = lambda vec : [x/math.sqrt(sum([x**2 for x in vec])) for x in vec]

def dist_vec(start, end):
    return [end[0]-start[0],end[1]-start[1]]

def dot_prod(vec_1, vec_2):
    return vec_1[0]*vec_2[0] + vec_1[1]*vec_2[1]

def external_norm(vec):
    return [-vec[1], vec[0]]

def internal_norm(vec):
    return [vec[1], -vec[0]]

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
                        #[robot_vertices[(i+1)%len(robot_vertices)][0] + rect_vertices[(j2+1)%len(rect_vertices)][0],
                            #robot_vertices[(i+1)%len(robot_vertices)][1] + rect_vertices[(j2+1)%len(rect_vertices)][1]]
                    ]
                )
                continue
            new_vertices.append(dist_vec(robot_vertices[i2], rect_vertices[j_b]))
    return new_vertices


'''
def get_map(sim, pioneer_handle, goal_handle non_obstacle_list):
    # Inicializar lista de retângulos
    rectangles = []

    # Obter a posição do Pioneer e do Goal
    pioneer_pos = sim.getObjectPosition(pioneer_handle)


    goal_pos = sim.getObjectPosition(goal_handle)

    # Obter todos os objetos na simulação
    objects = sim.GetObjects( sim.sim_handle_all)
        print('Número de objetos na simulação:', len(objects))

        for obj in objects:
            # Ignorar o robô e o destino
            if obj in [pioneer_handle, goal_handle]:
                continue

            # Obter a posição do objeto
            position = sim.GetObjectPosition( obj, -1)

                # Obter as dimensões do objeto
                width, height = self.get_object_bounding_box(obj)
                if (width is not None and width > 0.1) and (height is not None and height > 0.1):
                    # print(f'Dimensões do objeto {obj} - Largura: {width}, Altura: {height}')

                    # Filtrar objetos que podem ser quadrantes indesejados
                    if width > 2 and height > 2:  # Ajuste os critérios conforme necessário
                        print(f'Ignorando o objeto {obj} por ser um quadrante indesejado')
                        continue

                    # Verificar se o objeto está dentro da área ao redor do Pioneer e do Goal
                    if (self.is_within_area(position[0], position[1], pioneer_pos[0], pioneer_pos[1], 1) or
                            self.is_within_area(position[0], position[1], goal_pos[0], goal_pos[1], 1)):
                        print(f'Ignorando o objeto {obj} por estar dentro da área ao redor do Pioneer ou Goal')
                        continue

                    # Adicionar retângulo à lista (x, y, largura, altura)
                    rectangles.append((position[0], position[1], width, height))
                else:
                    print(f'Erro ao obter as dimensões do objeto {obj}')
            else:
                print(f'Erro ao obter a posição do objeto {obj}')
    else:
        print('Erro ao obter objetos:')

    self.map = rectangles.copy()
'''