import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors
import matplotlib.cm
import matplotlib.patches
import numpy as np

if len(sys.argv) < 3: print(f"Too few arguments, use {sys.argv[0]} <input_filepath> <output_images_folder_path>"); exit(0)
if len(sys.argv) > 3: print(f"Too many arguments, use {sys.argv[0]} <input_filepath> <output_images_folder_path>"); exit(0)

#se a simulação para no meio de uma escrita a última linha fica incompleta, dropna remove essa linha (e qualquer outra
# corrompida)
data = pd.read_csv(sys.argv[1]).dropna(axis=0)

image_extension=".svg"
time = data['Tempo(s)'].array
pos_x = data['X(m)'].array
pos_y = data['Y(m)'].array
yaw = data['Yaw(rad)'].array
vel_x = data['VelX(m/s)'].array
vel_y = data['VelY(m/s)'].array
input_vel_left = data['VelActEsquerda(m/s)'].array
input_vel_right = data['VelActDireita(m/s)'].array

min_t = min(time)
max_t = max(time)

def grafico3():
    #CONFIG INICIO
    sample_period_multiplier = 10
    tick_number = 10

    orientation_arrow_dist = 0.075
    orientation_arrow_scale = 15
    orientation_arrow_alpha = 0.8
    orientation_arrow_edgesize = 0.7

    velocity_arrow_scale = 20
    velocity_arrow_alpha = 0.8
    velocity_arrow_edgesize = 0.4

    base_res = 1920
    dpi=250
    aspect_ratio = (1,1)

    titulo="Gráfico de posições"
    subtitulo=f"Incrementado com a velocidade e orientação em cada ponto\nFator de subamostragem {sample_period_multiplier}"
    #FIM CONFIG

    global time,pos_x, pos_y, yaw, vel_x, vel_y, image_extension
    time_cmap = plt.get_cmap('cividis')
    time_normalizer = matplotlib.colors.Normalize(min_t, max_t)
    colorbar = matplotlib.cm.ScalarMappable(time_normalizer, time_cmap)
    time_color = lambda time : time_cmap(time_normalizer(time))

    figsize=(base_res*aspect_ratio[0]/dpi,base_res*aspect_ratio[1]/dpi)
    fig, ax = plt.subplots(figsize=figsize, dpi=dpi)


    get_vel_arrowprops = lambda time : dict(arrowstyle='simple', linestyle='-', facecolor=time_color(time), 
        edgecolor='green', alpha=velocity_arrow_alpha, lw=velocity_arrow_edgesize, mutation_scale=velocity_arrow_scale
    )

    get_ori_arrowprops = lambda time : dict(arrowstyle='->', linestyle=':', facecolor='red', edgecolor='red',
        alpha=orientation_arrow_alpha, lw=orientation_arrow_edgesize, mutation_scale=orientation_arrow_scale
    )

    dummy_vel_handle = matplotlib.patches.FancyArrowPatch((0,1),(0,0), **get_vel_arrowprops(min_t), label="Velocidade")
    dummy_ori_handle = matplotlib.patches.FancyArrowPatch((0,1),(0,0), **get_ori_arrowprops(min_t), label="Orientação")

    for i,(t,x,y,ori,dx,dy) in enumerate(zip(time,pos_x,pos_y,yaw,vel_x, vel_y)):
        if i % sample_period_multiplier != 0: continue
        vel_arrow_coords = (x + dx, y + dy)
        ax.annotate('', vel_arrow_coords, (x,y), arrowprops=get_vel_arrowprops(t))
        orientation_arrow_cords = (x + np.cos(ori)*orientation_arrow_dist, y + np.sin(ori)*orientation_arrow_dist)
        ax.annotate('', orientation_arrow_cords, (x,y),
            arrowprops=get_ori_arrowprops(t)
        )

    fig.colorbar(colorbar, ax=ax, label="Tempo (s)")

    min_x = min(pos_x)
    max_x = max(pos_x)
    x_ampl = max_x - min_x
    min_y = min(pos_y)
    max_y = max(pos_y)
    y_ampl = max_y - min_y

    xlim = (min_x - 0.1*x_ampl, max_x + 0.1*x_ampl)
    ylim = (min_y - 0.1*y_ampl, max_y + 0.1*y_ampl)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)

    ax.set_xticks(np.linspace(xlim[0],xlim[1],tick_number))
    ax.set_yticks(np.linspace(ylim[0],ylim[1],tick_number))

    fig.suptitle(titulo)
    ax.set_title(subtitulo)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    ax.legend(handles=[dummy_vel_handle, dummy_ori_handle])
    plt.tight_layout()
    fig.savefig(sys.argv[2]+"/grafico3"+image_extension)

def grafico2():
    global time, pos_x, pos_y, yaw, image_extension
    #CONFIG INICIO
    base_res = 720
    dpi=250
    aspect_ratio = (4,3)
    markersize=0.5

    figsize=(base_res*aspect_ratio[0]/dpi,base_res*aspect_ratio[1]/dpi)
    fig, axs = plt.subplots(figsize=figsize, dpi=dpi, nrows=3,ncols=1, sharex='all')
    titulo="Configuração x Tempo"

    series_dicts = [
        dict(
            y=pos_x,
            ylabel="X (m)",
        ),
        
        dict(
            y=pos_y,
            ylabel="Y (m)",
        ),
        
        dict(
            y=yaw,
            ylabel=r'$\theta$ (rad)',
        )

    ]
    colors = ["blue", "orange", "green"]
    #FIM CONFIG

    for i, (ax, series) in enumerate(zip(axs, series_dicts)):
        ax.grid(True)
        ax.scatter(time, series["y"], s=markersize, c=colors[i])
        ax.set_ylabel(series["ylabel"])
    
    
    axs[2].set_xlabel("Time (s)")
    fig.suptitle(titulo)
    plt.tight_layout()
    
    fig.savefig(sys.argv[2]+"/grafico2"+image_extension)

def grafico1():
    global time, input_vel_left, input_vel_right, image_extension
    #CONFIG INICIO
    base_res = 720
    dpi=250
    aspect_ratio = (4,2)
    markersize=0.8

    figsize=(base_res*aspect_ratio[0]/dpi,base_res*aspect_ratio[1]/dpi)
    fig, axs = plt.subplots(figsize=figsize, dpi=dpi, nrows=2,ncols=1, sharex='all')
    titulo="Entrada x Tempo"

    series_dicts = [
        dict(
            y=input_vel_left,
            ylabel="Velocidade (m/s)",
            title="Roda esquerda"
        ),
        
        dict(
            y=input_vel_right,
            ylabel="Velocidade (m/s)",
            title="Roda direita"
        )
    ]
    colors = ["blue", "orange"]
    #FIM CONFIG

    for i, (ax, series) in enumerate(zip(axs, series_dicts)):
        ax.grid(True)
        ax.scatter(time, series["y"], s=markersize, c=colors[i])
        ax.set_ylabel(series["ylabel"])
        ax.set_title(series["title"])
    
    
    axs[-1].set_xlabel("Time (s)")
    fig.suptitle(titulo)
    plt.tight_layout()
    
    fig.savefig(sys.argv[2]+"/grafico1"+image_extension)

grafico1()
grafico2()
grafico3()