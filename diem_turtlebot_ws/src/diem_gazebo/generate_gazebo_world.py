import cv2
import numpy as np
import os  # Per lavorare con i nomi dei file

def generate_gazebo_sdf_world(image_path, resolution, origin, output_directory, occupied_thresh=0.65, free_thresh=0.196):
    """
    Genera un file .sdf per Gazebo con muri basati su una mappa di occupazione.
    
    :param image_path: Percorso del file .pgm (mappa di occupazione).
    :param resolution: Risoluzione della mappa (metri per pixel).
    :param origin: Origine della mappa (x, y, theta) nel mondo Gazebo.
    :param output_directory: Directory dove salvare il file .sdf.
    :param occupied_thresh: Soglia per considerare una cella come occupata.
    :param free_thresh: Soglia per considerare una cella come libera.
    """
    # Estrai il nome del file senza estensione
    base_name = os.path.splitext(os.path.basename(image_path))[0]
    
    # Crea il percorso completo del file .sdf
    output_filename = os.path.join(output_directory, f"{base_name}.sdf")

    # Carica l'immagine della mappa di occupazione
    occ_grid = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if occ_grid is None:
        raise ValueError(f"Impossibile caricare la mappa {image_path}")
    
    height, width = occ_grid.shape
    
    # Crea il file .sdf per Gazebo
    with open(output_filename, 'w') as world_file:
        # Scrittura intestazione SDF
        world_file.write("""<sdf version="1.6">
<world name="default">
<include>
  <uri>model://ground_plane</uri>
</include>
<include>
  <uri>model://sun</uri>
</include>\n""")

        # Itera attraverso la mappa e crea muri per celle occupate
        for i in range(height):
            for j in range(width):
                # Converti i valori della mappa in un intervallo da 0 a 1
                cell_value = occ_grid[i, j] / 255.0

                # Verifica se la cella è occupata
                if cell_value > occupied_thresh:
                    # Le celle occupate possono essere muri
                    x = origin[0] + j * resolution
                    y = origin[1] + (height - i) * resolution

                    # Aggiungi un muro a questa posizione
                    world_file.write(f"""
  <model name="wall_{i}_{j}">
    <pose>{x} {y} 1.25 0 0 0</pose>  <!-- Posizione del muro, 1.25 è l'altezza media -->
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{resolution} {resolution} 2.5</size>  <!-- La dimensione del muro -->
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{resolution} {resolution} 2.5</size>  <!-- Visualizzazione -->
          </box>
        </geometry>
      </visual>
    </link>
  </model>\n""")

        # Chiude il file SDF
        world_file.write("</world>\n</sdf>")

    print(f"File {output_filename} generato con successo!")

# Parametri presi dal file YAML della tua mappa
image_path = "/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map/diem_map.pgm"  # Percorso del file immagine della mappa di occupazione
resolution = 0.050000  # Risoluzione in metri per pixel
origin = [-32.507755, -27.073547, 0.0]  # Origine della mappa nel mondo Gazebo
occupied_thresh = 0.65  # Soglia per considerare una cella come occupata
free_thresh = 0.196  # Soglia per considerare una cella come libera

# Directory dove salvare il file .sdf
output_directory = "/home/beniamino/turtlebot4/diem_turtlebot_ws/src/diem_gazebo/worlds"

# Genera il mondo Gazebo in formato SDF
generate_gazebo_sdf_world(image_path, resolution, origin, output_directory, occupied_thresh, free_thresh)
