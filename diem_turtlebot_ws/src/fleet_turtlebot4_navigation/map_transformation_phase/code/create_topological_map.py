# create_topological_map.py

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
create_topological_map.py

Genera una mappa topologica da una mappa di occupazione (.pgm) utilizzando un file di configurazione YAML.

Uso:
    python3 create_topological_map.py /percorso/al/tuo/file/config.yaml
    Opzionalmente, puoi specificare il min_feature_size:
    --min_feature_size <valore>

    Esempio:
    python3 create_topological_map.py config.yaml --min_feature_size 0.59
"""

import os
import argparse
import logging
import shutil
import numpy as np
# Import dei moduli
from config import Config
from coordinate_transformer import CoordinateTransformer
import image_processing as img_proc
from graph_creation import create_topological_graph_using_skeleton
from visualization import save_graph_as_json,  save_topological_graph_on_original_map

def create_map_directory(map_name):
    """
    Crea una nuova cartella per la mappa, eliminando eventuali cartelle esistenti.
    
    Parametri:
        map_name (str): Nome della mappa e della cartella.
    
    Ritorna:
        str: Percorso assoluto della directory creata.
    """
    # Ottieni il percorso del livello superiore dello script
    parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    
    # Unisci la directory superiore con il nome della mappa
    map_directory = os.path.join(parent_directory, map_name)
    
    # Rimuovi la cartella esistente se presente
    if os.path.exists(map_directory):
        shutil.rmtree(map_directory)
        logging.info(f"Cartella esistente '{map_directory}' rimossa.")
    
    # Crea la nuova cartella
    os.makedirs(map_directory)
    logging.info(f"Cartella '{map_directory}' creata.")
    
    return map_directory

def save_pixel_to_map_transformations(topo_map, filename, transformer):
    """
    Salva la trasformazione tra coordinate pixel e coordinate mappa in un file di testo.
    
    Parametri:
        topo_map (networkx.Graph): Il grafo topologico con i nodi.
        filename (str): Percorso al file di testo per salvare le trasformazioni.
        transformer (CoordinateTransformer): Oggetto per la trasformazione delle coordinate.
    """
    with open(filename, 'w') as file:
        file.write("Trasformazione tra coordinate pixel e coordinate mappa:\n")
        file.write("Formato: (Pixel X, Pixel Y) -> (Map X, Map Y)\n\n")

        for node in topo_map.nodes(data=True):
            node_coord = node[0]
            node_attr = node[1]
            y_pixel, x_pixel = node_coord
            x_map, y_map = transformer.pixel_to_map(node_coord)
            label = node_attr.get("label", "")
            file.write(f"{label}: Pixel ({x_pixel}, {y_pixel}) -> Mappa ({x_map:.2f}, {y_map:.2f})\n")

    logging.info(f"Trasformazione salvata in '{filename}'")

def process_map(config):
    """
    Coordina tutti i passaggi necessari per processare la mappa e generare i file finali.
    
    Parametri:
        config (Config): Oggettzo di configurazione con i parametri della mappa.
    """
    map_name = os.path.splitext(os.path.basename(config.image_path))[0]

    # Crea una cartella per questa mappa
    map_directory = create_map_directory(map_name + "_topological")

    # Step 1: Carica la mappa originale
    original_map = img_proc.load_map(config.image_path, config.negate)
    img_proc.save_as_png(original_map, os.path.join(map_directory, f"{map_name}_original_map.png"))

    # Step 2: Pulisci la mappa e salva i passaggi intermedi
    try:
        cleaned_map = img_proc.clean_map(original_map, config, map_directory, map_name)
    except ValueError as e:
        logging.error(f"Errore durante la pulizia della mappa: {e}")
        return

    # Salva la mappa pulita
    img_proc.save_as_png(cleaned_map, os.path.join(map_directory, f"{map_name}_cleaned_map.png"))

    # Step 3: Crea la mappa binaria
    binary_map = img_proc.create_binary_map(cleaned_map)
    img_proc.save_as_png(binary_map, os.path.join(map_directory, f"{map_name}cleaned_binary_map.png"))

    # Step 4: Calcola la mappa di distanza Euclidea
    distance_map = img_proc.compute_distance_map(binary_map)
    
    # Normalizza la mappa di distanza per la visualizzazione
    max_distance = np.max(distance_map)
    if max_distance > 0:
        distance_map_normalized = (distance_map / max_distance * 255).astype(np.uint8)
    else:
        distance_map_normalized = np.zeros_like(distance_map, dtype=np.uint8)
        logging.warning("La distanza massima nella mappa di distanza è 0. Mappa normalizzata a tutti zero.")
    
    # Salva la mappa di distanza normalizzata
    img_proc.save_as_png(distance_map_normalized, os.path.join(map_directory, f"{map_name}_distance_map.png"))

    # # Step 5: Crea le linee di Voronoi
    # voronoi_map = img_proc.create_voronoi_lines(distance_map)
    # img_proc.save_as_png(voronoi_map, os.path.join(map_directory, f"{map_name}_voronoi_map.png"))

    # Step 6: Skeletonizza le linee di Voronoi
    voronoi_skeleton = img_proc.skeletonize_voronoi(distance_map_normalized)
    img_proc.save_as_png(voronoi_skeleton, os.path.join(map_directory, f"{map_name}_skeleton_voronoi.png"))

    # Inizializza CoordinateTransformer
    transformer = CoordinateTransformer(
        image_height=cleaned_map.shape[0],
        resolution=config.resolution,
        origin=config.origin
    )

    # Step 7: Crea il grafo topologico usando la skeleton
    try:
        topo_map = create_topological_graph_using_skeleton(
            voronoi_skeleton,
            config
        )
    except ValueError as e:
        logging.error(f"Errore nella creazione del grafo topologico: {e}")
        return

    # Step 8: Converti i nodi in waypoints
    waypoints = [(transformer.pixel_to_map(node_coord)[0], transformer.pixel_to_map(node_coord)[1]) for node_coord, _ in topo_map.nodes(data=True)]
    
    # Salva il grafo topologico come JSON
    save_graph_as_json(topo_map, os.path.join(map_directory, f"{map_name}_topological_graph.json"), transformer)

    # Salva la trasformazione pixel-mappa in un file di testo
    save_pixel_to_map_transformations(
        topo_map,
        os.path.join(map_directory, f"{map_name}_pixel_to_map_transformations.txt"),
        transformer
    )

    # Step 10: Sovrapponi il grafo alla mappa originale
    output_map_with_graph_png = os.path.join(map_directory, f"{map_name}_graph_on_original_map.png")
    save_topological_graph_on_original_map(
        original_map,
        topo_map,
        output_map_with_graph_png,
        transformer
    )

    logging.info("Processamento della mappa topologica completato con successo.")

if __name__ == "__main__":
    # Configura il logging
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

    # Parsing degli argomenti da riga di comando
    parser = argparse.ArgumentParser(description='Genera una mappa topologica da una mappa di occupazione.')
    parser.add_argument('config', type=str, help='Percorso al file di configurazione YAML.')
    parser.add_argument('--min_feature_size', type=float, default=0.5, help='Dimensione minima delle caratteristiche in metri per le operazioni morfologiche. Default è 0.59 metri.')
    parser.add_argument('--line_tolerance', type=float, default=0.06, help='line_tolerance.')
    args = parser.parse_args()

    # Carica la configurazione con min_feature_size
    config = Config(args.config, min_feature_size=args.min_feature_size)
    logging.info("Configurazione caricata.")

    # Processa la mappa
    process_map(config)
