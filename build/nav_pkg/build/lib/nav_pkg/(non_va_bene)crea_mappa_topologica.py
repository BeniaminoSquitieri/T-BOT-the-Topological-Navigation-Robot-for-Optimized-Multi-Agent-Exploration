import os
import cv2
import yaml
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from scipy.spatial import distance

# Percorso al file YAML fornito
yaml_path = '/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map/skeleton_map.yaml'

# --- Passo 1: Carica la mappa scheletrizzata dal file YAML ---
try:
    with open(yaml_path, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
except Exception as e:
    print(f"Errore durante il caricamento del file YAML: {e}")
    exit()

# Estrarre il percorso dell'immagine dal file YAML
if 'image' in yaml_data:
    image_path = yaml_data['image']
    # Se il percorso dell'immagine è relativo, costruisci il percorso assoluto
    map_dir = os.path.dirname(yaml_path)
    if not os.path.isabs(image_path):
        image_path = os.path.join(map_dir, image_path)
else:
    print("Errore: Il file YAML non contiene la chiave 'image'. Verifica il contenuto.")
    exit()

print(f"Percorso dell'immagine: {image_path}")

# --- Passo 2: Carica l'immagine della mappa scheletrizzata ---
skeleton = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Controlla se l'immagine è stata caricata correttamente
if skeleton is None:
    print(f"Errore: impossibile aprire il file {image_path}. Verifica il percorso.")
    exit()

# Crea una maschera binaria (0 per sfondo, 1 per lo scheletro)
_, binary_skeleton = cv2.threshold(skeleton, 127, 1, cv2.THRESH_BINARY)

# --- Passo 3: Crea il grafo topologico ---
topo_map = nx.Graph()  # Crea un grafo vuoto

rows, cols = binary_skeleton.shape

# Identificazione dei nodi (intersezioni e punti terminali)
for x in range(1, rows - 1):
    for y in range(1, cols - 1):
        if binary_skeleton[x, y] == 1:
            # Trova i vicini del pixel attuale
            neighbors = binary_skeleton[x-1:x+2, y-1:y+2].flatten()
            num_neighbors = np.sum(neighbors) - 1  # Tolgo 1 perché include se stesso

            # Identificazione di un nodo
            if num_neighbors > 2:  # Punto di intersezione
                topo_map.add_node((x, y), type='intersection')
            elif num_neighbors == 1:  # Punto finale (cul-de-sac)
                topo_map.add_node((x, y), type='endpoint')

# Aggiunta degli archi
visited = set()
nodes_list = list(topo_map.nodes)  # Crea una lista dei nodi per evitare la modifica durante l'iterazione

for (x, y) in nodes_list:
    if (x, y) not in visited:
        current = (x, y)
        path = [current]
        
        while True:
            visited.add(current)
            # Trova i vicini del nodo corrente nello scheletro
            neighbors = [(current[0] + dx, current[1] + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
            neighbors = [n for n in neighbors if binary_skeleton[n[0], n[1]] == 1 and n not in visited]
            
            if len(neighbors) == 1:
                next_node = neighbors[0]
                path.append(next_node)
                current = next_node
            elif len(neighbors) == 0:
                # Nessun vicino - terminazione del percorso
                break
            else:
                # Arrivato a un nodo di intersezione o endpoint
                break
        
        if len(path) > 1:
            # Aggiungi arco tra il nodo di partenza e quello di arrivo
            topo_map.add_edge(path[0], path[-1], weight=len(path))

# --- Passo 4: Accorpa i nodi vicini tra di loro ---
# Soglia di distanza: i nodi più vicini di questa distanza saranno accorpati
soglia_accorpamento = 15  # Puoi regolare questo valore per accorpare più o meno nodi

# Lista dei nodi da verificare per l'accorpamento
nodes_to_merge = list(topo_map.nodes())

# Utilizza un set per memorizzare i nodi che devono essere rimossi dopo l'accorpamento
nodi_da_rimuovere = set()

# Itera attraverso la lista dei nodi e accorpa quelli vicini
for i, node1 in enumerate(nodes_to_merge):
    for j in range(i + 1, len(nodes_to_merge)):
        node2 = nodes_to_merge[j]
        # Calcola la distanza euclidea tra i due nodi
        dist = distance.euclidean(node1, node2)
        if dist < soglia_accorpamento:
            # Accorpa i nodi: scegli un rappresentante (ad esempio, node1) e aggiungi gli archi di node2 a node1
            for neighbor in list(topo_map.neighbors(node2)):
                topo_map.add_edge(node1, neighbor, weight=topo_map[node2][neighbor]['weight'])
            # Contrassegna node2 per la rimozione
            nodi_da_rimuovere.add(node2)

# Rimuovi i nodi accorpati
topo_map.remove_nodes_from(nodi_da_rimuovere)

# --- Passo 5: Visualizza la mappa topologica sovrapposta alla mappa scheletrizzata ---
plt.figure(figsize=(20, 20))

# Mostra l'immagine della mappa scheletrizzata come sfondo
plt.imshow(skeleton, cmap=plt.cm.gray)

# Filtra i nodi per tipo usando .get() per evitare il KeyError
intersection_nodes = [node for node, data in topo_map.nodes(data=True) if data.get('type') == 'intersection']
endpoint_nodes = [node for node, data in topo_map.nodes(data=True) if data.get('type') == 'endpoint']

# Crea il dizionario delle posizioni dei nodi
pos = {node: (node[1], node[0]) for node in topo_map.nodes()}  # Inverti (x, y) per matplotlib (col, row)

# Disegna i nodi di intersezione e endpoint con colori diversi
nx.draw_networkx_nodes(topo_map, pos, nodelist=intersection_nodes, node_color='red', node_size=20, label='Intersezioni')
nx.draw_networkx_nodes(topo_map, pos, nodelist=endpoint_nodes, node_color='blue', node_size=10, label='Punti Finali')

# Disegna gli archi del grafo
nx.draw_networkx_edges(topo_map, pos, edge_color='green', width=1)

plt.title("Mappa Topologica Sovrapposta alla Mappa Scheletrizzata (Accorpamento dei Nodi)")
plt.axis('off')
plt.legend()
plt.show()

