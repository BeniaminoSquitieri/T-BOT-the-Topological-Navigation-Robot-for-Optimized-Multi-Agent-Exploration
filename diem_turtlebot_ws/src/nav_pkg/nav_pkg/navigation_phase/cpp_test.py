import json
import networkx as nx
import argparse
import math

def main():
    # Parsing degli argomenti da linea di comando
    parser = argparse.ArgumentParser(description='Risoluzione del Problema del Postino Cinese su un grafo orientato.')
    parser.add_argument('graph_file', type=str, help='Percorso al file JSON contenente il grafo.')
    args = parser.parse_args()

    # Lettura del grafo dal file JSON
    with open(args.graph_file, 'r') as f:
        data = json.load(f)

    # Creazione del grafo orientato
    G = nx.DiGraph()

    # Aggiunta dei nodi con attributi
    for node in data['nodes']:
        G.add_node(node['label'], x=node['x'], y=node['y'])

    # Aggiunta degli archi con pesi (ad esempio, distanza euclidea)
    for edge in data['edges']:
        u = edge['from']
        v = edge['to']
        x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
        x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
        weight = math.hypot(x1 - x2, y1 - y2)
        G.add_edge(u, v, weight=weight)

    # Calcolo dei bilanciamenti dei nodi
    imbalances = {}
    for node in G.nodes():
        in_degree = G.in_degree(node)
        out_degree = G.out_degree(node)
        imbalance = out_degree - in_degree
        imbalances[node] = imbalance

    # Identificazione dei nodi sbilanciati
    positive_imbalances = {node: imbalance for node, imbalance in imbalances.items() if imbalance > 0}
    negative_imbalances = {node: -imbalance for node, imbalance in imbalances.items() if imbalance < 0}

    # Se il grafo è già bilanciato, ha un circuito Euleriano
    if not positive_imbalances and not negative_imbalances:
        print("Il grafo è Euleriano.")
        circuit = list(nx.eulerian_circuit(G))
    else:
        print("Il grafo non è Euleriano. Risolvendo il Problema del Postino Cinese per renderlo Euleriano.")

        # Creazione del grafo di flusso
        G_flow = nx.DiGraph()

        # Aggiunta degli archi esistenti con capacità infinita e costo pari al peso originale
        for u, v, data in G.edges(data=True):
            G_flow.add_edge(u, v, capacity=float('inf'), weight=data['weight'])

        # Aggiunta di archi tra nodi sbilanciati
        for u in positive_imbalances:
            for v in negative_imbalances:
                x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
                x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
                weight = math.hypot(x1 - x2, y1 - y2)
                G_flow.add_edge(u, v, capacity=positive_imbalances[u], weight=weight)

        # Aggiunta di un nodo sorgente e pozzo per il problema di flusso
        G_flow.add_node('s')
        G_flow.add_node('t')
        for node, imbalance in positive_imbalances.items():
            G_flow.add_edge('s', node, capacity=imbalance, weight=0)
        for node, imbalance in negative_imbalances.items():
            G_flow.add_edge(node, 't', capacity=imbalance, weight=0)

        # Risoluzione del problema di flusso di costo minimo
        flowCost, flowDict = nx.network_simplex(G_flow)
        print(f"Costo totale per bilanciare il grafo: {flowCost}")

        # Aggiunta degli archi necessari al grafo originale
        for u in G.nodes():
            for v, flow in flowDict[u].items():
                if flow > 0 and u != 's' and v != 't':
                    for _ in range(flow):
                        G.add_edge(u, v, weight=G_flow[u][v]['weight'])

        # Ora il grafo dovrebbe essere Euleriano
        if nx.is_eulerian(G):
            print("Il grafo è stato reso Euleriano.")
            circuit = list(nx.eulerian_circuit(G))
        else:
            print("Errore: Il grafo non è stato reso Euleriano.")
            return

    # Output del circuito Euleriano
    print("\nPercorso per attraversare tutti gli archi nel modo più efficiente:")
    for u, v in circuit:
        print(f"{u} -> {v}")

if __name__ == "__main__":
    main()
