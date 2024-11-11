# PHASE 1

# Generazione di una Mappa Topologica da una Mappa di Occupazione

## Introduzione
Questo modulo ha l'obiettivo di trasformare una mappa di occupazione (occupancy grid map) in una mappa topologica. Questa rappresentazione è ideale per applicazioni di robotica, come la navigazione autonoma e la pianificazione dei percorsi in ambienti interni complessi, consentendo al robot di comprendere le relazioni spaziali tra punti chiave dell'ambiente.

## Descrizione Generale
Il processo di generazione della mappa topologica si articola nei seguenti passaggi:

1. **Caricamento della Mappa di Occupazione**: Carica l'immagine della mappa.
2. **Creazione della Mappa Binaria**: Trasforma l'immagine in una rappresentazione binaria (ostacoli/spazi liberi).
3. **Calcolo della Mappa delle Distanze Euclidee**: Determina la distanza minima di ciascun punto dagli ostacoli.
4. **Generazione delle Linee di Voronoi**: Identifica i percorsi equidistanti dagli ostacoli.
5. **Scheletrizzazione delle Linee di Voronoi**: Riduce le linee a una rappresentazione di un pixel di larghezza.
6. **Creazione del Grafo Topologico**: Costruisce il grafo a partire dallo scheletro, con nodi e archi.
7. **Salvataggio dei Risultati**: Esporta immagini e file di configurazione per l'uso con ROS2.

## Passaggi Dettagliati

### 1. Caricamento della Mappa di Occupazione
- **Obiettivo**: Caricare l'immagine della mappa in scala di grigi.
- **Metodo**: Utilizza OpenCV per leggere l'immagine e ottenere una matrice rappresentante i livelli di occupazione.

### 2. Creazione della Mappa Binaria
- **Obiettivo**: Convertire l'immagine in una mappa binaria dove:
  - 0 (nero): rappresenta gli ostacoli.
  - 1 (bianco): rappresenta gli spazi liberi.
- **Motivazione**: Una rappresentazione binaria facilita il calcolo delle distanze e la generazione delle linee di Voronoi.

### 3. Calcolo della Mappa delle Distanze Euclidee
- **Obiettivo**: Calcolare, per ogni pixel libero, la distanza minima dall'ostacolo più vicino.
- **Motivazione**: La mappa delle distanze è fondamentale per generare le linee di Voronoi, che rappresentano i luoghi equidistanti dagli ostacoli.

### 4. Generazione delle Linee di Voronoi
- **Obiettivo**: Identificare i percorsi equidistanti dagli ostacoli, noti come linee di Voronoi.
- **Metodo**:
  - Applica un filtro per calcolare la differenza tra i valori di distanza dei pixel adiacenti.
  - I punti con differenze significative indicano posizioni equidistanti da più ostacoli.
- **Motivazione**: Le linee di Voronoi rappresentano i percorsi centrali, ideali per la navigazione robotica.

### 5. Scheletrizzazione delle Linee di Voronoi
- **Obiettivo**: Ridurre le linee di Voronoi a linee sottili di un pixel di larghezza.
- **Motivazione**: La scheletrizzazione facilita l'identificazione dei nodi e migliora l'accuratezza della costruzione del grafo topologico.

### 6. Creazione del Grafo Topologico
- **Obiettivo**: Costruire un grafo che rappresenta la mappa, composto da nodi e archi.
- **Passaggi**:
  1. **Identificazione dei Nodi**: Riconosce i nodi come punti di incrocio e estremità sullo scheletro.
  2. **Distribuzione dei Nodi**: Utilizza l'algoritmo di clustering DBSCAN per distribuire uniformemente i nodi. 
     - **DBSCAN (Density-Based Spatial Clustering of Applications with Noise)** è un algoritmo di clustering che raggruppa punti vicini in base alla densità dei dati. I punti vengono raggruppati se si trovano entro una certa distanza (parametro `eps`) e se hanno almeno un numero minimo di punti vicini (`min_samples`). Questo metodo è particolarmente utile in scenari dove ci sono zone di alta densità di punti che rappresentano nodi molto vicini tra loro, che possono essere fusi in un unico cluster. 

### 7. Salvataggio dei Risultati
- **Immagini Salvate**:
  - Mappa binaria
  - Mappa delle distanze normalizzata
  - Mappa di Voronoi
  - Scheletro di Voronoi
  - Mappa con i nodi sovrapposti

## Requisiti

### Librerie Python Necessarie
Le seguenti librerie Python sono richieste per eseguire il modulo:

- `numpy`
- `opencv-python`
- `networkx`
- `scipy`
- `scikit-image`
- `Pillow`
- `PyYAML`

### Ambiente di Esecuzione
- Python 3.x
- ROS2 Humble (se si vuole utilizzare la mappa con il sistema ROS2)

## Guida all'Utilizzo

### Installazione delle Dipendenze
Esegui il seguente comando per installare tutte le librerie necessarie:
```bash
pip install numpy opencv-python networkx scipy scikit-image Pillow PyYAML
```
### Esecuzione dello Script
Per eseguire lo script, assicurati di specificare il percorso dell'immagine della mappa.
```bash
python topological_map.py diem_map.pgm 
```

# Visualizzazione dei Risultati in RVIZ

Dopo aver generato la mappa, puoi visualizzarla in RVIZ per verificarne l'accuratezza. Segui i passaggi riportati di seguito.

## Passaggi per Visualizzare la Mappa

1. **Assicurati di aver copiato o specificato il percorso corretto della mappa YAML generata.**

2. **Esegui i seguenti comandi in tre diverse shell per lanciare i moduli necessari alla visualizzazione e navigazione della mappa.**

3. **Non dimenticarti di andare nella directory della mappa** 
- /turtlebot4/diem_turtlebot_ws/src/map/map_transformation_phase/diem_map_topological
### Shell 1 - Avvia la Localizzazione Utilizzando la Mappa
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=<map_yaml_file_path>
```
Nel mio caso: 

```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map/diem_map.yaml

```

### Shell 2 - Avvia il Modulo di Navigazione
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```
### Shell 3 - Avvia RVIZ per la Visualizzazione del Robot
```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```
> **Nota:** Sostituisci `<map_yaml_file_path>` con il percorso della mappa YAML generata.

## Preparazione dell'Ambiente

- Assicurati che l'immagine della mappa di occupazione sia disponibile e accessibile.
- Modifica il valore di `diem_map.pgm` nel comando con il percorso della tua mappa.

## Configurazione dei Parametri
- **image_path:** Specifica il percorso dell'immagine della mappa.
- **max_nodes:** Specifica il numero massimo di nodi da includere nel grafo topologico. Se non impostato, verranno utilizzati tutti i nodi disponibili.

## Output e Visualizzazione dei Risultati

Al termine dell'esecuzione, verrà creata una cartella con il nome della mappa che conterrà:

### 1. Immagini Salvate
- Mappa scheletrizzata
- Mappa con i nodi sovrapposti
- Mappa binaria
- Mappa delle distanze normalizzata
- Mappa di Voronoi

### 2. File YAML
- Contiene informazioni come risoluzione e origine della mappa per l'integrazione con ROS2.

### 3. Rappresentazione del Grafo Topologico
- Formato JSON con la struttura del grafo topologico.
# PHASE 2

## File di Visualizzazione del Grafo

Il file `visualize_graph.py` consente di visualizzare il grafo topologico generato sovrapponendolo a un'immagine di mappa di sfondo e di salvare il risultato sia in formato PNG che PGM.

### Requisiti

- `matplotlib`
- `networkx`
- `Pillow`
- `numpy`

Assicurati di aver installato queste librerie utilizzando il seguente comando:

```bash
pip install matplotlib networkx pillow numpy
```

### Guida all'Utilizzo

Per visualizzare il grafo topologico, esegui il seguente comando passando il percorso del file JSON generato:

```bash
python visualize_graph.py <path_to_json>
```

### Descrizione del File `visualize_graph.py`

- **Input**:
  - Un file JSON contenente la descrizione del grafo topologico (nodi e archi) con coordinate basate sul sistema RVIZ.
  - Il percorso dell'immagine `diem_map.pgm` utilizzata come sfondo della mappa.
- **Output**: Due file immagine del grafo topologico sovrapposto alla mappa, salvati nella directory `graph`:
  - Un file PNG per la visualizzazione.
  - Un file PGM per l'utilizzo con strumenti che richiedono formati di immagine compatibili con ROS.
  
- **Funzionalità**:
  - Converte le coordinate del grafo da RVIZ a coordinate pixel, utilizzando i valori di `origin` e `resolution` specificati nella configurazione YAML della mappa.
  - Disegna il grafo topologico utilizzando `matplotlib` e `networkx` sopra l'immagine della mappa.
  - Salva l'immagine del grafo sovrapposto in formato PNG e PGM nella cartella `graph`.

### Esempio di Utilizzo

Supponiamo di aver generato un file JSON chiamato `navigation_graph.json`. Per visualizzare e sovrapporre il grafo alla mappa come immagine:

```bash
python visualize_graph.py navigation_graph.json
```

Dopo l'esecuzione, verranno create due immagini nella cartella `graph`:
- `navigation_graph_graph_map.png`: la sovrapposizione del grafo alla mappa in formato PNG.
- `navigation_graph_graph_map.pgm`: la sovrapposizione del grafo alla mappa in formato PGM, utile per il caricamento in ROS.
