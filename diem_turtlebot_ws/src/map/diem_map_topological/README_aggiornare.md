# Generazione di una Mappa Topologica da una Mappa di Occupazione

## Introduzione
Il seguente script ha lo scopo di generare una mappa topologica a partire da una mappa di occupazione (occupancy grid map). La mappa topologica risultante è essenziale per applicazioni di robotica, come la navigazione autonoma e la pianificazione dei percorsi in ambienti indoor complessi.

## Descrizione Generale
Il processo consiste in una serie di passaggi che trasformano la mappa di occupazione in una rappresentazione topologica utilizzabile:

1. Caricamento della Mappa di Occupazione
2. Creazione della Mappa Binaria
3. Calcolo della Mappa delle Distanze Euclidee
4. Generazione delle Linee di Voronoi
5. Scheletrizzazione delle Linee di Voronoi
6. Creazione del Grafo Topologico
7. Salvataggio dei Risultati

## Passaggi Dettagliati

### 1. Caricamento della Mappa di Occupazione
- **Obiettivo**: Caricare l'immagine della mappa in scala di grigi.

### 2. Creazione della Mappa Binaria
- **Obiettivo**: Convertire l'immagine in una mappa binaria dove:
  - 0 (nero): rappresenta gli ostacoli.
  - 1 (bianco): rappresenta gli spazi liberi.
- **Motivazione**: Una mappa binaria è essenziale per calcolare correttamente la mappa delle distanze e per applicare operazioni di morfologia matematica.

### 3. Calcolo della Mappa delle Distanze Euclidee
- **Obiettivo**: Calcolare, per ogni pixel libero, la distanza minima dall'ostacolo più vicino.
- **Motivazione**: La mappa delle distanze è fondamentale per generare le linee di Voronoi, che rappresentano i luoghi equidistanti dagli ostacoli circostanti.

### 4. Generazione delle Linee di Voronoi
- **Obiettivo**: Identificare le linee equidistanti dagli ostacoli, note come linee di Voronoi.
- **Metodo**:
  - Utilizzare un filtro che calcola la differenza tra il valore massimo e minimo dei pixel vicini.
  - I punti con differenze significative indicano posizioni equidistanti da più ostacoli.
- **Motivazione**: Le linee di Voronoi rappresentano i percorsi centrali nell'ambiente, ideali per la navigazione robotica.

### 5. Scheletrizzazione delle Linee di Voronoi
- **Obiettivo**: Ridurre le linee di Voronoi a linee sottili di un pixel di larghezza.
- **Motivazione**:
  - Facilitare l'identificazione di nodi come incroci ed estremità.
  - Migliorare l'accuratezza nella costruzione del grafo topologico.
  
  **Nota Importante**: Anche in ambienti statici, le linee di Voronoi possono essere spesse a causa della natura del calcolo delle distanze. La scheletrizzazione è necessaria per ottenere linee sottili adatte all'analisi topologica.

### 6. Creazione del Grafo Topologico
- **Obiettivo**: Costruire un grafo che rappresenta la mappa, composto da nodi e archi.
- **Passaggi**:
  1. **Identificazione dei Nodi**:
     - I nodi sono punti nello scheletro con un numero di vicini diverso da 2 (estremità e incroci).
  2. **Distribuzione Uniforme dei Nodi**:
     - Suddividere la mappa in una griglia basata su `max_nodes`.
     - In ogni cella, selezionare un nodo rappresentativo.
  3. **Creazione degli Archi**:
     - Verificare se esiste un percorso nello scheletro tra coppie di nodi.
     - Aggiungere un arco nel grafo se il percorso esiste.

- **Motivazione**:
  - Una distribuzione uniforme dei nodi assicura una copertura completa dell'area.
  - Il grafo topologico facilita la pianificazione dei percorsi e la navigazione.

### 7. Salvataggio dei Risultati
- **Immagini Salvate**:
  - Mappa binaria.
  - Mappa delle distanze normalizzata.
  - Mappa di Voronoi.
  - Scheletro di Voronoi.
  - Mappa scheletrizzata con nodi sovrapposti.
- **File YAML**:
  - Contiene informazioni sulla mappa (es. risoluzione, origine) per l'utilizzo in ROS2.
  
- **Motivazione**:
  - Salvare le immagini intermedie permette di visualizzare e verificare ogni fase del processo.
  - Il file YAML è necessario per l'integrazione con sistemi robotici.

## Requisiti

### Librerie Python Necessarie:
- `numpy`
- `opencv-python`
- `networkx`
- `scipy`
- `scikit-image`
- `Pillow`
- `PyYAML`

### Ambiente di Esecuzione:
- Python 3.x

## Guida all'Utilizzo

### Installazione delle Dipendenze:
Eseguire il seguente comando per installare tutte le librerie necessarie:
```bash
pip install numpy opencv-python networkx scipy scikit-image Pillow PyYAML
```
### Eseguire il file
Senza specificare il numero massimo di nodi (userà tutti i nodi disponibili):

```bash
py .\topological_map.py diem_map.pgm
``` 
### Eseguire il file
Specificando il numero massimo di nodi (ad esempio, 50):

```bash
py .\topological_map.py diem_map.pgm --max_nodes 50
``` 

### Try the map in RVIZ
Per provare la mappa in RVIZ esegui i seguenti comandi: 
Assicurarsi di stare nella directory dove è presente la mappa oppure in alternativa indicare il percorso della mappa.

Shell 1: 

```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=<map_yaml_file_path>
``` 
Shell 2: 

```bash
ros2 launch turtlebot4_navigation nav2.launch.py
``` 
Shell 3: 

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
``` 
### Preparazione dell'Ambiente: 
- Assicurarsi che l'immagine della mappa di occupazione sia disponibile e accessibile.
- Sostituire 'diem_map.pgm' nel codice con il percorso corretto della vostra mappa.

### Configurazione dei Parametri:
- image_path: Il percorso dell'immagine della mappa.
- max_nodes: Il numero massimo di nodi da includere nel grafo topologico (ad esempio, 50).

### Visualizzazione dei Risultati:
- Al termine dell'esecuzione, verrà creata una cartella con il nome della mappa. 
- All'interno della cartella troverete:

1. Le immagini dei vari passaggi del processo.
2. La mappa topologica con i nodi sovrapposti.
3. Il file YAML per l'utilizzo in ROS2.
