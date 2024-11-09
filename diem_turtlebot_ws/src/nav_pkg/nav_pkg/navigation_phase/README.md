# Risoluzione del Problema del Postino Cinese su un Grafo Orientato

## Introduzione

Lo script *cpp.py* fornisce un'implementazione in Python per risolvere il **Problema del Postino Cinese (Chinese Postman Problem)** su un grafo orientato e pesato. Il problema consiste nel trovare un percorso di costo minimo che attraversa tutti gli archi di un grafo almeno una volta, ottimizzando il tragitto per minimizzare il costo totale. Questo tipo di problema è rilevante in vari contesti reali, come la pianificazione delle rotte di consegna, la manutenzione stradale e la programmazione di robot per la copertura completa di un'area.

## Descrizione del Progetto

Lo scopo è quello di risolvere il problema del postino cinese su un grafo orientato e pesato, fornendo un percorso ottimale che minimizzi il costo necessario per percorrere tutti gli archi. Per raggiungere questo obiettivo, il codice effettua diversi passaggi fondamentali, descritti di seguito.

### Panoramica del Flusso di Lavoro

1. **Lettura del Grafo**: Il programma legge un grafo da un file JSON specificato dall'utente. Il grafo contiene nodi ed archi con pesi che rappresentano, ad esempio, la distanza tra due punti.

2. **Verifica Euleriana del Grafo**: Il codice verifica se il grafo è Euleriano, ovvero se esiste un percorso che attraversa ogni arco esattamente una volta senza ripassare su nessun arco inutilmente.

3. **Bilanciamento del Grafo**: Se il grafo non è Euleriano, il programma bilancia i nodi aggiungendo archi a costo minimo per renderlo Euleriano.

4. **Ricerca del Circuito Euleriano**: Una volta bilanciato il grafo, viene calcolato un circuito Euleriano che rappresenta il percorso ottimale desiderato.

5. **Output del Percorso**: Il programma stampa il percorso di costo minimo, mostrando la sequenza dei nodi attraversati.

## Dipendenze

- **Python 3.x**: Linguaggio di programmazione utilizzato per implementare il progetto.
- **NetworkX**: Una libreria Python utilizzata per la creazione, manipolazione e studio della struttura, dinamica e funzioni di grafi complessi. Questa libreria è fondamentale per il calcolo delle proprietà del grafo e la risoluzione del problema di flusso.

Per installare NetworkX, è possibile utilizzare il seguente comando:

```bash
pip install networkx
```

## Struttura del Grafo in JSON

Il file JSON che rappresenta il grafo deve contenere due sezioni principali:

- **nodes**: Elenco di nodi con le relative coordinate (`x`, `y`). Ogni nodo è identificato da un'etichetta unica.
- **edges**: Elenco di archi del grafo. Ogni arco deve avere un nodo di origine (`from`), un nodo di destinazione (`to`), e può includere un peso (`weight`). Se il peso non è fornito, verrà calcolato automaticamente come la distanza euclidea tra i nodi.

## Descrizione dei Passaggi del Codice

### 1. Parsing degli Argomenti

Il programma utilizza il modulo `argparse` per accettare l'input dell'utente dalla linea di comando, specificando il percorso al file JSON contenente il grafo. Questo consente un'esecuzione flessibile, permettendo all'utente di lavorare con qualsiasi grafo desiderato, purché rispetti la struttura definita.

### 2. Creazione e Configurazione del Grafo

Il grafo viene creato utilizzando `nx.DiGraph()` di NetworkX, che rappresenta un grafo orientato. I nodi vengono aggiunti con le loro coordinate come attributi, mentre gli archi vengono creati con pesi calcolati (se non specificati), basati sulla distanza euclidea tra i nodi.

### 3. Calcolo del Bilanciamento dei Nodi

Ogni nodo nel grafo viene analizzato per calcolare il bilanciamento, determinato come la differenza tra il numero di archi uscenti e il numero di archi entranti. Il bilanciamento viene utilizzato per determinare se il grafo è Euleriano o meno:

- Un **grafo è Euleriano** se tutti i nodi hanno lo stesso numero di archi entranti e uscenti.
- Se ci sono **nodi sbilanciati**, il grafo non è Euleriano e deve essere bilanciato.

### 4. Bilanciamento del Grafo Non Euleriano

Se il grafo non è Euleriano, il codice esegue i seguenti passaggi per bilanciarlo:

- **Nodi sbilanciati** vengono divisi in due gruppi: quelli con **più archi uscenti** che entranti (sbilanciamento positivo) e quelli con **più archi entranti** che uscenti (sbilanciamento negativo).
- Viene creato un **grafo di flusso** che include sia gli archi esistenti che nuovi archi potenziali per bilanciare il grafo. Gli archi vengono aggiunti in base al costo minimo calcolato (es. distanza euclidea).
- **Nodo sorgente 's'** e **nodo pozzo 't'** vengono aggiunti per modellare il problema di flusso, collegando rispettivamente i nodi sbilanciati con archi a costo zero.

### 5. Risoluzione del Problema di Flusso di Costo Minimo

Il codice utilizza l'algoritmo **Network Simplex** di NetworkX per risolvere il problema di flusso di costo minimo. Questo approccio consente di trovare il set di archi da aggiungere al grafo originale per bilanciarlo nel modo più economico possibile.

### 6. Aggiunta degli Archi al Grafo Originale

Gli archi risultanti dal flusso di costo minimo vengono aggiunti al grafo originale per bilanciarlo, assicurandosi che diventi Euleriano.

### 7. Ricerca e Output del Circuito Euleriano

Una volta che il grafo è bilanciato, viene verificato nuovamente se è **Euleriano**. In caso affermativo, il programma calcola un **circuito Euleriano**, che rappresenta il percorso ottimale per attraversare tutti gli archi con il minimo costo.

Il percorso viene poi stampato, mostrando la sequenza di nodi attraversati per coprire l'intero grafo nel modo più efficiente.

## Perché Questo Algoritmo È Preferito

### Vantaggi dell'Algoritmo

1. **Efficienza nel Coprire Tutti gli Archi**: L'algoritmo mira a trovare il percorso di costo minimo che attraversa tutti gli archi almeno una volta.
2. **Bilanciamento Ottimale**: Aggiunge il minor numero possibile di archi (o archi di costo minimo) per rendere il grafo Euleriano, evitando percorsi inutilmente lunghi.
3. **Applicabilità a Grafi Orientati e Pesati**: Può gestire grafi complessi con direzioni specifiche e pesi diversi, tipici di molte applicazioni del mondo reale.

### Vantaggi Rispetto ad Altri Algoritmi

- **Algoritmo di Fleury**: È semplice ma non efficiente per grafi grandi o non Euleriani. Quest' approccio offre un approccio più scalabile.
- **Algoritmo di Hierholzer**: Hierholzer funziona solo su grafi Euleriani; il nostro algoritmo è in grado di **bilanciare un grafo non Euleriano** per trovare un circuito ottimale.

## Applicazioni Pratiche

- **Servizi di Consegna e Raccolta Rifiuti**: Ottimizzazione delle rotte per minimizzare il tempo e il costo complessivo.
- **Manutenzione Stradale**: Pianificazione di percorsi per ispezionare o mantenere le strade di una città.
- **Robotica e Automazione**: Programmazione di robot per coprire completamente un'area, come in applicazioni di pulizia o sorveglianza.

## Come Eseguire il Codice

### Preparare il File JSON

Creare un file JSON contenente il grafo, definendo nodi e archi (inclusi i pesi se disponibili, ma non è il nostro caso). Assicurarsi che il file rispetti la struttura richiesta.

### Eseguire il Programma

Esegui il seguente comando passando il percorso del file JSON:

```bash
python your_script.py path_to_graph.json
```

Sostituire `your_script.py` con il nome del file contenente il codice e `path_to_graph.json` con il percorso al file JSON del grafo.

## Altre Considerazioni

### Limitazioni

- **Complessità Computazionale**: Il problema del Postino Cinese su grafi orientati è computazionalmente intensivo, specialmente per grafi molto grandi.
- **Assunzioni sui Dati**: Si presume che il grafo sia connesso e che i pesi degli archi siano non negativi.

### Possibili Miglioramenti

- **Parallelizzazione**: Per grafi molto grandi, si potrebbe considerare l'uso di algoritmi paralleli o distribuiti per velocizzare il calcolo.
- **Interfaccia Utente**: Potrebbe essere utile aggiungere una GUI per facilitare l'input dei dati e la visualizzazione dei risultati.
- **Visualizzazione Grafica**: Integrare la visualizzazione del grafo e del percorso trovato utilizzando librerie come Matplotlib.

