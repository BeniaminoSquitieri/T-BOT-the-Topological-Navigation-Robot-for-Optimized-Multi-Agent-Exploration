# config.py

import yaml
import cv2
import logging

class Config:
    """
    Classe di configurazione per parametri della mappa e del grafo caricati da un file YAML.
    """
    def __init__(self, config_path, min_feature_size=0.5):
        self.load_config(config_path)
        self.min_feature_size = min_feature_size
        self.calculate_dynamic_parameters()

    def load_config(self, config_path):
        with open(config_path, 'r') as file:
            config_data = yaml.safe_load(file)
        
        # Campi obbligatori
        self.image_path = config_data.get('image')
        self.resolution = config_data.get('resolution')  # metri per pixel
        self.origin = tuple(config_data.get('origin'))  # (x_origin, y_origin)
        self.negate = config_data.get('negate')
        self.occupied_thresh = config_data.get('occupied_thresh')
        self.free_thresh = config_data.get('free_thresh')
        self.merge_threshold = config_data.get('merge_threshold', 50)
        self.max_connection_distance = config_data.get('max_connection_distance', 100000)
        self.line_tolerance = config_data.get('max_connection_distance', 0.06)
        # Verifica se il percorso dell'immagine esiste
        if cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE) is None:
            raise FileNotFoundError(f"Errore: impossibile aprire il file {self.image_path}")
        
    def calculate_dynamic_parameters(self):
        """
        Calcola i parametri dinamici per le operazioni morfologiche basandosi sulla risoluzione.
        """
        # Calcola la dimensione del kernel per ciascuna operazione
        self.kernel_close_size = max(3, int((3.39 * self.min_feature_size) / self.resolution))
        self.kernel_dilate_size = max(3, int((0.8 * self.min_feature_size) / self.resolution))
        self.kernel_open_size = max(3, int((1.0 * self.min_feature_size) / self.resolution))
        self.kernel_erode_size = max(3, int((0.4 * self.min_feature_size) / self.resolution))
        
        # Determina la dimensione minima delle componenti connesse come una frazione dell'area totale
        map_dimensions = self.get_map_dimensions()
        map_area = map_dimensions[0] * map_dimensions[1] * (self.resolution ** 2)
        self.min_component_size = max(50, int(map_area * 0.0005))  # 0.05% dell'area totale con un minimo di 50

    def get_map_dimensions(self):
        """
        Restituisce le dimensioni della mappa in pixel.
        
        Returns:
            tuple: (altezza, larghezza) della mappa in pixel.
        """
        map_image = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)
        return map_image.shape  # (altezza, larghezza)
