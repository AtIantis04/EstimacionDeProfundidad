"""
Funciones auxiliares para el sistema
"""

import cv2
import yaml
import os
from collections import deque
import time


def load_config(config_path):
    """Carga archivo de configuración YAML"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def create_output_dirs():
    """Crea directorios de salida si no existen"""
    dirs = ['output', 'calibration_data', 'calibration_data/stereo']
    for d in dirs:
        os.makedirs(d, exist_ok=True)


class FPSCounter:
    """Contador de FPS con buffer"""

    def __init__(self, buffer_size=30):
        self.buffer = deque(maxlen=buffer_size)
        self.last_time = time.time()

    def update(self, processing_time=None):
        """Actualiza contador"""
        if processing_time is None:
            current_time = time.time()
            processing_time = current_time - self.last_time
            self.last_time = current_time

        if processing_time > 0:
            fps = 1.0 / processing_time
            self.buffer.append(fps)

    def get_fps(self):
        """Obtiene FPS promedio"""
        if len(self.buffer) == 0:
            return 0.0
        return sum(self.buffer) / len(self.buffer)

    def reset(self):
        """Resetea contador"""
        self.buffer.clear()
        self.last_time = time.time()