"""
Conversión de disparidad a profundidad métrica
"""

import cv2
import numpy as np


class DepthEstimator:
    """Conversión de disparidad a profundidad métrica"""

    def __init__(self, baseline, focal_length, min_depth=0.5, max_depth=50.0):
        """
        Args:
            baseline: Distancia entre cámaras en metros (ej: 0.35)
            focal_length: Distancia focal en píxeles (de matriz K rectificada)
            min_depth: Profundidad mínima válida en metros
            max_depth: Profundidad máxima válida en metros
        """
        self.baseline = baseline
        self.focal_length = focal_length
        self.min_depth = min_depth
        self.max_depth = max_depth

        print(f"\n{'=' * 60}")
        print("INICIALIZANDO DEPTH ESTIMATOR")
        print(f"{'=' * 60}")
        print(f"✓ Baseline: {baseline:.4f} m ({baseline * 100:.2f} cm)")
        print(f"✓ Focal length: {focal_length:.2f} px")
        print(f"✓ Rango de profundidad: {min_depth:.1f}m - {max_depth:.1f}m")
        print(f"{'=' * 60}\n")

    def disparity_to_depth(self, disparity):
        """
        Convierte mapa de disparidad a profundidad

        Fórmula: Z = (f * B) / d
        donde:
            Z = profundidad (metros)
            f = focal length (píxeles)
            B = baseline (metros)
            d = disparidad (píxeles)

        Args:
            disparity: Mapa de disparidad (float32)

        Returns:
            depth_map: Profundidad en metros (float32)
        """
        # Evitar división por cero
        disparity_safe = disparity.copy()
        disparity_safe[disparity_safe <= 0] = 0.01

        # Cálculo de profundidad
        depth = (self.focal_length * self.baseline) / disparity_safe

        # Filtrar valores fuera de rango
        depth[disparity <= 0] = 0
        depth[depth < self.min_depth] = 0
        depth[depth > self.max_depth] = 0

        return depth.astype(np.float32)

    def depth_to_disparity(self, depth):
        """
        Conversión inversa (útil para validación)

        Args:
            depth: Mapa de profundidad en metros

        Returns:
            disparity: Mapa de disparidad en píxeles
        """
        depth_safe = depth.copy()
        depth_safe[depth_safe == 0] = self.max_depth

        disparity = (self.focal_length * self.baseline) / depth_safe
        disparity[depth == 0] = 0

        return disparity

    def get_depth_colored(self, depth_map, colormap=cv2.COLORMAP_JET):
        """
        Genera visualización coloreada del mapa de profundidad

        Args:
            depth_map: Mapa de profundidad en metros
            colormap: Mapa de colores de OpenCV

        Returns:
            depth_colored: Imagen BGR para visualización
        """
        # Normalizar a rango 0-255
        depth_normalized = depth_map.copy()
        valid_mask = depth_normalized > 0

        if valid_mask.any():
            depth_normalized[valid_mask] = np.clip(
                (depth_normalized[valid_mask] - self.min_depth) /
                (self.max_depth - self.min_depth) * 255,
                0, 255
            )

        depth_normalized = depth_normalized.astype(np.uint8)

        # Aplicar colormap
        depth_colored = cv2.applyColorMap(depth_normalized, colormap)

        # Establecer píxeles inválidos en negro
        depth_colored[~valid_mask] = 0

        return depth_colored

    def get_depth_statistics(self, depth_map):
        """
        Calcula estadísticas del mapa de profundidad

        Args:
            depth_map: Mapa de profundidad

        Returns:
            dict con estadísticas
        """
        valid_depths = depth_map[depth_map > 0]

        if len(valid_depths) == 0:
            return {
                'valid_pixels': 0,
                'valid_percentage': 0.0,
                'min_depth': 0.0,
                'max_depth': 0.0,
                'mean_depth': 0.0,
                'median_depth': 0.0
            }

        total_pixels = depth_map.size
        valid_pixels = len(valid_depths)

        return {
            'valid_pixels': valid_pixels,
            'valid_percentage': (valid_pixels / total_pixels) * 100,
            'min_depth': np.min(valid_depths),
            'max_depth': np.max(valid_depths),
            'mean_depth': np.mean(valid_depths),
            'median_depth': np.median(valid_depths)
        }