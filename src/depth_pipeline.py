# src/depth_pipeline.py
"""
Pipeline integrado: rectificación → stereo → profundidad
"""

import cv2
import numpy as np
from src.stereo_rectifier import StereoRectifier
from src.stereo_matcher import StereoMatcher
from src.depth_estimator import DepthEstimator


class DepthPipeline:
    """Pipeline completo de estimación de profundidad"""

    def __init__(self, calibration_data, camera_pair=('front', 'right')):
        """
        Args:
            calibration_data: Dict con parámetros de calibración
            camera_pair: Tupla (cam_left, cam_right)
        """
        self.cam_left, self.cam_right = camera_pair

        print(f"\n{'=' * 60}")
        print(f"INICIALIZANDO DEPTH PIPELINE")
        print(f"{'=' * 60}")
        print(f"Par estéreo: {self.cam_left} - {self.cam_right}")
        print(f"{'=' * 60}")

        # Cargar parámetros de calibración
        try:
            K_left = calibration_data[f'K_{self.cam_left}']
            K_right = calibration_data[f'K_{self.cam_right}']
            dist_left = calibration_data[f'dist_{self.cam_left}']
            dist_right = calibration_data[f'dist_{self.cam_right}']
            R = calibration_data[f'R_{self.cam_left}_{self.cam_right}']
            T = calibration_data[f'T_{self.cam_left}_{self.cam_right}']
        except KeyError as e:
            raise ValueError(f"Parámetros de calibración faltantes: {e}\n"
                             f"Asegúrate de haber calibrado el par estéreo.")

        # Inicializar módulos
        print("\n[1/3] Inicializando rectificador...")
        self.rectifier = StereoRectifier(
            K_left, dist_left, K_right, dist_right, R, T, (640, 480)
        )

        print("[2/3] Inicializando stereo matcher...")
        self.stereo_matcher = StereoMatcher(mode='sgbm', use_wls_filter=True)

        print("[3/3] Inicializando depth estimator...")
        # Calcular baseline y focal de cámaras rectificadas
        baseline = self.rectifier.get_baseline()
        focal_length = self.rectifier.get_focal_length()

        self.depth_estimator = DepthEstimator(
            baseline,
            focal_length,
            min_depth=0.5,
            max_depth=30.0
        )

        print(f"{'=' * 60}")
        print("✓ DEPTH PIPELINE LISTO")
        print(f"{'=' * 60}\n")

    def process(self, img_left, img_right, fast_mode=False):
        """
        Procesa par estéreo completo

        Args:
            img_left: Imagen izquierda (sin rectificar)
            img_right: Imagen derecha (sin rectificar)
            fast_mode: Si True, usa versión rápida sin WLS filter

        Returns:
            depth_map: Mapa de profundidad (metros)
            disparity: Mapa de disparidad (píxeles)
            rect_left, rect_right: Imágenes rectificadas
        """
        # 1. Rectificar
        rect_left, rect_right = self.rectifier.rectify_pair(img_left, img_right)

        # 2. Calcular disparidad
        if fast_mode:
            disparity = self.stereo_matcher.compute_disparity_fast(rect_left, rect_right)
        else:
            disparity = self.stereo_matcher.compute_disparity(rect_left, rect_right)

        # 3. Convertir a profundidad
        depth_map = self.depth_estimator.disparity_to_depth(disparity)

        return depth_map, disparity, rect_left, rect_right

    def process_fast(self, img_left, img_right):
        """Versión rápida (sin WLS filter)"""
        return self.process(img_left, img_right, fast_mode=True)

    def get_baseline(self):
        """Retorna baseline en metros"""
        return self.rectifier.get_baseline()

    def get_focal_length(self):
        """Retorna focal length en píxeles"""
        return self.rectifier.get_focal_length()