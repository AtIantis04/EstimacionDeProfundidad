# src/stereo_matcher.py
"""
Cálculo de mapa de disparidad entre par estéreo
"""

import cv2
import numpy as np


class StereoMatcher:
    """Cálculo de mapa de disparidad"""

    def __init__(self, mode='sgbm', use_wls_filter=True):
        """
        Args:
            mode: 'sgbm' (mejor calidad) o 'bm' (más rápido)
            use_wls_filter: Si usar filtro WLS para refinamiento
        """
        self.mode = mode
        self.use_wls_filter = use_wls_filter

        print(f"\n{'=' * 60}")
        print(f"INICIALIZANDO STEREO MATCHER - Modo: {mode.upper()}")
        print(f"{'=' * 60}")

        if mode == 'sgbm':
            # StereoSGBM (Semi-Global Block Matching) - mejor calidad
            self.stereo = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=16 * 6,  # Debe ser múltiplo de 16 (96)
                blockSize=5,  # Tamaño de ventana (impar, 3-11)
                P1=8 * 3 * 5 ** 2,  # Penalización disparidad suave
                P2=32 * 3 * 5 ** 2,  # Penalización disparidad grande
                disp12MaxDiff=1,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=32,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )
            print("✓ Algoritmo: SGBM (Semi-Global Block Matching)")

        elif mode == 'bm':
            # StereoBM - más rápido pero menos robusto
            self.stereo = cv2.StereoBM_create(
                numDisparities=16 * 6,
                blockSize=15
            )
            print("✓ Algoritmo: BM (Block Matching)")

        # Parámetros actuales
        print(f"✓ Número de disparidades: {self.stereo.getNumDisparities()}")
        print(f"✓ Tamaño de bloque: {self.stereo.getBlockSize()}")

        # WLS Filter para post-procesamiento
        if use_wls_filter:
            try:
                self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.stereo)
                self.wls_filter.setLambda(8000)
                self.wls_filter.setSigmaColor(1.5)

                # Stereo matcher para imagen derecha (necesario para WLS)
                self.right_matcher = cv2.ximgproc.createRightMatcher(self.stereo)

                print("✓ Filtro WLS activado (mejor calidad, más lento)")
            except AttributeError:
                print("⚠ Módulo ximgproc no disponible, WLS filter desactivado")
                self.use_wls_filter = False
                self.wls_filter = None
        else:
            print("✓ Filtro WLS desactivado (más rápido)")
            self.wls_filter = None

        print(f"{'=' * 60}\n")

    def compute_disparity(self, img_left, img_right, use_wls=None):
        """
        Calcula mapa de disparidad entre par estéreo rectificado

        Args:
            img_left: Imagen izquierda (rectificada)
            img_right: Imagen derecha (rectificada)
            use_wls: Usar filtro WLS (None = usar configuración por defecto)

        Returns:
            disparity: Mapa de disparidad (float32, valores <= 0 son inválidos)
        """
        # Usar configuración por defecto si no se especifica
        if use_wls is None:
            use_wls = self.use_wls_filter

        # Convertir a escala de grises si es necesario
        if len(img_left.shape) == 3:
            gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
        else:
            gray_left = img_left
            gray_right = img_right

        # Calcular disparidad
        disp_left = self.stereo.compute(gray_left, gray_right)

        if use_wls and self.wls_filter is not None:
            # Calcular disparidad derecha
            disp_right = self.right_matcher.compute(gray_right, gray_left)

            # Aplicar filtro WLS
            disp_filtered = self.wls_filter.filter(
                disp_left, gray_left, disparity_map_right=disp_right
            )

            # Normalizar
            disparity = disp_filtered.astype(np.float32) / 16.0
        else:
            # Normalizar (SGBM/BM retornan valores * 16)
            disparity = disp_left.astype(np.float32) / 16.0

        # Filtrar valores inválidos
        disparity[disparity <= 0] = -1

        return disparity

    def compute_disparity_fast(self, img_left, img_right):
        """Versión rápida sin WLS filter"""
        return self.compute_disparity(img_left, img_right, use_wls=False)

    def set_num_disparities(self, num_disp):
        """
        Ajusta el número de disparidades

        Args:
            num_disp: Debe ser múltiplo de 16
        """
        if num_disp % 16 != 0:
            num_disp = ((num_disp // 16) + 1) * 16
            print(f"⚠ Ajustado a múltiplo de 16: {num_disp}")

        self.stereo.setNumDisparities(num_disp)
        print(f"✓ Número de disparidades actualizado: {num_disp}")

    def set_block_size(self, block_size):
        """
        Ajusta el tamaño de bloque

        Args:
            block_size: Debe ser impar
        """
        if block_size % 2 == 0:
            block_size += 1
            print(f"⚠ Ajustado a impar: {block_size}")

        self.stereo.setBlockSize(block_size)
        print(f"✓ Tamaño de bloque actualizado: {block_size}")

    def get_disparity_visualization(self, disparity, colormap=cv2.COLORMAP_JET):
        """
        Genera visualización coloreada del mapa de disparidad

        Args:
            disparity: Mapa de disparidad
            colormap: Mapa de colores de OpenCV

        Returns:
            Imagen BGR coloreada
        """
        # Normalizar a 0-255
        disp_vis = disparity.copy()
        valid_mask = disp_vis > 0

        if valid_mask.any():
            min_disp = np.min(disp_vis[valid_mask])
            max_disp = np.max(disp_vis[valid_mask])

            disp_vis[valid_mask] = ((disp_vis[valid_mask] - min_disp) /
                                    (max_disp - min_disp) * 255)

        disp_vis = disp_vis.astype(np.uint8)

        # Aplicar colormap
        disp_colored = cv2.applyColorMap(disp_vis, colormap)

        # Establecer píxeles inválidos en negro
        disp_colored[~valid_mask] = 0

        return disp_colored