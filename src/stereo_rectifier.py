# src/stereo_rectifier.py
"""
Rectificación de par estéreo para stereo matching
"""

import cv2
import numpy as np


class StereoRectifier:
    """Rectificación de par estéreo"""

    def __init__(self, K_left, dist_left, K_right, dist_right, R, T, img_size):
        """
        Args:
            K_left, K_right: Matrices de cámara intrínsecos (3x3)
            dist_left, dist_right: Coeficientes de distorsión
            R: Matriz de rotación entre cámaras (3x3)
            T: Vector de traslación entre cámaras (3x1)
            img_size: (width, height) de las imágenes
        """
        self.img_size = img_size
        self.K_left = K_left
        self.K_right = K_right

        print(f"\n{'=' * 60}")
        print("INICIALIZANDO RECTIFICACIÓN ESTÉREO")
        print(f"{'=' * 60}")
        print(f"Tamaño de imagen: {img_size[0]}x{img_size[1]}")

        # Calcular matrices de rectificación
        self.R1, self.R2, self.P1, self.P2, self.Q, roi_left, roi_right = \
            cv2.stereoRectify(
                K_left, dist_left,
                K_right, dist_right,
                img_size, R, T,
                alpha=0,  # 0 = recortar, 1 = mantener todos los píxeles
                newImageSize=img_size
            )

        # Calcular mapas de remapeo (una sola vez para optimización)
        print("Calculando mapas de remapeo...")
        self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
            K_left, dist_left, self.R1, self.P1, img_size, cv2.CV_16SC2
        )

        self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
            K_right, dist_right, self.R2, self.P2, img_size, cv2.CV_16SC2
        )

        # Matriz Q para reproyección 3D
        self.Q_matrix = self.Q

        # Calcular baseline
        self.baseline = abs(T[0, 0])  # Distancia horizontal entre cámaras

        # Calcular focal length de las cámaras rectificadas
        self.focal_length = self.P1[0, 0]

        print(f"✓ Baseline: {self.baseline:.4f} m ({self.baseline * 100:.2f} cm)")
        print(f"✓ Focal length (rectificado): {self.focal_length:.2f} px")
        print(f"✓ ROI izquierda: {roi_left}")
        print(f"✓ ROI derecha: {roi_right}")
        print(f"{'=' * 60}\n")

    def rectify_pair(self, img_left, img_right):
        """
        Rectifica par de imágenes estéreo

        Args:
            img_left: Imagen izquierda
            img_right: Imagen derecha

        Returns:
            rect_left, rect_right: Imágenes rectificadas
        """
        rect_left = cv2.remap(img_left, self.map1_left, self.map2_left,
                              cv2.INTER_LINEAR)
        rect_right = cv2.remap(img_right, self.map1_right, self.map2_right,
                               cv2.INTER_LINEAR)

        return rect_left, rect_right

    def draw_epilines(self, rect_left, rect_right, num_lines=10):
        """
        Dibuja líneas epipolares para verificar rectificación

        Args:
            rect_left, rect_right: Imágenes rectificadas
            num_lines: Número de líneas horizontales a dibujar

        Returns:
            Visualización con líneas epipolares
        """
        # Concatenar horizontalmente
        vis = np.hstack([rect_left, rect_right])
        h, w = rect_left.shape[:2]

        # Dibujar líneas horizontales
        for i in range(0, h, h // num_lines):
            color = tuple(np.random.randint(0, 255, 3).tolist())
            cv2.line(vis, (0, i), (w * 2, i), color, 1)

        # Agregar etiquetas
        cv2.putText(vis, "LEFT", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(vis, "RIGHT", (w + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return vis

    def get_baseline(self):
        """Retorna baseline en metros"""
        return self.baseline

    def get_focal_length(self):
        """Retorna focal length en píxeles"""
        return self.focal_length

    def get_Q_matrix(self):
        """Retorna matriz Q para reproyección 3D"""
        return self.Q_matrix