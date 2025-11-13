# src/panorama_stitcher.py
"""
Generador de vista panorámica multi-cámara
"""

import cv2
import numpy as np


class PanoramaStitcher:
    """Generador de vista panorámica desde múltiples cámaras"""

    def __init__(self):
        self.homographies = {}  # Homografías precalculadas
        self.canvas_size = None
        self.offset = None
        self.initialized = False

        # Detector de características (para cálculo inicial de homografías)
        try:
            self.feature_detector = cv2.SIFT_create(nfeatures=1000)
        except:
            # Fallback a ORB si SIFT no está disponible
            self.feature_detector = cv2.ORB_create(nfeatures=1000)

        self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)

        print(f"\n{'=' * 60}")
        print("INICIALIZANDO PANORAMA STITCHER")
        print(f"{'=' * 60}")
        print("✓ Detector de características listo")
        print(f"{'=' * 60}\n")

    def compute_homographies(self, frames_ref):
        """
        Calcula homografías entre cámaras usando frames de referencia
        Solo se ejecuta una vez al inicio o cuando cambie la configuración

        Args:
            frames_ref: Dict con frames de referencia {'front': img, 'right': img, ...}
        """
        print("\n⏳ Calculando homografías para panorama...")

        if 'front' not in frames_ref:
            print("❌ Se requiere cámara 'front' como referencia")
            return False

        base_frame = frames_ref['front']
        h_base, w_base = base_frame.shape[:2]

        # La cámara frontal es la referencia (identidad)
        self.homographies['front'] = np.eye(3, dtype=np.float32)

        # Calcular homografías para otras cámaras
        for cam_id, frame in frames_ref.items():
            if cam_id == 'front':
                continue

            print(f"  Calculando homografía: front → {cam_id}...")

            H = self._compute_homography(base_frame, frame)

            if H is not None:
                self.homographies[cam_id] = H
                print(f"    ✓ Homografía calculada")
            else:
                print(f"    ⚠ No se pudo calcular, usando identidad")
                self.homographies[cam_id] = np.eye(3, dtype=np.float32)

        # Calcular tamaño del canvas
        self._compute_canvas_size(frames_ref)
        self.initialized = True

        print(f"\n✓ Panorama inicializado")
        print(f"  Canvas: {self.canvas_size[0]}x{self.canvas_size[1]} px")
        print(f"  Offset: {self.offset}")

        return True

    def _compute_homography(self, img1, img2):
        """
        Calcula homografía entre dos imágenes

        Args:
            img1: Imagen base (referencia)
            img2: Imagen a transformar

        Returns:
            H: Matriz de homografía 3x3 o None si falla
        """
        # Detectar características
        kp1, des1 = self.feature_detector.detectAndCompute(img1, None)
        kp2, des2 = self.feature_detector.detectAndCompute(img2, None)

        if des1 is None or des2 is None:
            return None

        # Emparejar características
        try:
            matches = self.matcher.knnMatch(des1, des2, k=2)
        except:
            return None

        # Ratio test de Lowe
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)

        if len(good_matches) < 10:
            return None

        # Extraer puntos correspondientes
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Calcular homografía con RANSAC
        H, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)

        return H

    def _compute_canvas_size(self, frames):
        """Calcula dimensiones del canvas panorámico"""
        h, w = frames['front'].shape[:2]

        # Estimación conservadora para 4 cámaras
        # Depende de la disposición de las cámaras
        num_cams = len(frames)

        if num_cams == 1:
            self.canvas_size = (w, h)
            self.offset = (0, 0)
        elif num_cams == 2:
            self.canvas_size = (int(w * 1.8), h)
            self.offset = (w // 4, 0)
        elif num_cams >= 3:
            # Para 3-4 cámaras: canvas más grande
            self.canvas_size = (int(w * 2.5), int(h * 1.5))
            self.offset = (w // 2, h // 4)

        print(f"  Canvas calculado: {self.canvas_size}")

    def stitch(self, frames, mode='fast'):
        """
        Une frames en panorama

        Args:
            frames: Dict con frames actuales {'front': img, 'right': img, ...}
            mode: 'fast' (sin blending) o 'quality' (con blending)

        Returns:
            panorama: Imagen panorámica
        """
        if not self.initialized:
            print("⚠ Panorama no inicializado. Ejecuta compute_homographies() primero")
            # Intentar inicializar con frames actuales
            self.compute_homographies(frames)

        # Crear canvas vacío
        panorama = np.zeros((self.canvas_size[1], self.canvas_size[0], 3),
                            dtype=np.uint8)

        if mode == 'quality':
            # Modo con blending (más lento pero mejor calidad)
            return self._stitch_with_blending(frames, panorama)
        else:
            # Modo rápido (sin blending)
            return self._stitch_fast(frames, panorama)

    def _stitch_fast(self, frames, panorama):
        """Stitching rápido sin blending"""

        # Orden de warping: cámaras traseras primero, frontal al final
        # para que la vista frontal tenga prioridad en solapamientos
        order = []

        # Determinar orden según cámaras disponibles
        if 'back' in frames:
            order.append('back')
        if 'left' in frames:
            order.append('left')
        if 'right' in frames:
            order.append('right')
        if 'front' in frames:
            order.append('front')

        for cam_id in order:
            if cam_id not in frames or cam_id not in self.homographies:
                continue

            frame = frames[cam_id]
            H = self.homographies[cam_id].copy()

            # Ajustar homografía con offset
            H[0, 2] += self.offset[0]
            H[1, 2] += self.offset[1]

            # Warp frame
            warped = cv2.warpPerspective(
                frame, H,
                (self.canvas_size[0], self.canvas_size[1])
            )

            # Combinar (overlay simple)
            mask = (warped > 0).any(axis=2)
            panorama[mask] = warped[mask]

        return panorama

    def _stitch_with_blending(self, frames, panorama):
        """Stitching con blending suave (mejor calidad)"""

        # Acumuladores para blending
        panorama_float = np.zeros((self.canvas_size[1], self.canvas_size[0], 3),
                                  dtype=np.float32)
        weight_sum = np.zeros((self.canvas_size[1], self.canvas_size[0]),
                              dtype=np.float32)

        for cam_id, frame in frames.items():
            if cam_id not in self.homographies:
                continue

            H = self.homographies[cam_id].copy()
            H[0, 2] += self.offset[0]
            H[1, 2] += self.offset[1]

            # Crear máscara de peso (gradiente desde el centro)
            h, w = frame.shape[:2]
            y, x = np.ogrid[0:h, 0:w]
            center_y, center_x = h // 2, w // 2

            # Peso gaussiano desde el centro
            weight = np.exp(-((x - center_x) ** 2 + (y - center_y) ** 2) /
                            (2 * (w / 4) ** 2))
            weight = weight.astype(np.float32)

            # Warp frame y peso
            warped = cv2.warpPerspective(
                frame, H,
                (self.canvas_size[0], self.canvas_size[1])
            )
            weight_warped = cv2.warpPerspective(
                weight, H,
                (self.canvas_size[0], self.canvas_size[1])
            )

            # Acumular con pesos
            for c in range(3):
                panorama_float[:, :, c] += warped[:, :, c].astype(np.float32) * weight_warped
            weight_sum += weight_warped

        # Normalizar
        mask = weight_sum > 0
        for c in range(3):
            panorama_float[:, :, c][mask] /= weight_sum[mask]

        panorama = panorama_float.astype(np.uint8)

        return panorama

    def stitch_fast_preset(self, frames):
        """Método rápido optimizado (para tiempo real)"""
        return self.stitch(frames, mode='fast')

    def reset(self):
        """Reinicia el stitcher (para recalcular homografías)"""
        self.homographies = {}
        self.initialized = False
        print("✓ Panorama stitcher reiniciado")