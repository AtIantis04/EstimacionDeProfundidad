"""
Módulo de calibración de cámaras
"""

import cv2
import numpy as np
import os


class Calibrator:
    """Manejo de calibración de cámaras"""

    def __init__(self, config_path=None):
        """
        Args:
            config_path: Ruta al archivo de calibración (opcional)
        """
        self.camera_matrices = {}  # K para cada cámara
        self.dist_coeffs = {}  # Coeficientes de distorsión
        self.extrinsics = {}  # Transformaciones entre cámaras

        if config_path and os.path.exists(config_path):
            self.load_calibration(config_path)
            print(f"✓ Calibración cargada desde: {config_path}")
        else:
            print("⚠ No se encontró archivo de calibración")

    def calibrate_camera(self, images, pattern_size=(9, 6), square_size=0.025):
        """
        Calibra una cámara usando patrón de tablero de ajedrez

        Args:
            images: Lista de imágenes del patrón (numpy arrays BGR)
            pattern_size: (cols-1, rows-1) esquinas internas del tablero
            square_size: Tamaño del cuadrado en metros (ej: 0.025 = 25mm)

        Returns:
            dict con 'camera_matrix', 'dist_coeffs', 'rms_error', etc.
        """
        print(f"\nCalibrando cámara con {len(images)} imágenes...")

        # Preparar puntos del objeto (mundo 3D)
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0],
                      0:pattern_size[1]].T.reshape(-1, 2)
        objp *= square_size

        # Arrays para almacenar puntos
        objpoints = []  # Puntos 3D en el mundo real
        imgpoints = []  # Puntos 2D en la imagen

        img_shape = None
        images_used = 0

        for i, img in enumerate(images):
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_shape = gray.shape[::-1]  # (width, height)

            # Encontrar esquinas del tablero
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

            if ret:
                objpoints.append(objp)

                # Refinar posición de esquinas (sub-pixel accuracy)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                            30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11),
                                            (-1, -1), criteria)
                imgpoints.append(corners2)
                images_used += 1
                print(f"  ✓ Imagen {i + 1}/{len(images)}: Patrón detectado")
            else:
                print(f"  ✗ Imagen {i + 1}/{len(images)}: Patrón NO detectado")

        if images_used < 10:
            raise ValueError(f"Solo se detectó el patrón en {images_used} imágenes. "
                             f"Se necesitan al menos 10.")

        print(f"\n  Calibrando con {images_used} imágenes válidas...")

        # Calibración
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, img_shape, None, None
        )

        # Calcular error de reproyección
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i],
                                              tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error

        mean_error = mean_error / len(objpoints)

        print(f"  ✓ Calibración completada")
        print(f"  RMS Error: {ret:.4f} píxeles")
        print(f"  Error medio de reproyección: {mean_error:.4f} píxeles")
        print(f"  Focal length (fx): {mtx[0, 0]:.2f} px")
        print(f"  Focal length (fy): {mtx[1, 1]:.2f} px")

        return {
            'camera_matrix': mtx,
            'dist_coeffs': dist,
            'rvecs': rvecs,
            'tvecs': tvecs,
            'rms_error': ret,
            'mean_reproj_error': mean_error,
            'images_used': images_used
        }

    def calibrate_stereo_pair(self, images_left, images_right,
                              K_left, dist_left, K_right, dist_right,
                              pattern_size=(9, 6), square_size=0.025):
        """
        Calibración estéreo entre dos cámaras

        Args:
            images_left: Lista de imágenes de cámara izquierda
            images_right: Lista de imágenes de cámara derecha (capturadas simultáneamente)
            K_left, K_right: Matrices de cámara (3x3)
            dist_left, dist_right: Coeficientes de distorsión
            pattern_size: Tamaño del patrón
            square_size: Tamaño del cuadrado en metros

        Returns:
            dict con 'R', 'T', 'E', 'F', 'rms_error'
        """
        print(f"\nCalibrando par estéreo con {len(images_left)} pares de imágenes...")

        # Preparar puntos del objeto
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0],
                      0:pattern_size[1]].T.reshape(-1, 2)
        objp *= square_size

        objpoints = []
        imgpoints_left = []
        imgpoints_right = []

        pairs_used = 0

        # Encontrar correspondencias en ambas cámaras
        for i, (img_l, img_r) in enumerate(zip(images_left, images_right)):
            gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

            ret_l, corners_l = cv2.findChessboardCorners(gray_l, pattern_size)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, pattern_size)

            if ret_l and ret_r:
                objpoints.append(objp)

                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                            30, 0.001)
                corners_l = cv2.cornerSubPix(gray_l, corners_l, (11, 11),
                                             (-1, -1), criteria)
                corners_r = cv2.cornerSubPix(gray_r, corners_r, (11, 11),
                                             (-1, -1), criteria)

                imgpoints_left.append(corners_l)
                imgpoints_right.append(corners_r)
                pairs_used += 1
                print(f"  ✓ Par {i + 1}: Patrón detectado en ambas cámaras")
            else:
                status = []
                if not ret_l:
                    status.append("izquierda")
                if not ret_r:
                    status.append("derecha")
                print(f"  ✗ Par {i + 1}: Patrón NO detectado en {', '.join(status)}")

        if pairs_used < 10:
            raise ValueError(f"Solo {pairs_used} pares válidos. Se necesitan al menos 10.")

        print(f"\n  Calibrando con {pairs_used} pares válidos...")

        img_shape = gray_l.shape[::-1]

        # Calibración estéreo
        ret, K_l, dist_l, K_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
            objpoints, imgpoints_left, imgpoints_right,
            K_left, dist_left, K_right, dist_right,
            img_shape,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                      100, 1e-5),
            flags=cv2.CALIB_FIX_INTRINSIC
        )

        baseline = np.linalg.norm(T)

        print(f"  ✓ Calibración estéreo completada")
        print(f"  RMS Error: {ret:.4f}")
        print(f"  Baseline: {baseline:.4f} metros ({baseline * 100:.2f} cm)")
        print(f"  Traslación T: [{T[0][0]:.4f}, {T[1][0]:.4f}, {T[2][0]:.4f}]")

        return {
            'R': R,
            'T': T,
            'E': E,
            'F': F,
            'rms_error': ret,
            'baseline': baseline
        }

    def save_calibration(self, filepath='config/camera_params.yml'):
        """Guarda parámetros de calibración en archivo YAML (OpenCV)"""

        print(f"\nGuardando calibración en: {filepath}")

        # Crear directorio si no existe
        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_WRITE)

        # Guardar intrínsecos
        for cam_id in self.camera_matrices.keys():
            fs.write(f'K_{cam_id}', self.camera_matrices[cam_id])
            fs.write(f'dist_{cam_id}', self.dist_coeffs[cam_id])
            print(f"  ✓ Guardados parámetros de cámara: {cam_id}")

        # Guardar extrínsecos
        for pair_id, extrinsic in self.extrinsics.items():
            fs.write(f'R_{pair_id}', extrinsic['R'])
            fs.write(f'T_{pair_id}', extrinsic['T'])
            if 'E' in extrinsic:
                fs.write(f'E_{pair_id}', extrinsic['E'])
            if 'F' in extrinsic:
                fs.write(f'F_{pair_id}', extrinsic['F'])
            print(f"  ✓ Guardados parámetros estéreo: {pair_id}")

        fs.release()
        print(f"✓ Calibración guardada exitosamente\n")

    def load_calibration(self, filepath='config/camera_params.yml'):
        """Carga parámetros desde archivo"""

        if not os.path.exists(filepath):
            print(f"⚠ Archivo no encontrado: {filepath}")
            return False

        fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)

        # Intentar cargar para cada posible cámara
        for cam_id in ['front', 'back', 'left', 'right']:
            K = fs.getNode(f'K_{cam_id}').mat()
            dist = fs.getNode(f'dist_{cam_id}').mat()

            if K is not None and dist is not None:
                self.camera_matrices[cam_id] = K
                self.dist_coeffs[cam_id] = dist

        # Intentar cargar extrínsecos
        stereo_pairs = ['front_right', 'front_left', 'back_right', 'back_left']
        for pair_id in stereo_pairs:
            R = fs.getNode(f'R_{pair_id}').mat()
            T = fs.getNode(f'T_{pair_id}').mat()

            if R is not None and T is not None:
                self.extrinsics[pair_id] = {
                    'R': R,
                    'T': T
                }

                E = fs.getNode(f'E_{pair_id}').mat()
                F = fs.getNode(f'F_{pair_id}').mat()
                if E is not None:
                    self.extrinsics[pair_id]['E'] = E
                if F is not None:
                    self.extrinsics[pair_id]['F'] = F

        fs.release()
        return True

    def undistort_frame(self, frame, cam_id):
        """
        Aplica corrección de distorsión a un frame

        Args:
            frame: Imagen a corregir
            cam_id: ID de la cámara ('front', 'right', etc.)

        Returns:
            Frame sin distorsión
        """
        if cam_id not in self.camera_matrices:
            print(f"⚠ No hay calibración para {cam_id}")
            return frame

        K = self.camera_matrices[cam_id]
        dist = self.dist_coeffs[cam_id]

        h, w = frame.shape[:2]
        new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))

        undistorted = cv2.undistort(frame, K, dist, None, new_K)

        return undistorted

    def undistort_frames(self, frames):
        """
        Aplica corrección de distorsión a todos los frames

        Args:
            frames: Dict {cam_id: frame}

        Returns:
            Dict {cam_id: undistorted_frame}
        """
        undistorted = {}

        for cam_id, frame in frames.items():
            undistorted[cam_id] = self.undistort_frame(frame, cam_id)

        return undistorted

    def get_calibration_dict(self):
        """Retorna diccionario con todos los parámetros de calibración"""
        calib_data = {}

        # Agregar intrínsecos
        for cam_id in self.camera_matrices.keys():
            calib_data[f'K_{cam_id}'] = self.camera_matrices[cam_id]
            calib_data[f'dist_{cam_id}'] = self.dist_coeffs[cam_id]

        # Agregar extrínsecos
        for pair_id, extrinsic in self.extrinsics.items():
            calib_data[f'R_{pair_id}'] = extrinsic['R']
            calib_data[f'T_{pair_id}'] = extrinsic['T']

        return calib_data