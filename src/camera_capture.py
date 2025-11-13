"""
Módulo de captura multi-cámara sincronizada
"""

import cv2
import threading
from collections import deque
import time


class CameraCapture:
    """Captura sincronizada de múltiples cámaras USB"""

    def __init__(self, n_cameras=4, resolution=(640, 480), fps=30):
        """
        Args:
            n_cameras: Número de cámaras
            resolution: Tupla (width, height)
            fps: FPS objetivo
        """
        self.n_cameras = n_cameras
        self.resolution = resolution
        self.target_fps = fps

        # Mapeo de ID de cámara a índice USB
        # IMPORTANTE: Ajustar indices en base a las camaras
        self.camera_ids = {
            'front': 0,
            #'right': 1,
            #'back': 2,
            #'left': 3
        }

        # Inicializar capturadores
        self.captures = {}
        self.threads = {}
        self.frame_buffers = {}
        self.locks = {}
        self.running = False

        self._initialize_cameras()

    def _initialize_cameras(self):
        """Inicializa todas las cámaras"""
        print("\nInicializando cámaras...")

        for cam_id, usb_id in self.camera_ids.items():
            cap = cv2.VideoCapture(usb_id)

            if not cap.isOpened():
                print(f"⚠ Warning: No se pudo abrir cámara {cam_id} (USB {usb_id})")
                continue

            # Configurar resolución y FPS
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            cap.set(cv2.CAP_PROP_FPS, self.target_fps)

            # Optimizaciones
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Buffer mínimo
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

            self.captures[cam_id] = cap
            self.frame_buffers[cam_id] = deque(maxlen=1)
            self.locks[cam_id] = threading.Lock()

            print(f"✓ Cámara {cam_id} inicializada (USB {usb_id})")

        if len(self.captures) == 0:
            raise RuntimeError("No se pudo inicializar ninguna cámara")

        print(f"✓ Total de cámaras activas: {len(self.captures)}/{self.n_cameras}")

    def _capture_thread(self, cam_id):
        """Thread de captura individual por cámara"""
        cap = self.captures[cam_id]

        while self.running:
            ret, frame = cap.read()

            if ret:
                with self.locks[cam_id]:
                    self.frame_buffers[cam_id].append({
                        'frame': frame,
                        'timestamp': time.time()
                    })
            else:
                print(f"⚠ Error capturando frame de {cam_id}")
                time.sleep(0.01)

    def start(self):
        """Inicia captura en todos los threads"""
        self.running = True

        for cam_id in self.captures.keys():
            thread = threading.Thread(
                target=self._capture_thread,
                args=(cam_id,),
                daemon=True
            )
            thread.start()
            self.threads[cam_id] = thread

        print("✓ Captura multi-cámara iniciada")

    def get_synchronized_frames(self, max_time_diff=0.1):
        """
        Obtiene set de frames sincronizados

        Args:
            max_time_diff: Diferencia máxima de tiempo aceptable (segundos)

        Returns:
            frames: Dict {cam_id: frame} o None si no hay frames nuevos
        """
        frames = {}
        timestamps = {}

        # Obtener último frame de cada cámara Y REMOVERLO del buffer
        for cam_id in self.captures.keys():
            with self.locks[cam_id]:
                if len(self.frame_buffers[cam_id]) > 0:
                    # POP: Remover el frame del buffer (esto es clave)
                    data = self.frame_buffers[cam_id].pop()
                    frames[cam_id] = data['frame'].copy()  # Copiar para evitar referencias
                    timestamps[cam_id] = data['timestamp']
                else:
                    # No hay frames nuevos en el buffer
                    import time
                    time.sleep(0.001)  # Esperar 1ms si no hay frames
                    return None

        # Verificar que tengamos todos los frames
        if len(frames) != len(self.captures):
            return None

        # Verificar sincronización temporal
        ts_values = list(timestamps.values())
        time_range = max(ts_values) - min(ts_values)

        if time_range > max_time_diff:
            # Solo mostrar warning ocasionalmente (no cada frame)
            pass  # Comentado para no saturar consola

        return frames

    def release(self):
        """Libera recursos de todas las cámaras"""
        self.running = False

        for thread in self.threads.values():
            thread.join(timeout=1.0)

        for cap in self.captures.values():
            cap.release()

        print("✓ Cámaras liberadas")