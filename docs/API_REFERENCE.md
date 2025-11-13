# API Reference

Documentación de la API interna para desarrolladores.

---

## Módulos Principales

### `CameraCapture`
```python
from src.camera_capture import CameraCapture

# Inicializar
capture = CameraCapture(n_cameras=4, resolution=(640, 480), fps=30)

# Iniciar captura
capture.start()

# Obtener frames sincronizados
frames = capture.get_synchronized_frames()
# Returns: dict {'front': frame, 'right': frame, ...} o None

# Liberar recursos
capture.release()
```

### `Calibrator`
```python
from src.calibrator import Calibrator

# Cargar calibración existente
calibrator = Calibrator(config_path='config/camera_params.yml')

# Calibrar cámara individual
result = calibrator.calibrate_camera(
    images,                    # Lista de imágenes
    pattern_size=(9, 6),       # Esquinas internas del tablero
    square_size=0.025          # Tamaño del cuadrado en metros
)

# Calibrar par estéreo
stereo_result = calibrator.calibrate_stereo_pair(
    images_left, images_right,
    K_left, dist_left, K_right, dist_right
)

# Guardar calibración
calibrator.save_calibration('config/camera_params.yml')

# Aplicar undistortion
undistorted = calibrator.undistort_frame(frame, 'front')
```

### `DepthPipeline`
```python
from src.depth_pipeline import DepthPipeline

# Inicializar
pipeline = DepthPipeline(calibration_data, camera_pair=('front', 'right'))

# Procesar par estéreo
depth_map, disparity, rect_left, rect_right = pipeline.process(
    img_left, img_right, fast_mode=False
)

# Modo rápido
depth_map, disparity, rect_left, rect_right = pipeline.process_fast(
    img_left, img_right
)
```

### `PointCloudBuilder`
```python
from src.pointcloud_builder import PointCloudBuilder

# Inicializar
pc_builder = PointCloudBuilder(camera_matrix, max_points=100000)

# Generar nube de puntos
pointcloud = pc_builder.build(
    depth_map,
    color_image=frame,
    downsample=2
)

# Filtrar
filtered = pc_builder.filter_pointcloud(
    pointcloud,
    x_range=(-10, 10),
    y_range=(-5, 5),
    z_range=(0, 30)
)

# Guardar
pc_builder.save_ply(pointcloud, 'output/cloud.ply')
```

---

## Estructuras de Datos

### Frame Dictionary
```python
frames = {
    'front': np.ndarray,  # BGR image (H, W, 3)
    'right': np.ndarray,
    'back': np.ndarray,
    'left': np.ndarray
}
```

### Point Cloud Dictionary
```python
pointcloud = {
    'points': np.ndarray,  # (N, 3) float32 - coordenadas XYZ
    'colors': np.ndarray   # (N, 3) float32 - colores RGB [0-1]
}
```

### Calibration Data Dictionary
```python
calibration_data = {
    'K_front': np.ndarray,       # (3, 3) matriz de cámara
    'dist_front': np.ndarray,    # (5,) coeficientes de distorsión
    'R_front_right': np.ndarray, # (3, 3) rotación
    'T_front_right': np.ndarray  # (3, 1) traslación
}
```

---

## Extender el Sistema

### Agregar Nuevo Algoritmo de Stereo
```python
# src/my_stereo_matcher.py
class MyStereoMatcher:
    def compute_disparity(self, img_left, img_right):
        # Tu implementación aquí
        return disparity
```

### Agregar Nueva Visualización
```python
# src/my_visualizer.py
class MyVisualizer:
    def visualize(self, pointcloud):
        # Tu implementación aquí
        pass
```

---

## Eventos y Callbacks
```python
# Ejemplo de procesamiento custom
def my_processing_callback(frames, depth_map, pointcloud):
    # Tu código aquí
    print(f"Procesado frame con {len(pointcloud['points'])} puntos")

# Integrar en el sistema
system.add_callback(my_processing_callback)
```

---

## Configuración Programática
```python
# Crear configuración custom
config = {
    'n_cameras': 2,
    'resolution': [1280, 720],
    'stereo_pair': ['cam0', 'cam1'],
    'use_fast_mode': True
}

# Inicializar con configuración
system = MultiCameraDepthSystem(config=config)
```