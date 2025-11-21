# Sistema de Estimación de Profundidad Multi-Cámara

Sistema completo de visión por computador para estimación de profundidad y reconstrucción 3D en tiempo real utilizando múltiples cámaras.

![Status](https://img.shields.io/badge/status-active-success.svg)
![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
![OpenCV](https://img.shields.io/badge/opencv-4.5+-green.svg)

---

## 📋 Tabla de Contenidos

- [Características](#características)
- [Requisitos](#requisitos)
- [Instalación](#instalación)
- [Uso Rápido](#uso-rápido)
- [Documentación Detallada](#documentación-detallada)
- [Estructura del Proyecto](#estructura-del-proyecto)
- [Contribuciones](#contribuciones)

---

## ✨ Características

- **Captura multi-cámara sincronizada** (hasta 4 cámaras USB)
- **Vista panorámica 360°** usando stitching automático
- **Estimación de profundidad estéreo** con algoritmo SGBM
- **Generación de nubes de puntos 3D** coloreadas
- **Calibración automatizada** con asistente interactivo
- **Procesamiento en tiempo real** (~20 FPS)
- **Interfaz interactiva** con controles en vivo
- **Modular y extensible** para agregar nuevas funcionalidades

---

## 🔧 Requisitos

### Hardware

- **Mínimo:**
  - 1-4 cámaras USB (Logitech C920 recomendadas)
  - CPU: Intel i5 o equivalente
  - RAM: 8GB
  - Puertos USB disponibles (uno por cámara)

- **Recomendado:**
  - CPU: Intel i7 o superior
  - RAM: 16GB
  - GPU: NVIDIA (para aceleración CUDA)
  - Jetson Nano/Orin (para deployment)

### Software

- Python 3.8 o superior
- OpenCV 4.5+ (con módulos contrib)
- NumPy
- PyYAML
- Open3D

---

## 📦 Instalación

### 1. Clonar el repositorio
```bash
git clone https://github.com/tu-usuario/depth-estimation-project.git
cd depth-estimation-project
```

### 2. Crear entorno virtual
```bash
python -m venv venv

# Windows
venv\Scripts\activate

# Linux/Mac
source venv/bin/activate
```

### 3. Instalar dependencias
```bash
pip install opencv-contrib-python==4.8.1.78
pip install numpy
pip install pyyaml
pip install open3d  
pip install numba   
```

### 4. Configurar cámaras

Identifica los índices USB de tus cámaras:
```bash
python test_camera_ids.py
```

Actualiza `src/camera_capture.py` con los índices correctos:
```python
self.camera_ids = {
    'front': 0,   
    'right': 1,   
    'back': 2,    
    'left': 3     
}
```

---

## 🚀 Uso Rápido

### Paso 1: Calibración

Antes de usar el sistema, debes calibrar las cámaras:
```bash
python run_calibration_wizard.py
```

Sigue las instrucciones en pantalla para:
1. Calibrar cada cámara individualmente (25 imágenes por cámara)
2. Calibrar pares estéreo
3. Guardar la calibración

**Requisito:** Tablero de ajedrez 9x6 (imprimible desde [aquí](https://markhedleyjones.com/projects/calibration-checkerboard-collection))

### Paso 2: Ejecutar el sistema
```bash
python main.py
```

### Controles en Tiempo Real

| Tecla | Acción |
|-------|--------|
| `P` | Toggle panorama |
| `D` | Toggle mapa de profundidad |
| `C` | Toggle nube de puntos |
| `F` | Toggle modo rápido/calidad |
| `S` | Guardar frame actual |
| `R` | Reiniciar panorama |
| `I` | Mostrar información del sistema |
| `ESC` | Salir |

---

## 📚 Documentación Detallada

### Scripts de Prueba

#### `test_camera_ids.py`
Detecta todas las cámaras conectadas y muestra sus índices.
```bash
python test_camera_ids.py
```

#### `test_single_camera.py`
Prueba básica de captura con una cámara.
```bash
python test_single_camera.py
```

#### `test_single_camera_stereo.py`
Simula visión estéreo con una sola cámara moviéndola entre dos posiciones.
```bash
python test_single_camera_stereo.py
```

---

### Configuración

Edita `config/system_config.yml` para personalizar el sistema:
```yaml
# Número de cámaras
n_cameras: 4

# Resolución (width, height)
resolution: [640, 480]

# FPS objetivo
fps: 30

# Par estéreo principal
stereo_pair: ["front", "right"]

# Límite de puntos en nube
max_points: 50000

# Rangos de filtrado (metros)
x_range: [-10, 10]
y_range: [-3, 3]
z_range: [0.5, 25]

# Modo rápido (true/false)
use_fast_mode: true
target_fps: 20
```

---

### Arquitectura del Sistema
```
┌─────────────────────────────────────────────────────────┐
│                   MAIN SYSTEM                           │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────┐    ┌──────────────┐                   │
│  │   Camera     │───▶│  Calibrator  │                  │
│  │   Capture    │    │              │                  │
│  └──────────────┘    └──────────────┘                  │
│         │                    │                          │
│         ▼                    ▼                          │
│  ┌──────────────┐    ┌──────────────┐                  │
│  │   Panorama   │    │    Depth     │                  │
│  │   Stitcher   │    │   Pipeline   │                  │
│  └──────────────┘    └──────────────┘                  │
│                             │                           │
│                             ▼                           │
│                      ┌──────────────┐                   │
│                      │  Point Cloud │                   │
│                      │   Builder    │                   │
│                      └──────────────┘                   │
│                             │                           │
│                             ▼                           │
│                      ┌──────────────┐                   │
│                      │  Visualizer  │                   │
│                      └──────────────┘                   │
└─────────────────────────────────────────────────────────┘
```

---

### Módulos Principales

#### `src/camera_capture.py`
Captura sincronizada de múltiples cámaras usando threading.

#### `src/calibrator.py`
Calibración de cámaras (intrínsecos y extrínsecos).

#### `src/panorama_stitcher.py`
Generación de vista panorámica 360°.

#### `src/stereo_matcher.py`
Cálculo de mapa de disparidad (SGBM).

#### `src/depth_estimator.py`
Conversión de disparidad a profundidad métrica.

#### `src/pointcloud_builder.py`
Generación de nubes de puntos 3D.

#### `src/visualizer.py`
Visualización 2D y 3D de resultados.

---

## 📁 Estructura del Proyecto
```
depth_estimation_project/
├── src/                        # Código fuente principal
│   ├── camera_capture.py       # Captura multi-cámara
│   ├── calibrator.py           # Calibración
│   ├── panorama_stitcher.py    # Stitching panorámico
│   ├── stereo_matcher.py       # Stereo matching
│   ├── depth_estimator.py      # Estimación de profundidad
│   ├── depth_pipeline.py       # Pipeline integrado
│   ├── pointcloud_builder.py   # Generación de nube de puntos
│   ├── visualizer.py           # Visualización
│   ├── stereo_rectifier.py     # Rectificación estéreo
│   └── utils.py                # Utilidades
├── config/                     # Archivos de configuración
│   ├── system_config.yml       # Configuración del sistema
│   └── camera_params.yml       # Parámetros de calibración
├── calibration_data/           # Datos de calibración
│   ├── front/                  # Imágenes cámara frontal
│   ├── right/                  # Imágenes cámara derecha
│   ├── back/                   # Imágenes cámara trasera
│   ├── left/                   # Imágenes cámara izquierda
│   └── stereo/                 # Pares estéreo
├── output/                     # Resultados guardados
│   ├── pointcloud_*.ply        # Nubes de puntos
│   └── depth_*.npy             # Mapas de profundidad
├── test_camera_ids.py          # Test de cámaras
├── test_single_camera.py       # Test cámara única
├── test_single_camera_stereo.py # Test estéreo simulado
├── run_calibration_wizard.py   # Asistente de calibración
├── main.py                     # Sistema principal
├── README.md                   # Este archivo
└── requirements.txt            # Dependencias Python
```

---

## 🎯 Casos de Uso

### 1. Vehículos Autónomos
- Detección de obstáculos
- Estimación de distancias
- Mapeo del entorno

### 2. Robótica
- Navegación autónoma
- Manipulación de objetos
- SLAM (localización y mapeo)

### 3. Realidad Aumentada
- Reconstrucción 3D de escenas
- Oclusión realista
- Interacción con el entorno

### 4. Vigilancia y Seguridad
- Tracking 3D de personas
- Análisis de comportamiento
- Detección de intrusiones

---

## 🐛 Solución de Problemas

### FPS bajos

**Problema:** El sistema corre a menos de 10 FPS.

**Soluciones:**
- Reducir resolución en `config/system_config.yml`
- Activar modo rápido: `python main.py --fast`
- Aumentar `pc_downsample` a 3 o 4
- Cerrar otras aplicaciones

### Mapa de disparidad con huecos

**Problema:** Muchos píxeles negros en el mapa de disparidad.

**Soluciones:**
- Mejorar iluminación de la escena
- Agregar objetos con textura
- Ajustar parámetros SGBM en `src/stereo_matcher.py`
- Verificar calibración estéreo

### Calibración falla

**Problema:** No se detecta el tablero de ajedrez.

**Soluciones:**
- Verificar que el patrón es 9x6 (esquinas internas)
- Mejorar iluminación
- Mantener tablero plano (sin dobleces)
- Limpiar lente de la cámara

### Cámara no detectada

**Problema:** `test_camera_ids.py` no encuentra todas las cámaras.

**Soluciones:**
- Conectar a diferentes puertos USB
- Probar con USB 2.0 en lugar de 3.0
- Reiniciar el sistema
- Verificar drivers de cámara

---

## 📊 Rendimiento

### Benchmarks (PC - Intel i7, 16GB RAM)

| Configuración | FPS | Latencia | Calidad |
|---------------|-----|----------|---------|
| 1 cámara, 640x480, Fast | 30 | 33ms | Buena |
| 2 cámaras, 640x480, Fast | 28 | 36ms | Buena |
| 4 cámaras, 640x480, Fast | 22 | 45ms | Buena |
| 4 cámaras, 640x480, Quality | 12 | 83ms | Excelente |
| 4 cámaras, 1280x720, Fast | 15 | 67ms | Excelente |

---

## 🔮 Trabajo Futuro

- [ ] Migración a C++/CUDA para Jetson
- [ ] Integración con redes de profundidad monocular (MiDaS)
- [ ] SLAM en tiempo real
- [ ] Interfaz gráfica (GUI)
- [ ] API REST para integración
- [ ] Soporte para más tipos de cámaras
- [ ] Calibración automática sin tablero

---

## 🤝 Contribuciones

Las contribuciones son bienvenidas. Por favor:

1. Fork el proyecto
2. Crea una rama para tu feature (`git checkout -b feature/AmazingFeature`)
3. Commit tus cambios (`git commit -m 'Add some AmazingFeature'`)
4. Push a la rama (`git push origin feature/AmazingFeature`)
5. Abre un Pull Request

## 👤 Autor

**Tu Nombre**
- Tesis de Maestría en Ciencia de Datos
- Universidad: Instituto Tecnologico de Queretaro
- Email: m24140002@queretaro.tecnm.mx

---

## 🙏 Agradecimientos

- OpenCV Team por las herramientas de visión por computador
- Hartley & Zisserman por "Multiple View Geometry in Computer Vision"
- Comunidad de Python y Open Source

---

**Última actualización:**  2025
