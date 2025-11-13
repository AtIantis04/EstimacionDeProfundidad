# Guía de Usuario

Esta guía explica cómo usar el sistema paso a paso.

---

## Inicio Rápido

### Opción 1: Línea de Comandos (Avanzado)
```bash
python main.py
```

### Opción 2: Interfaz Gráfica (Recomendado)
```bash
python gui_main.py
```

---

## Usando la Interfaz Gráfica

### 1. Iniciar el Sistema

1. Ejecuta: `python gui_main.py`
2. Haz clic en **"▶ Iniciar Sistema"**
3. Espera a que se inicialice (barra de progreso)
4. El sistema comenzará a procesar automáticamente

### 2. Panel de Controles

#### Visualización
- ☑ **Mostrar Panorama**: Vista panorámica 360°
- ☑ **Mostrar Profundidad**: Mapa de profundidad en color
- ☑ **Mostrar Nube de Puntos**: Vista 3D desde arriba

#### Configuración
- **Modo de Procesamiento**:
  - *Rápido*: Mejor rendimiento (~25-30 FPS)
  - *Calidad*: Mejor precisión (~15-20 FPS)

- **Resolución**:
  - 640x480: Recomendado para tiempo real
  - 1280x720: Mayor calidad, menor FPS
  - 1920x1080: Máxima calidad, solo para PC potentes

#### Acciones
- **🔧 Calibrar Cámaras**: Abre instrucciones de calibración
- **💾 Guardar Frame**: Guarda la imagen actual
- **🔄 Reiniciar Panorama**: Recalcula homografías

### 3. Pestañas de Visualización

- **Vista Principal**: Panorama o vista principal
- **Cámaras**: Grid con todas las cámaras
- **Profundidad**: Mapa de profundidad detallado
- **Nube de Puntos**: Vista 3D (top-down)

### 4. Información en Tiempo Real

El panel muestra:
- **FPS**: Frames por segundo actual
- **Frames**: Número total de frames procesados
- **Cámaras**: Número de cámaras activas
- **Barra de Progreso**: Estado del sistema

### 5. Log de Eventos

El área de log muestra:
- Mensajes del sistema
- Advertencias
- Errores
- Confirmaciones de acciones

---

## Calibración

### Preparación

1. Descarga e imprime el [tablero de ajedrez](https://markhedleyjones.com/projects/calibration-checkerboard-collection)
2. Pégalo en una superficie rígida
3. Asegúrate de tener buena iluminación

### Proceso

1. Desde la GUI: Click en **"🔧 Calibrar Cámaras"**
2. O desde terminal: `python run_calibration_wizard.py`
3. Sigue las instrucciones en pantalla:
   - Calibra cada cámara (25 imágenes)
   - Calibra pares estéreo (20 pares)
   - Guarda la calibración

### Consejos

- Mueve el tablero lentamente
- Cubre todo el campo de visión
- Incluye diferentes ángulos y distancias
- Mantén el tablero completamente plano

---

## Solución de Problemas Comunes

### La GUI no inicia

**Error**: `ModuleNotFoundError: No module named 'PyQt5'`

**Solución**:
```bash
pip install PyQt5
```

### FPS muy bajos en GUI

**Problema**: La interfaz gráfica consume recursos adicionales.

**Soluciones**:
- Usar modo "Rápido"
- Reducir resolución a 640x480
- Desmarcar visualizaciones no necesarias
- Usar versión de línea de comandos (`main.py`)

### La imagen se ve distorsionada

**Problema**: Cámaras no calibradas correctamente.

**Solución**:
1. Ejecutar calibración nuevamente
2. Asegurar RMS error < 0.5 píxeles
3. Reiniciar el sistema

---

## Atajos de Teclado (versión línea de comandos)

| Tecla | Acción |
|-------|--------|
| P | Toggle panorama |
| D | Toggle depth |
| C | Toggle point cloud |
| F | Toggle fast mode |
| S | Save frame |
| R | Reset panorama |
| I | Show info |
| ESC | Exit |

---

## Exportar Resultados

### Guardar Frame Actual

**GUI**: Click en **"💾 Guardar Frame"**

**CLI**: Presiona `S`

Se guarda en: `output/frame_TIMESTAMP.jpg`

### Guardar Nube de Puntos

Durante ejecución, los resultados se guardan automáticamente en:
- `output/pointcloud_TIMESTAMP.ply`
- `output/depth_TIMESTAMP.npy`

### Abrir Nube de Puntos

Usa software de visualización 3D:
- **CloudCompare** (gratuito)
- **MeshLab** (gratuito)
- **Blender** (gratuito, avanzado)
```bash
# Ejemplo con CloudCompare
cloudcompare output/pointcloud_12345.ply
```

---

## Configuración Avanzada

Edita `config/system_config.yml`:
```yaml
# Ajusta estos valores según tu hardware
resolution: [640, 480]
target_fps: 20
max_points: 50000

# Filtros de profundidad (metros)
z_range: [0.5, 25]  # Min y max distancia
```

---

## Preguntas Frecuentes

**P: ¿Cuántas cámaras necesito?**
R: Mínimo 1 para pruebas, 2 para profundidad, 4 para panorama completo.

**P: ¿Qué cámaras son compatibles?**
R: Cualquier webcam USB (Logitech C920 recomendadas).

**P: ¿Funciona en tiempo real?**
R: Sí, 20-30 FPS en PC moderno con 4 cámaras.

**P: ¿Puedo usar en Raspberry Pi?**
R: Sí, pero con rendimiento reducido. Jetson Nano recomendado.

**P: ¿Necesito GPU?**
R: No es obligatorio, pero mejora el rendimiento.

---

## Soporte

Para problemas o preguntas:
- Revisa el archivo `README.md`
- Consulta `TROUBLESHOOTING.md`
- Abre un issue en GitHub