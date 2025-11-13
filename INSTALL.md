# Guía de Instalación Detallada

Esta guía te llevará paso a paso por la instalación del sistema.

---

## Requisitos Previos

### Windows

1. **Python 3.8+**
   - Descargar de: https://www.python.org/downloads/
   - ⚠ Marcar "Add Python to PATH" durante instalación

2. **Microsoft Visual C++ Build Tools** (para compilar algunas librerías)
   - Descargar de: https://visualstudio.microsoft.com/visual-cpp-build-tools/

3. **Git** (opcional, para clonar el repositorio)
   - Descargar de: https://git-scm.com/downloads

### Linux (Ubuntu/Debian)
```bash
sudo apt update
sudo apt install python3.8 python3-pip python3-venv
sudo apt install python3-dev
sudo apt install libopencv-dev  # Opcional
```

### macOS
```bash
# Instalar Homebrew si no lo tienes
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Instalar Python
brew install python@3.8
```

---

## Instalación Paso a Paso

### 1. Obtener el Código

**Opción A: Descargar ZIP**
- Descargar el proyecto como ZIP
- Extraer en la ubicación deseada

**Opción B: Clonar con Git**
```bash
git clone https://github.com/tu-usuario/depth-estimation-project.git
cd depth-estimation-project
```

### 2. Crear Entorno Virtual

**Windows:**
```cmd
python -m venv venv
venv\Scripts\activate
```

**Linux/Mac:**
```bash
python3 -m venv venv
source venv/bin/activate
```

Deberías ver `(venv)` al inicio de tu línea de comandos.

### 3. Instalar Dependencias
```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### 4. Verificar Instalación
```bash
python -c "import cv2; print(cv2.__version__)"
```

Debería mostrar: `4.8.1` o similar.

### 5. Configurar Cámaras
```bash
python test_camera_ids.py
```

Anota los índices que aparecen.

### 6. Actualizar Configuración

Edita `src/camera_capture.py` línea ~20:
```python
self.camera_ids = {
    'front': 0,   # ← Tu índice aquí
    'right': 1,
    'back': 2,
    'left': 3
}
```

### 7. Prueba Inicial
```bash
python test_single_camera.py
```

Si ves tu cámara funcionando, ¡éxito! ✅

---

## Problemas Comunes

### Error: "No module named 'cv2'"

**Solución:**
```bash
pip uninstall opencv-python opencv-contrib-python
pip install opencv-contrib-python==4.8.1.78
```

### Error: "DLL load failed" (Windows)

**Solución:**
Instalar Visual C++ Redistributable:
https://aka.ms/vs/17/release/vc_redist.x64.exe

### Error: "Camera not found"

**Solución:**
- Verificar que la cámara esté conectada
- Probar con diferentes puertos USB
- Dar permisos de cámara (en configuración del sistema)

---

## Instalación en Jetson Nano
```bash
# Actualizar sistema
sudo apt update && sudo apt upgrade

# Instalar dependencias
sudo apt install python3-pip python3-dev
sudo apt install python3-opencv

# Instalar librerías Python
pip3 install numpy pyyaml

# Clonar proyecto
git clone https://github.com/tu-usuario/depth-estimation-project.git
cd depth-estimation-project

# Ejecutar
python3 main.py
```

---

## Desinstalación
```bash
# Desactivar entorno virtual
deactivate

# Eliminar entorno virtual
rm -rf venv  # Linux/Mac
rmdir /s venv  # Windows

# Eliminar proyecto
cd ..
rm -rf depth-estimation-project
```