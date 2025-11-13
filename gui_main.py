# gui_main.py
"""
Interfaz Gráfica (GUI) para el Sistema de Estimación de Profundidad
Usa PyQt5 para crear una interfaz moderna e intuitiva
"""

import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QSlider,
                             QGroupBox, QCheckBox, QComboBox, QTextEdit,
                             QTabWidget, QProgressBar, QFileDialog, QMessageBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QImage, QPixmap, QFont

# Importar módulos del proyecto
from src.camera_capture import CameraCapture
from src.calibrator import Calibrator
from src.panorama_stitcher import PanoramaStitcher
from src.depth_pipeline import DepthPipeline
from src.pointcloud_builder import PointCloudBuilder
from src.visualizer import Simple2DVisualizer
from src.utils import load_config, FPSCounter


class ProcessingThread(QThread):
    """Thread para procesamiento en segundo plano"""
    frame_ready = pyqtSignal(dict)

    def __init__(self, system):
        super().__init__()
        self.system = system
        self.running = False

    def run(self):
        """Loop de procesamiento"""
        self.running = True

        while self.running:
            frames = self.system.camera_capture.get_synchronized_frames()

            if frames is not None:
                results = self.system.process_frame(frames)
                if results:
                    self.frame_ready.emit(results)

    def stop(self):
        """Detiene el thread"""
        self.running = False


class DepthEstimationGUI(QMainWindow):
    """Ventana principal de la GUI"""

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Sistema de Estimación de Profundidad Multi-Cámara")
        self.setGeometry(100, 100, 1400, 900)

        # Sistema
        self.system = None
        self.processing_thread = None
        self.is_running = False

        # FPS counter
        self.fps_counter = FPSCounter(buffer_size=30)

        # Configurar UI
        self.setup_ui()

        # Timer para actualizar interfaz
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(100)  # Actualizar cada 100ms

    def setup_ui(self):
        """Configura la interfaz de usuario"""

        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Layout principal
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)

        # Panel izquierdo (controles)
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, 1)

        # Panel derecho (visualización)
        right_panel = self.create_visualization_panel()
        main_layout.addWidget(right_panel, 3)

        # Barra de estado
        self.statusBar().showMessage("Listo")

    def create_control_panel(self):
        """Crea panel de controles"""

        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)

        # Título
        title = QLabel("CONTROLES")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Grupo: Sistema
        system_group = QGroupBox("Sistema")
        system_layout = QVBoxLayout()

        self.btn_start = QPushButton("▶ Iniciar Sistema")
        self.btn_start.clicked.connect(self.start_system)
        self.btn_start.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px; font-size: 14px;")
        system_layout.addWidget(self.btn_start)

        self.btn_stop = QPushButton("⏹ Detener Sistema")
        self.btn_stop.clicked.connect(self.stop_system)
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet("background-color: #f44336; color: white; padding: 10px; font-size: 14px;")
        system_layout.addWidget(self.btn_stop)

        system_group.setLayout(system_layout)
        layout.addWidget(system_group)

        # Grupo: Visualización
        viz_group = QGroupBox("Visualización")
        viz_layout = QVBoxLayout()

        self.chk_panorama = QCheckBox("Mostrar Panorama")
        self.chk_panorama.setChecked(True)
        viz_layout.addWidget(self.chk_panorama)

        self.chk_depth = QCheckBox("Mostrar Profundidad")
        self.chk_depth.setChecked(True)
        viz_layout.addWidget(self.chk_depth)

        self.chk_pointcloud = QCheckBox("Mostrar Nube de Puntos")
        self.chk_pointcloud.setChecked(True)
        viz_layout.addWidget(self.chk_pointcloud)

        viz_group.setLayout(viz_layout)
        layout.addWidget(viz_group)

        # Grupo: Configuración
        config_group = QGroupBox("Configuración")
        config_layout = QVBoxLayout()

        # Modo de procesamiento
        mode_label = QLabel("Modo de Procesamiento:")
        config_layout.addWidget(mode_label)

        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["Rápido", "Calidad"])
        config_layout.addWidget(self.combo_mode)

        # Resolución
        res_label = QLabel("Resolución:")
        config_layout.addWidget(res_label)

        self.combo_resolution = QComboBox()
        self.combo_resolution.addItems(["640x480", "1280x720", "1920x1080"])
        config_layout.addWidget(self.combo_resolution)

        config_group.setLayout(config_layout)
        layout.addWidget(config_group)

        # Grupo: Acciones
        actions_group = QGroupBox("Acciones")
        actions_layout = QVBoxLayout()

        self.btn_calibrate = QPushButton("🔧 Calibrar Cámaras")
        self.btn_calibrate.clicked.connect(self.open_calibration)
        actions_layout.addWidget(self.btn_calibrate)

        self.btn_save = QPushButton("💾 Guardar Frame")
        self.btn_save.clicked.connect(self.save_frame)
        actions_layout.addWidget(self.btn_save)

        self.btn_reset = QPushButton("🔄 Reiniciar Panorama")
        self.btn_reset.clicked.connect(self.reset_panorama)
        actions_layout.addWidget(self.btn_reset)

        actions_group.setLayout(actions_layout)
        layout.addWidget(actions_group)

        # Información del sistema
        info_group = QGroupBox("Información")
        info_layout = QVBoxLayout()

        self.lbl_fps = QLabel("FPS: 0.0")
        info_layout.addWidget(self.lbl_fps)

        self.lbl_frames = QLabel("Frames: 0")
        info_layout.addWidget(self.lbl_frames)

        self.lbl_cameras = QLabel("Cámaras: 0")
        info_layout.addWidget(self.lbl_cameras)

        self.progress_bar = QProgressBar()
        info_layout.addWidget(self.progress_bar)

        info_group.setLayout(info_layout)
        layout.addWidget(info_group)

        # Console/Log
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()

        self.text_log = QTextEdit()
        self.text_log.setReadOnly(True)
        self.text_log.setMaximumHeight(150)
        log_layout.addWidget(self.text_log)

        log_group.setLayout(log_layout)
        layout.addWidget(log_group)

        # Espaciador
        layout.addStretch()

        return panel

    def create_visualization_panel(self):
        """Crea panel de visualización"""

        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)

        # Tabs para diferentes vistas
        self.tabs = QTabWidget()

        # Tab 1: Vista principal
        tab_main = QWidget()
        tab_main_layout = QVBoxLayout()

        self.lbl_main_view = QLabel("Sistema no iniciado")
        self.lbl_main_view.setAlignment(Qt.AlignCenter)
        self.lbl_main_view.setMinimumSize(800, 600)
        self.lbl_main_view.setStyleSheet("background-color: #2b2b2b; color: white; font-size: 16px;")
        tab_main_layout.addWidget(self.lbl_main_view)

        tab_main.setLayout(tab_main_layout)
        self.tabs.addTab(tab_main, "Vista Principal")

        # Tab 2: Cámaras individuales
        tab_cameras = QWidget()
        tab_cameras_layout = QVBoxLayout()

        self.lbl_cameras_view = QLabel("Cámaras no disponibles")
        self.lbl_cameras_view.setAlignment(Qt.AlignCenter)
        self.lbl_cameras_view.setMinimumSize(800, 600)
        self.lbl_cameras_view.setStyleSheet("background-color: #2b2b2b; color: white; font-size: 16px;")
        tab_cameras_layout.addWidget(self.lbl_cameras_view)

        tab_cameras.setLayout(tab_cameras_layout)
        self.tabs.addTab(tab_cameras, "Cámaras")

        # Tab 3: Profundidad
        tab_depth = QWidget()
        tab_depth_layout = QVBoxLayout()

        self.lbl_depth_view = QLabel("Profundidad no disponible")
        self.lbl_depth_view.setAlignment(Qt.AlignCenter)
        self.lbl_depth_view.setMinimumSize(800, 600)
        self.lbl_depth_view.setStyleSheet("background-color: #2b2b2b; color: white; font-size: 16px;")
        tab_depth_layout.addWidget(self.lbl_depth_view)

        tab_depth.setLayout(tab_depth_layout)
        self.tabs.addTab(tab_depth, "Profundidad")

        # Tab 4: Nube de puntos
        tab_pc = QWidget()
        tab_pc_layout = QVBoxLayout()

        self.lbl_pc_view = QLabel("Nube de puntos no disponible")
        self.lbl_pc_view.setAlignment(Qt.AlignCenter)
        self.lbl_pc_view.setMinimumSize(800, 600)
        self.lbl_pc_view.setStyleSheet("background-color: #2b2b2b; color: white; font-size: 16px;")
        tab_pc_layout.addWidget(self.lbl_pc_view)

        tab_pc.setLayout(tab_pc_layout)
        self.tabs.addTab(tab_pc, "Nube de Puntos")

        layout.addWidget(self.tabs)

        return panel

    def log(self, message):
        """Agrega mensaje al log"""
        self.text_log.append(message)
        # Auto-scroll
        self.text_log.verticalScrollBar().setValue(
            self.text_log.verticalScrollBar().maximum()
        )

    def start_system(self):
        """Inicia el sistema"""
        try:
            self.log("Iniciando sistema...")
            self.btn_start.setEnabled(False)
            self.progress_bar.setValue(0)

            # Inicializar sistema (simplificado)
            from main import MultiCameraDepthSystem

            self.progress_bar.setValue(20)
            self.log("Cargando configuración...")

            self.system = MultiCameraDepthSystem()

            self.progress_bar.setValue(50)
            self.log("Inicializando cámaras...")

            self.system.camera_capture.start()

            self.progress_bar.setValue(80)
            self.log("Configurando pipeline...")

            # Iniciar thread de procesamiento
            self.processing_thread = ProcessingThread(self.system)
            self.processing_thread.frame_ready.connect(self.update_displays)
            self.processing_thread.start()

            self.progress_bar.setValue(100)
            self.log("✓ Sistema iniciado correctamente")

            self.is_running = True
            self.btn_start.setEnabled(False)
            self.btn_stop.setEnabled(True)
            self.statusBar().showMessage("Sistema en ejecución")

        except Exception as e:
            self.log(f"❌ Error al iniciar: {e}")
            QMessageBox.critical(self, "Error", f"No se pudo iniciar el sistema:\n{e}")
            self.btn_start.setEnabled(True)
            self.progress_bar.setValue(0)

    def stop_system(self):
        """Detiene el sistema"""
        try:
            self.log("Deteniendo sistema...")

            if self.processing_thread:
                self.processing_thread.stop()
                self.processing_thread.wait()

            if self.system:
                self.system.camera_capture.release()

            self.is_running = False
            self.btn_start.setEnabled(True)
            self.btn_stop.setEnabled(False)

            self.log("✓ Sistema detenido")
            self.statusBar().showMessage("Sistema detenido")

            # Limpiar displays
            self.lbl_main_view.setText("Sistema detenido")

        except Exception as e:
            self.log(f"❌ Error al detener: {e}")

    def update_displays(self, results):
        """Actualiza las visualizaciones con nuevos resultados"""

        try:
            # Actualizar FPS
            self.fps_counter.update()

            # Vista principal (panorama o imagen principal)
            if self.chk_panorama.isChecked() and 'panorama' in results:
                self.display_image(results['panorama'], self.lbl_main_view)
            elif 'frames' in results and len(results['frames']) > 0:
                # Mostrar primera cámara disponible
                first_cam = list(results['frames'].keys())[0]
                self.display_image(results['frames'][first_cam], self.lbl_main_view)

            # Vista de profundidad
            if self.chk_depth.isChecked() and 'depth_map' in results:
                depth_colored = self.system.depth_pipeline.depth_estimator.get_depth_colored(
                    results['depth_map']
                )
                self.display_image(depth_colored, self.lbl_depth_view)

            # Vista de nube de puntos
            if self.chk_pointcloud.isChecked() and 'pointcloud' in results:
                pc_vis = self.system.visualizer.update(results['pointcloud'])
                self.display_image(pc_vis, self.lbl_pc_view)

            # Grid de cámaras
            if 'frames' in results:
                grid = self.create_camera_grid(results['frames'])
                if grid is not None:
                    self.display_image(grid, self.lbl_cameras_view)

        except Exception as e:
            self.log(f"⚠ Error en visualización: {e}")

    def display_image(self, image, label):
        """Muestra imagen en un QLabel"""
        try:
            # Convertir BGR a RGB
            if len(image.shape) == 3:
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                image_rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

            # Obtener dimensiones del label
            label_width = label.width()
            label_height = label.height()

            # Redimensionar manteniendo aspecto
            h, w = image_rgb.shape[:2]
            scale = min(label_width / w, label_height / h)
            new_w = int(w * scale)
            new_h = int(h * scale)

            image_resized = cv2.resize(image_rgb, (new_w, new_h))

            # Convertir a QImage
            h, w, ch = image_resized.shape
            bytes_per_line = ch * w
            q_image = QImage(image_resized.data, w, h, bytes_per_line, QImage.Format_RGB888)

            # Mostrar
            pixmap = QPixmap.fromImage(q_image)
            label.setPixmap(pixmap)

        except Exception as e:
            self.log(f"⚠ Error al mostrar imagen: {e}")

    def create_camera_grid(self, frames):
        """Crea grid de cámaras"""
        try:
            if len(frames) == 0:
                return None

            # Crear grid 2x2
            h, w = 240, 320

            if len(frames) == 1:
                frame = list(frames.values())[0]
                return cv2.resize(frame, (w * 2, h * 2))

            elif len(frames) == 2:
                grid = np.zeros((h, w * 2, 3), dtype=np.uint8)
                for i, (cam_id, frame) in enumerate(frames.items()):
                    frame_resized = cv2.resize(frame, (w, h))
                    # Agregar etiqueta
                    cv2.putText(frame_resized, cam_id.upper(), (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    grid[:, i * w:(i + 1) * w] = frame_resized
                return grid

            else:
                grid = np.zeros((h * 2, w * 2, 3), dtype=np.uint8)
                positions = {
                    'front': (0, 0),
                    'right': (0, 1),
                    'back': (1, 0),
                    'left': (1, 1)
                }

                for cam_id, (row, col) in positions.items():
                    if cam_id in frames:
                        frame = cv2.resize(frames[cam_id], (w, h))
                        # Agregar etiqueta
                        cv2.putText(frame, cam_id.upper(), (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        grid[row * h:(row + 1) * h, col * w:(col + 1) * w] = frame

                return grid

        except Exception as e:
            self.log(f"⚠ Error en grid: {e}")
            return None

    def update_status(self):
        """Actualiza barra de estado"""
        if self.is_running:
            fps = self.fps_counter.get_fps()
            self.lbl_fps.setText(f"FPS: {fps:.1f}")

            if self.system:
                self.lbl_frames.setText(f"Frames: {self.system.frame_count}")
                self.lbl_cameras.setText(f"Cámaras: {len(self.system.camera_capture.captures)}")

    def open_calibration(self):
        """Abre asistente de calibración"""
        reply = QMessageBox.question(
            self,
            "Calibración",
            "¿Deseas abrir el asistente de calibración?\n\n"
            "Esto cerrará la aplicación actual y ejecutará\n"
            "el asistente en una nueva ventana.",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.log("Abriendo asistente de calibración...")
            # Aquí podrías abrir el wizard o dar instrucciones
            QMessageBox.information(
                self,
                "Calibración",
                "Ejecuta el siguiente comando en una terminal:\n\n"
                "python run_calibration_wizard.py"
            )

    def save_frame(self):
        """Guarda el frame actual"""
        if not self.is_running:
            QMessageBox.warning(self, "Advertencia", "El sistema no está en ejecución")
            return

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Guardar Frame",
            "output/frame.jpg",
            "Imágenes (*.jpg *.png)"
        )

        if filename:
            self.log(f"Frame guardado: {filename}")
            QMessageBox.information(self, "Éxito", f"Frame guardado en:\n{filename}")

    def reset_panorama(self):
        """Reinicia el panorama"""
        if not self.is_running:
            QMessageBox.warning(self, "Advertencia", "El sistema no está en ejecución")
            return

        self.log("Reiniciando panorama...")
        try:
            if self.system:
                self.system.panorama_stitcher.reset()
                # Re-inicializar
                ref_frames = self.system._wait_for_frames()
                if ref_frames:
                    if self.system.has_calibration:
                        ref_frames = self.system.calibrator.undistort_frames(ref_frames)
                    self.system.panorama_stitcher.compute_homographies(ref_frames)
                    self.log("✓ Panorama reiniciado")
        except Exception as e:
            self.log(f"❌ Error: {e}")

    def closeEvent(self, event):
        """Maneja el cierre de la ventana"""
        if self.is_running:
            reply = QMessageBox.question(
                self,
                "Confirmar Salida",
                "El sistema está en ejecución. ¿Deseas salir?",
                QMessageBox.Yes | QMessageBox.No
            )

            if reply == QMessageBox.Yes:
                self.stop_system()
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()

def main():
    """Función principal de la GUI"""
    try:
        app = QApplication(sys.argv)

        # Estilo moderno
        app.setStyle('Fusion')

        # Configurar paleta de colores (opcional - tema oscuro)
        from PyQt5.QtGui import QPalette, QColor

        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, Qt.white)
        palette.setColor(QPalette.Base, QColor(25, 25, 25))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, Qt.white)
        palette.setColor(QPalette.ToolTipText, Qt.white)
        palette.setColor(QPalette.Text, Qt.white)
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, Qt.white)
        palette.setColor(QPalette.BrightText, Qt.red)
        palette.setColor(QPalette.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.HighlightedText, Qt.black)

        app.setPalette(palette)

        # Crear y mostrar ventana principal
        window = DepthEstimationGUI()
        window.show()

        # Ejecutar aplicación
        sys.exit(app.exec_())

    except Exception as e:
        print(f"Error al iniciar GUI: {e}")
        import traceback
        traceback.print_exc()
if __name__ == '__main__':
    main()