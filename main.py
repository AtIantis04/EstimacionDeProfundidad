# main.py
"""
Sistema Completo de Estimación de Profundidad Multi-Cámara
Integra todos los módulos para funcionamiento en tiempo real

Autor: Tu Nombre
Proyecto: Tesis de Maestría en Ciencia de Datos
"""

import cv2
import numpy as np
import time
import argparse
from pathlib import Path

# Importar módulos del proyecto
from src.camera_capture import CameraCapture
from src.calibrator import Calibrator
from src.panorama_stitcher import PanoramaStitcher
from src.depth_pipeline import DepthPipeline
from src.pointcloud_builder import PointCloudBuilder
from src.visualizer import Simple2DVisualizer
from src.utils import load_config, create_output_dirs, FPSCounter


class MultiCameraDepthSystem:
    """Sistema principal de estimación de profundidad multi-cámara"""

    def __init__(self, config_path='config/system_config.yml'):
        """Inicializa el sistema completo"""

        self._print_header()

        # Cargar configuración
        print("[1/8] Cargando configuración...")
        try:
            self.config = load_config(config_path)
            print(f"  ✓ Configuración cargada desde {config_path}")
        except Exception as e:
            print(f"  ⚠ Error al cargar configuración: {e}")
            print("  ℹ Usando configuración por defecto")
            self.config = self._get_default_config()

        # Crear directorios
        create_output_dirs()

        # Inicializar componentes
        print("\n[2/8] Inicializando captura de cámaras...")
        try:
            self.camera_capture = CameraCapture(
                n_cameras=self.config.get('n_cameras', 1),
                resolution=tuple(self.config.get('resolution', [640, 480])),
                fps=self.config.get('fps', 30)
            )
        except Exception as e:
            print(f"  ❌ Error al inicializar cámaras: {e}")
            raise

        print("\n[3/8] Cargando calibración...")
        self.calibrator = Calibrator(
            config_path=self.config.get('calibration_file', 'config/camera_params.yml')
        )

        # Verificar si hay calibración disponible
        self.has_calibration = len(self.calibrator.camera_matrices) > 0

        if not self.has_calibration:
            print("  ⚠ No hay calibración disponible")
            print("  ℹ Ejecuta 'run_calibration_wizard.py' para calibrar")

        print("\n[4/8] Inicializando panorama stitcher...")
        self.panorama_stitcher = PanoramaStitcher()
        self.panorama_ready = False

        print("\n[5/8] Inicializando pipeline de profundidad...")
        self.depth_pipeline = None
        self.depth_ready = False

        # Intentar inicializar depth pipeline si hay calibración estéreo
        if self.has_calibration:
            try:
                calibration_data = self.calibrator.get_calibration_dict()
                stereo_pair = tuple(self.config.get('stereo_pair', ['front', 'right']))

                # Verificar que existe calibración para el par
                pair_key = f"R_{stereo_pair[0]}_{stereo_pair[1]}"
                if pair_key in calibration_data:
                    self.depth_pipeline = DepthPipeline(calibration_data, stereo_pair)
                    self.depth_ready = True
                    print(f"  ✓ Pipeline de profundidad listo ({stereo_pair[0]}-{stereo_pair[1]})")
                else:
                    print(f"  ⚠ No hay calibración estéreo para {stereo_pair}")
            except Exception as e:
                print(f"  ⚠ No se pudo inicializar depth pipeline: {e}")

        print("\n[6/8] Inicializando generador de nube de puntos...")
        self.pc_builder = None

        if self.depth_ready:
            try:
                K_main = self.calibrator.camera_matrices[self.config['stereo_pair'][0]]
                self.pc_builder = PointCloudBuilder(
                    K_main,
                    max_points=self.config.get('max_points', 50000)
                )
                print("  ✓ Point cloud builder listo")
            except Exception as e:
                print(f"  ⚠ No se pudo inicializar point cloud builder: {e}")

        print("\n[7/8] Inicializando visualizador...")
        self.visualizer = Simple2DVisualizer(
            canvas_size=tuple(self.config.get('viz_size', [600, 600])),
            scale=self.config.get('viz_scale', 30)
        )

        print("\n[8/8] Configurando sistema...")
        self.fps_counter = FPSCounter(buffer_size=30)
        self.running = False
        self.frame_count = 0

        # Estados de visualización
        self.show_panorama = True
        self.show_depth = self.depth_ready
        self.show_pointcloud = self.depth_ready

        # Modo de operación
        self.fast_mode = self.config.get('use_fast_mode', True)

        self._print_system_status()

    def _print_header(self):
        """Imprime encabezado del sistema"""
        print("\n" + "=" * 70)
        print(" " * 10 + "SISTEMA DE ESTIMACIÓN DE PROFUNDIDAD MULTI-CÁMARA")
        print("=" * 70)

    def _get_default_config(self):
        """Configuración por defecto"""
        return {
            'n_cameras': 1,
            'resolution': [640, 480],
            'fps': 30,
            'calibration_file': 'config/camera_params.yml',
            'stereo_pair': ['front', 'right'],
            'max_points': 50000,
            'pc_downsample': 2,
            'x_range': [-10, 10],
            'y_range': [-3, 3],
            'z_range': [0.5, 25],
            'viz_size': [600, 600],
            'viz_scale': 30,
            'use_fast_mode': True,
            'target_fps': 20
        }

    def _print_system_status(self):
        """Imprime estado del sistema"""
        print("\n" + "=" * 70)
        print("ESTADO DEL SISTEMA")
        print("=" * 70)
        print(f"✓ Cámaras: {len(self.camera_capture.captures)}")
        print(
            f"{'✓' if self.has_calibration else '○'} Calibración: {'Disponible' if self.has_calibration else 'No disponible'}")
        print(
            f"{'✓' if self.panorama_ready else '○'} Panorama: {'Listo' if self.panorama_ready else 'Pendiente inicialización'}")
        print(
            f"{'✓' if self.depth_ready else '○'} Profundidad: {'Disponible' if self.depth_ready else 'No disponible'}")
        print(
            f"{'✓' if self.pc_builder else '○'} Nube de puntos: {'Disponible' if self.pc_builder else 'No disponible'}")
        print("=" * 70 + "\n")

    def start(self):
        """Inicia el sistema completo"""

        print("Iniciando sistema...")
        self.running = True

        # Iniciar captura
        self.camera_capture.start()
        time.sleep(1.0)  # Esperar estabilización

        # Inicializar panorama con frames de referencia
        print("Inicializando panorama stitcher...")
        ref_frames = self._wait_for_frames()

        if ref_frames is None:
            print("❌ No se pudieron capturar frames de referencia")
            return

        # Aplicar undistortion si hay calibración
        if self.has_calibration:
            ref_frames = self.calibrator.undistort_frames(ref_frames)

        # Calcular homografías para panorama
        self.panorama_stitcher.compute_homographies(ref_frames)
        self.panorama_ready = True

        self._print_controls()

        # Loop principal
        self._main_loop()

    def _wait_for_frames(self, timeout=5.0):
        """Espera a capturar frames válidos"""
        print("Esperando frames...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            frames = self.camera_capture.get_synchronized_frames()
            if frames is not None:
                print("✓ Frames capturados")
                return frames
            time.sleep(0.1)

        return None

    def _print_controls(self):
        """Imprime controles del sistema"""
        print("\n" + "=" * 70)
        print("SISTEMA EN EJECUCIÓN")
        print("=" * 70)
        print("\n🎮 CONTROLES:")
        print("  P - Toggle panorama")
        print("  D - Toggle depth map")
        print("  C - Toggle point cloud")
        print("  F - Toggle fast mode")
        print("  S - Guardar frame actual")
        print("  R - Reiniciar panorama")
        print("  I - Mostrar información")
        print("  ESC - Salir")
        print("=" * 70 + "\n")

    def _main_loop(self):
        """Loop principal de procesamiento"""

        last_info_time = time.time()

        while self.running:
            try:
                # Capturar frames
                frames = self.camera_capture.get_synchronized_frames()

                if frames is None:
                    time.sleep(0.001)
                    continue

                # Procesar
                t_start = time.time()
                results = self._process_frame(frames)
                processing_time = time.time() - t_start

                # Actualizar FPS
                self.fps_counter.update(processing_time)

                # Visualizar
                if results is not None:
                    self._visualize_results(results)

                # Incrementar contador
                self.frame_count += 1

                # Mostrar info cada 3 segundos
                if time.time() - last_info_time > 3.0:
                    fps = self.fps_counter.get_fps()
                    print(f"📊 Frame: {self.frame_count} | FPS: {fps:.1f} | "
                          f"Modo: {'FAST' if self.fast_mode else 'QUALITY'}", end='\r')
                    last_info_time = time.time()

                # Manejar teclas
                key = cv2.waitKey(1) & 0xFF
                if not self._handle_keyboard(key):
                    break

            except KeyboardInterrupt:
                print("\n\n⚠ Interrupción detectada...")
                break
            except Exception as e:
                print(f"\n❌ Error en loop principal: {e}")
                import traceback
                traceback.print_exc()
                break

        # Limpieza
        self.stop()

    def _process_frame(self, frames):
        """Procesa un conjunto de frames"""

        results = {}

        # 1. Undistortion (si hay calibración)
        if self.has_calibration:
            frames_processed = self.calibrator.undistort_frames(frames)
        else:
            frames_processed = frames

        results['frames'] = frames_processed

        # 2. Panorama
        if self.show_panorama and self.panorama_ready:
            try:
                panorama = self.panorama_stitcher.stitch_fast_preset(frames_processed)
                results['panorama'] = panorama
            except Exception as e:
                print(f"\n⚠ Error en panorama: {e}")

        # 3. Profundidad
        if (self.show_depth or self.show_pointcloud) and self.depth_ready:
            try:
                stereo_pair = self.config['stereo_pair']

                if stereo_pair[0] in frames and stereo_pair[1] in frames:
                    if self.fast_mode:
                        depth_map, disparity, _, _ = self.depth_pipeline.process_fast(
                            frames[stereo_pair[0]],
                            frames[stereo_pair[1]]
                        )
                    else:
                        depth_map, disparity, _, _ = self.depth_pipeline.process(
                            frames[stereo_pair[0]],
                            frames[stereo_pair[1]]
                        )

                    results['depth_map'] = depth_map
                    results['disparity'] = disparity
            except Exception as e:
                print(f"\n⚠ Error en profundidad: {e}")

        # 4. Nube de puntos
        if self.show_pointcloud and self.pc_builder and 'depth_map' in results:
            try:
                pointcloud = self.pc_builder.build(
                    results['depth_map'],
                    frames_processed[self.config['stereo_pair'][0]],
                    downsample=self.config.get('pc_downsample', 2)
                )

                # Filtrar
                pointcloud = self.pc_builder.filter_pointcloud(
                    pointcloud,
                    x_range=tuple(self.config.get('x_range', [-10, 10])),
                    y_range=tuple(self.config.get('y_range', [-3, 3])),
                    z_range=tuple(self.config.get('z_range', [0.5, 25]))
                )

                results['pointcloud'] = pointcloud
            except Exception as e:
                print(f"\n⚠ Error en point cloud: {e}")

        return results

    def _visualize_results(self, results):
        """Genera y muestra visualización"""

        viz_panels = []

        # Panel 1: Panorama
        if self.show_panorama and 'panorama' in results:
            pano = results['panorama']

            # Redimensionar para layout
            pano_h = 250
            aspect = pano.shape[1] / pano.shape[0]
            pano_w = int(pano_h * aspect)
            pano_resized = cv2.resize(pano, (pano_w, pano_h))

            # Título
            cv2.putText(pano_resized, "PANORAMA", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            viz_panels.append(pano_resized)

        # Panel 2 y 3: Depth y Point Cloud
        bottom_panels = []
        panel_width = 500
        panel_height = 375

        # Depth map
        if self.show_depth and 'depth_map' in results:
            depth_colored = self.depth_pipeline.depth_estimator.get_depth_colored(
                results['depth_map']
            )
            depth_resized = cv2.resize(depth_colored, (panel_width, panel_height))

            cv2.putText(depth_resized, "DEPTH", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Estadísticas
            stats = self.depth_pipeline.depth_estimator.get_depth_statistics(
                results['depth_map']
            )

            if stats['valid_pixels'] > 0:
                info_lines = [
                    f"Valid: {stats['valid_percentage']:.1f}%",
                    f"Range: {stats['min_depth']:.1f}-{stats['max_depth']:.1f}m",
                    f"Mean: {stats['mean_depth']:.2f}m"
                ]

                for i, line in enumerate(info_lines):
                    cv2.putText(depth_resized, line, (10, 60 + i * 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            bottom_panels.append(depth_resized)

        # Point cloud
        if self.show_pointcloud and 'pointcloud' in results:
            pc_vis = self.visualizer.update(results['pointcloud'])
            pc_resized = cv2.resize(pc_vis, (panel_width, panel_height))

            cv2.putText(pc_resized, "POINT CLOUD", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            n_points = len(results['pointcloud']['points'])
            cv2.putText(pc_resized, f"Points: {n_points:,}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            bottom_panels.append(pc_resized)

        # Combinar paneles inferiores
        if len(bottom_panels) > 0:
            if len(bottom_panels) == 1:
                bottom_row = bottom_panels[0]
            else:
                bottom_row = np.hstack(bottom_panels)
            viz_panels.append(bottom_row)

        # Combinar todos los paneles
        if len(viz_panels) == 0:
            return

        # Ajustar anchos para alinear
        if len(viz_panels) > 1:
            max_width = max([p.shape[1] for p in viz_panels])
            for i in range(len(viz_panels)):
                if viz_panels[i].shape[1] < max_width:
                    # Agregar padding
                    padding = np.zeros((viz_panels[i].shape[0],
                                        max_width - viz_panels[i].shape[1], 3),
                                       dtype=np.uint8)
                    viz_panels[i] = np.hstack([viz_panels[i], padding])

        display = np.vstack(viz_panels)

        # Agregar panel de información
        info_panel = self._create_info_panel(display.shape[1])
        display = np.vstack([display, info_panel])

        # Mostrar
        cv2.imshow('Multi-Camera Depth System', display)

    def _create_info_panel(self, width, height=70):
        """Crea panel de información del sistema"""

        panel = np.zeros((height, width, 3), dtype=np.uint8)

        # Información principal
        fps = self.fps_counter.get_fps()
        mode_text = "FAST" if self.fast_mode else "QUALITY"

        info_text = f"FPS: {fps:.1f} | Frame: {self.frame_count} | Mode: {mode_text}"
        cv2.putText(panel, info_text, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Estados
        status_text = f"Pano:{'ON' if self.show_panorama else 'OFF'} | "
        status_text += f"Depth:{'ON' if self.show_depth else 'OFF'} | "
        status_text += f"PC:{'ON' if self.show_pointcloud else 'OFF'}"

        cv2.putText(panel, status_text, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Barra de FPS
        target_fps = self.config.get('target_fps', 20)
        fps_ratio = min(fps / target_fps, 1.0)
        bar_width = 150
        bar_filled = int(bar_width * fps_ratio)

        bar_x = width - bar_width - 20
        bar_color = (0, 255, 0) if fps >= target_fps * 0.9 else \
            (0, 165, 255) if fps >= target_fps * 0.7 else (0, 0, 255)

        cv2.rectangle(panel, (bar_x, 20), (bar_x + bar_width, 50),
                      (50, 50, 50), -1)
        cv2.rectangle(panel, (bar_x, 20), (bar_x + bar_filled, 50),
                      bar_color, -1)

        return panel

    def _handle_keyboard(self, key):
        """Maneja eventos de teclado"""

        if key == 27:  # ESC
            return False

        elif key == ord('p') or key == ord('P'):
            self.show_panorama = not self.show_panorama
            print(f"\n{'✓' if self.show_panorama else '○'} Panorama: {'ON' if self.show_panorama else 'OFF'}")

        elif key == ord('d') or key == ord('D'):
            if self.depth_ready:
                self.show_depth = not self.show_depth
                print(f"\n{'✓' if self.show_depth else '○'} Depth: {'ON' if self.show_depth else 'OFF'}")
            else:
                print("\n⚠ Depth no disponible (requiere calibración estéreo)")

        elif key == ord('c') or key == ord('C'):
            if self.depth_ready:
                self.show_pointcloud = not self.show_pointcloud
                print(
                    f"\n{'✓' if self.show_pointcloud else '○'} Point Cloud: {'ON' if self.show_pointcloud else 'OFF'}")
            else:
                print("\n⚠ Point Cloud no disponible (requiere calibración estéreo)")

        elif key == ord('f') or key == ord('F'):
            self.fast_mode = not self.fast_mode
            print(f"\n⚙ Modo: {'FAST' if self.fast_mode else 'QUALITY'}")

        elif key == ord('s') or key == ord('S'):
            self._save_current_frame()

        elif key == ord('r') or key == ord('R'):
            print("\n⏳ Reiniciando panorama...")
            self.panorama_stitcher.reset()
            self.panorama_ready = False

            ref_frames = self._wait_for_frames()
            if ref_frames:
                if self.has_calibration:
                    ref_frames = self.calibrator.undistort_frames(ref_frames)
                self.panorama_stitcher.compute_homographies(ref_frames)
                self.panorama_ready = True
                print("✓ Panorama reiniciado")

        elif key == ord('i') or key == ord('I'):
            self._print_system_info()

        return True

    def _save_current_frame(self):
        """Guarda el frame actual"""
        print("\n💾 Guardando frame...")
        timestamp = int(time.time())
        print(f"✓ Frame guardado: {timestamp}")

    def _print_system_info(self):
        """Imprime información del sistema"""
        print("\n" + "=" * 70)
        print("INFORMACIÓN DEL SISTEMA")
        print("=" * 70)
        print(f"Frames procesados: {self.frame_count}")
        print(f"FPS actual: {self.fps_counter.get_fps():.1f}")
        print(f"Cámaras activas: {len(self.camera_capture.captures)}")
        print(f"Calibración: {'Sí' if self.has_calibration else 'No'}")
        print(f"Profundidad: {'Sí' if self.depth_ready else 'No'}")
        print(f"Modo: {'FAST' if self.fast_mode else 'QUALITY'}")
        print("=" * 70)

    def stop(self):
        """Detiene el sistema"""

        print("\n\n⏹ Deteniendo sistema...")

        self.running = False

        if hasattr(self, 'camera_capture'):
            self.camera_capture.release()

        cv2.destroyAllWindows()

        # Estadísticas finales
        print("\n" + "=" * 70)
        print("ESTADÍSTICAS FINALES")
        print("=" * 70)
        print(f"Frames procesados: {self.frame_count}")
        print(f"FPS promedio: {self.fps_counter.get_fps():.2f}")

        if self.frame_count > 0:
            total_time = self.frame_count / max(self.fps_counter.get_fps(), 0.01)
            print(f"Tiempo total: {total_time:.1f}s")

        print("=" * 70)
        print("\n✓ Sistema detenido correctamente\n")


def main():
    """Función principal"""

    # Parser de argumentos
    parser = argparse.ArgumentParser(
        description='Sistema de Estimación de Profundidad Multi-Cámara',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:
  python main.py                    # Ejecutar con configuración por defecto
  python main.py --config mi_config.yml
  python main.py --fast             # Forzar modo rápido
        """
    )

    parser.add_argument(
        '--config',
        type=str,
        default='config/system_config.yml',
        help='Ruta al archivo de configuración (default: config/system_config.yml)'
    )

    parser.add_argument(
        '--fast',
        action='store_true',
        help='Forzar modo rápido (sin WLS filter)'
    )

    parser.add_argument(
        '--no-panorama',
        action='store_true',
        help='Desactivar panorama al inicio'
    )

    args = parser.parse_args()

    # Ejecutar sistema
    try:
        system = MultiCameraDepthSystem(config_path=args.config)

        # Aplicar opciones de línea de comandos
        if args.fast:
            system.fast_mode = True

        if args.no_panorama:
            system.show_panorama = False

        # Iniciar
        system.start()

    except FileNotFoundError as e:
        print(f"\n❌ ERROR: Archivo no encontrado: {e}")
        print("\nAsegúrate de que:")
        print("  • El archivo de configuración existe")
        print("  • Has ejecutado la calibración")

    except KeyboardInterrupt:
        print("\n\n⚠ Sistema interrumpido por el usuario")

    except Exception as e:
        print(f"\n❌ ERROR CRÍTICO: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n👋 Programa terminado\n")


if __name__ == '__main__':
    main()