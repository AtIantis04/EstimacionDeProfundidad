# test_single_camera_stereo.py
"""
Simulación de estéreo con una sola cámara
Captura dos imágenes moviendo la cámara manualmente
"""

import cv2
import numpy as np
import time
from src.stereo_matcher import StereoMatcher
from src.depth_estimator import DepthEstimator
from src.pointcloud_builder import PointCloudBuilder
from src.visualizer import Simple2DVisualizer


class SingleCameraStereo:
    """Sistema de estéreo con una sola cámara"""

    def __init__(self):
        self.baseline = 0.20  # 20cm - distancia estimada entre capturas
        self.focal_length = 800  # Estimado, se ajustará después

        # Inicializar componentes
        self.stereo_matcher = StereoMatcher(mode='sgbm', use_wls_filter=False)
        self.depth_estimator = None  # Se inicializa después de estimar focal
        self.pointcloud_builder = None
        self.visualizer = Simple2DVisualizer(canvas_size=(600, 600), scale=30)

        print("\n" + "=" * 70)
        print("  SIMULACIÓN DE ESTÉREO CON 1 CÁMARA")
        print("=" * 70)

    def run(self):
        """Ejecuta el sistema completo"""

        # Capturar par estéreo
        img_left, img_right = self.capture_stereo_pair()

        if img_left is None or img_right is None:
            print("\n❌ Captura cancelada")
            return

        print("\n✓ Par estéreo capturado exitosamente")

        # Mostrar imágenes capturadas
        self._show_stereo_pair(img_left, img_right)

        # Opciones de procesamiento
        while True:
            print("\n" + "=" * 70)
            print("OPCIONES")
            print("=" * 70)
            print("1. Calcular mapa de disparidad")
            print("2. Calcular mapa de profundidad")
            print("3. Generar nube de puntos 3D")
            print("4. Capturar nuevo par estéreo")
            print("5. Ajustar parámetros")
            print("6. Guardar resultados")
            print("0. Salir")

            choice = input("\nSelecciona opción: ").strip()

            if choice == '1':
                disparity = self.compute_disparity(img_left, img_right)
                if disparity is not None:
                    self.show_disparity(disparity)

            elif choice == '2':
                depth_map = self.compute_depth(img_left, img_right)
                if depth_map is not None:
                    self.show_depth(depth_map)

            elif choice == '3':
                self.generate_pointcloud(img_left, img_right)

            elif choice == '4':
                img_left, img_right = self.capture_stereo_pair()
                if img_left is not None and img_right is not None:
                    self._show_stereo_pair(img_left, img_right)

            elif choice == '5':
                self.adjust_parameters()

            elif choice == '6':
                self.save_results(img_left, img_right)

            elif choice == '0':
                break

            else:
                print("\n❌ Opción inválida")

        print("\n✓ Sistema finalizado\n")

    def capture_stereo_pair(self):
        """Captura par estéreo con guía paso a paso"""

        print("\n" + "-" * 70)
        print("CAPTURA DE PAR ESTÉREO")
        print("-" * 70)

        # Abrir cámara
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            print("❌ No se pudo abrir la cámara")
            return None, None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # ===== CAPTURA IMAGEN IZQUIERDA =====
        print("\n📷 PASO 1: Captura imagen IZQUIERDA")
        print("-" * 70)
        print("Instrucciones:")
        print("  1. Posiciona la cámara apuntando a la escena")
        print("  2. Mantén objetos a diferentes distancias (cerca y lejos)")
        print("  3. Evita superficies lisas sin textura")
        print("  4. Presiona ESPACIO cuando estés listo")
        print("  5. Presiona Q para cancelar")

        img_left = None

        while True:
            ret, frame = cap.read()

            if not ret:
                print("❌ Error al capturar")
                cap.release()
                return None, None

            display = frame.copy()

            # Información en pantalla
            cv2.putText(display, "IMAGEN IZQUIERDA", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(display, "Presiona ESPACIO para capturar", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Dibujar guías
            h, w = display.shape[:2]
            cv2.line(display, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)
            cv2.line(display, (0, h // 2), (w, h // 2), (0, 255, 0), 1)

            cv2.imshow('Captura Estereo', display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):
                img_left = frame.copy()
                print("  ✓ Imagen izquierda capturada")
                # Mostrar captura por 1 segundo
                cv2.putText(display, "CAPTURADA!", (w // 2 - 100, h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                cv2.imshow('Captura Estereo', display)
                cv2.waitKey(1000)
                break

            elif key == ord('q') or key == ord('Q'):
                cap.release()
                cv2.destroyAllWindows()
                return None, None

        # ===== CAPTURA IMAGEN DERECHA =====
        print("\n📷 PASO 2: Captura imagen DERECHA")
        print("-" * 70)
        print("Instrucciones:")
        print("  1. ⚠ SIN MOVER los objetos de la escena")
        print("  2. Mueve la cámara ~20-30cm a la DERECHA")
        print("  3. Mantén la cámara HORIZONTAL (no inclines)")
        print("  4. Apunta a la MISMA escena")
        print("  5. Presiona ESPACIO cuando estés listo")

        input("\n✓ Presiona ENTER cuando hayas movido la cámara...")

        img_right = None

        while True:
            ret, frame = cap.read()

            if not ret:
                print("❌ Error al capturar")
                cap.release()
                return None, None

            display = frame.copy()

            # Información en pantalla
            cv2.putText(display, "IMAGEN DERECHA", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(display, "Presiona ESPACIO para capturar", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display, f"Baseline estimado: {self.baseline * 100:.0f}cm", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            # Dibujar guías
            h, w = display.shape[:2]
            cv2.line(display, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)
            cv2.line(display, (0, h // 2), (w, h // 2), (0, 255, 0), 1)

            cv2.imshow('Captura Estereo', display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):
                img_right = frame.copy()
                print("  ✓ Imagen derecha capturada")
                cv2.putText(display, "CAPTURADA!", (w // 2 - 100, h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                cv2.imshow('Captura Estereo', display)
                cv2.waitKey(1000)
                break

            elif key == ord('q') or key == ord('Q'):
                cap.release()
                cv2.destroyAllWindows()
                return None, None

        cap.release()
        cv2.destroyAllWindows()

        return img_left, img_right

    def _show_stereo_pair(self, img_left, img_right):
        """Muestra el par estéreo lado a lado"""
        pair = np.hstack([img_left, img_right])

        h, w = img_left.shape[:2]
        cv2.putText(pair, "IZQUIERDA", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(pair, "DERECHA", (w + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Par Estereo', pair)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()

    def compute_disparity(self, img_left, img_right):
        """Calcula mapa de disparidad"""
        print("\n⏳ Calculando disparidad...")

        t_start = time.time()
        disparity = self.stereo_matcher.compute_disparity(img_left, img_right)
        elapsed = time.time() - t_start

        # Estadísticas
        valid_pixels = np.sum(disparity > 0)
        total_pixels = disparity.size
        percentage = (valid_pixels / total_pixels) * 100

        print(f"✓ Disparidad calculada en {elapsed:.2f}s")
        print(f"  Píxeles válidos: {valid_pixels:,} ({percentage:.1f}%)")

        if valid_pixels > 0:
            print(f"  Disparidad mín: {np.min(disparity[disparity > 0]):.1f} px")
            print(f"  Disparidad máx: {np.max(disparity):.1f} px")

        return disparity

    def show_disparity(self, disparity):
        """Muestra mapa de disparidad"""
        disp_vis = self.stereo_matcher.get_disparity_visualization(disparity)

        cv2.imshow('Mapa de Disparidad', disp_vis)
        print("\n💡 Presiona cualquier tecla para cerrar")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def compute_depth(self, img_left, img_right):
        """Calcula mapa de profundidad"""

        # Calcular disparidad primero
        disparity = self.compute_disparity(img_left, img_right)

        if disparity is None:
            return None

        # Inicializar depth estimator si no existe
        if self.depth_estimator is None:
            print(f"\n⚙ Inicializando depth estimator...")
            print(f"  Baseline: {self.baseline:.2f}m")
            print(f"  Focal length (estimado): {self.focal_length:.0f}px")

            self.depth_estimator = DepthEstimator(
                baseline=self.baseline,
                focal_length=self.focal_length,
                min_depth=0.3,
                max_depth=10.0
            )

        print("\n⏳ Calculando profundidad...")
        depth_map = self.depth_estimator.disparity_to_depth(disparity)

        # Estadísticas
        stats = self.depth_estimator.get_depth_statistics(depth_map)

        print(f"✓ Profundidad calculada")
        print(f"  Píxeles válidos: {stats['valid_pixels']:,} ({stats['valid_percentage']:.1f}%)")

        if stats['valid_pixels'] > 0:
            print(f"  Profundidad mín: {stats['min_depth']:.2f}m")
            print(f"  Profundidad máx: {stats['max_depth']:.2f}m")
            print(f"  Profundidad media: {stats['mean_depth']:.2f}m")

        return depth_map

    def show_depth(self, depth_map):
        """Muestra mapa de profundidad"""
        if self.depth_estimator is None:
            print("❌ Depth estimator no inicializado")
            return

        depth_vis = self.depth_estimator.get_depth_colored(depth_map)

        # Agregar escala
        h, w = depth_vis.shape[:2]
        scale_bar = np.zeros((60, w, 3), dtype=np.uint8)

        # Gradiente
        gradient = np.linspace(0, 255, w).astype(np.uint8)
        for i in range(w):
            color = cv2.applyColorMap(np.array([[gradient[i]]], dtype=np.uint8),
                                      cv2.COLORMAP_JET)[0, 0]
            scale_bar[15:45, i] = color

        # Etiquetas
        cv2.putText(scale_bar, f"{self.depth_estimator.min_depth:.1f}m", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(scale_bar, f"{self.depth_estimator.max_depth:.1f}m", (w - 70, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        result = np.vstack([depth_vis, scale_bar])

        cv2.imshow('Mapa de Profundidad', result)
        print("\n💡 Presiona cualquier tecla para cerrar")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def generate_pointcloud(self, img_left, img_right):
        """Genera y visualiza nube de puntos"""

        # Calcular profundidad
        depth_map = self.compute_depth(img_left, img_right)

        if depth_map is None:
            return

        # Inicializar point cloud builder si no existe
        if self.pointcloud_builder is None:
            # Crear matriz de cámara estimada
            h, w = img_left.shape[:2]
            K = np.array([
                [self.focal_length, 0, w / 2],
                [0, self.focal_length, h / 2],
                [0, 0, 1]
            ], dtype=np.float32)

            self.pointcloud_builder = PointCloudBuilder(K, max_points=50000)

        print("\n⏳ Generando nube de puntos...")
        pointcloud = self.pointcloud_builder.build(
            depth_map,
            img_left,
            downsample=2
        )

        # Filtrar
        pointcloud = self.pointcloud_builder.filter_pointcloud(
            pointcloud,
            x_range=(-3, 3),
            y_range=(-2, 2),
            z_range=(0.3, 10)
        )

        print(f"✓ Nube de puntos generada: {len(pointcloud['points']):,} puntos")

        # Visualizar
        print("\n🎨 Mostrando visualización 2D (vista desde arriba)...")
        pc_vis = self.visualizer.update(pointcloud)

        cv2.imshow('Nube de Puntos - Vista Superior', pc_vis)
        print("\n💡 Presiona cualquier tecla para cerrar")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Opción de guardar
        save = input("\n¿Guardar nube de puntos? (s/n): ").lower()
        if save == 's':
            timestamp = int(time.time())
            filename = f'output/pointcloud_{timestamp}.ply'
            self.pointcloud_builder.save_ply(pointcloud, filename)

    def adjust_parameters(self):
        """Ajusta parámetros del sistema"""
        print("\n" + "=" * 70)
        print("AJUSTE DE PARÁMETROS")
        print("=" * 70)

        print(f"\nParámetros actuales:")
        print(f"  Baseline: {self.baseline:.2f}m ({self.baseline * 100:.0f}cm)")
        print(f"  Focal length: {self.focal_length:.0f}px")

        print("\n1. Ajustar baseline")
        print("2. Ajustar focal length")
        print("3. Ajustar parámetros SGBM")
        print("0. Volver")

        choice = input("\nSelecciona opción: ").strip()

        if choice == '1':
            try:
                new_baseline = float(input("Nuevo baseline en metros (ej: 0.25): "))
                if 0.05 <= new_baseline <= 1.0:
                    self.baseline = new_baseline
                    self.depth_estimator = None  # Reinicializar
                    print(f"✓ Baseline actualizado: {self.baseline:.2f}m")
                else:
                    print("❌ Valor fuera de rango (0.05 - 1.0m)")
            except ValueError:
                print("❌ Valor inválido")

        elif choice == '2':
            try:
                new_focal = float(input("Nuevo focal length en píxeles (ej: 800): "))
                if 200 <= new_focal <= 2000:
                    self.focal_length = new_focal
                    self.depth_estimator = None  # Reinicializar
                    self.pointcloud_builder = None
                    print(f"✓ Focal length actualizado: {self.focal_length:.0f}px")
                else:
                    print("❌ Valor fuera de rango (200 - 2000px)")
            except ValueError:
                print("❌ Valor inválido")

        elif choice == '3':
            print("\nAjuste de parámetros SGBM:")
            try:
                num_disp = int(input("  Número de disparidades (múltiplo de 16, ej: 96): "))
                self.stereo_matcher.set_num_disparities(num_disp)

                block_size = int(input("  Tamaño de bloque (impar, 5-25, ej: 7): "))
                self.stereo_matcher.set_block_size(block_size)

                print("✓ Parámetros SGBM actualizados")
            except ValueError:
                print("❌ Valores inválidos")

    def save_results(self, img_left, img_right):
        """Guarda imágenes y resultados"""
        timestamp = int(time.time())

        print("\n💾 Guardando resultados...")

        # Guardar imágenes
        cv2.imwrite(f'output/stereo_left_{timestamp}.jpg', img_left)
        cv2.imwrite(f'output/stereo_right_{timestamp}.jpg', img_right)
        print(f"  ✓ Imágenes guardadas")

        # Calcular y guardar disparidad
        disparity = self.compute_disparity(img_left, img_right)
        if disparity is not None:
            disp_vis = self.stereo_matcher.get_disparity_visualization(disparity)
            cv2.imwrite(f'output/disparity_{timestamp}.jpg', disp_vis)
            np.save(f'output/disparity_{timestamp}.npy', disparity)
            print(f"  ✓ Disparidad guardada")

        # Calcular y guardar profundidad
        if self.depth_estimator is not None:
            depth_map = self.depth_estimator.disparity_to_depth(disparity)
            depth_vis = self.depth_estimator.get_depth_colored(depth_map)
            cv2.imwrite(f'output/depth_{timestamp}.jpg', depth_vis)
            np.save(f'output/depth_{timestamp}.npy', depth_map)
            print(f"  ✓ Profundidad guardada")

        print(f"\n✓ Resultados guardados en output/ con timestamp {timestamp}")


def main():
    """Función principal"""

    print("\n" + "=" * 70)
    print("  TEST DE ESTÉREO CON 1 CÁMARA")
    print("=" * 70)
    print("\nEste script te permite experimentar con visión estéreo")
    print("usando una sola cámara moviéndola entre dos posiciones.")
    print("\n📚 Aprenderás:")
    print("  • Cómo funciona la visión estéreo")
    print("  • Cálculo de mapas de disparidad")
    print("  • Estimación de profundidad")
    print("  • Generación de nubes de puntos 3D")
    print("=" * 70)

    input("\n✓ Presiona ENTER para comenzar...")

    system = SingleCameraStereo()

    try:
        system.run()
    except KeyboardInterrupt:
        print("\n\n⚠ Sistema interrumpido")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

    print("\n" + "=" * 70)
    print("Sistema finalizado")
    print("=" * 70 + "\n")


if __name__ == '__main__':
    main()