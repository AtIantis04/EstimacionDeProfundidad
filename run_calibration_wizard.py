# run_calibration_wizard.py
"""
Asistente interactivo de calibración para el sistema multi-cámara
"""

import cv2
import numpy as np
import os
import glob
import time
from src.camera_capture import CameraCapture
from src.calibrator import Calibrator


class CalibrationWizard:
    """Asistente de calibración paso a paso"""

    def __init__(self):
        self.calibrator = Calibrator()
        self.pattern_size = (9, 6)  # Esquinas internas del tablero
        self.square_size = 0.025  # 25mm por cuadrado

        print("\n" + "=" * 70)
        print(" " * 15 + "ASISTENTE DE CALIBRACIÓN")
        print("=" * 70)
        print("\nEste asistente te guiará por el proceso completo de calibración:")
        print("  1. Calibración individual de cada cámara")
        print("  2. Calibración estéreo entre pares de cámaras")
        print("  3. Validación de resultados")
        print("\n" + "=" * 70)

    def run(self):
        """Ejecuta el asistente completo"""
        """
        print("\n🎯 PREPARACIÓN")
        print("-" * 70)
        print("\n¿Tienes el tablero de ajedrez listo?")
        print(f"  - Patrón requerido: {self.pattern_size[0] + 1}x{self.pattern_size[1] + 1} cuadrados")
        print(f"  - Esquinas internas: {self.pattern_size[0]}x{self.pattern_size[1]}")
        print(f"  - Tamaño de cuadrado: {self.square_size * 1000:.0f}mm")
        print("\nSi no tienes el tablero, puedes descargarlo de:")
        print("  https://markhedleyjones.com/projects/calibration-checkerboard-collection")
        """
        input("\n✓ Presiona ENTER cuando estés listo para comenzar...")

        # Detectar cámaras disponibles
        camera_ids = self._detect_cameras()

        if len(camera_ids) == 0:
            print("\n❌ No se detectaron cámaras. Verifica las conexiones.")
            return

        print(f"\n✓ Cámaras detectadas: {len(camera_ids)}")

        # Menú principal
        while True:
            choice = self._show_main_menu(camera_ids)

            if choice == '1':
                self._calibrate_individual_cameras(camera_ids)
            elif choice == '2':
                self._calibrate_stereo_pairs(camera_ids)
            elif choice == '3':
                self._validate_calibration(camera_ids)
            elif choice == '4':
                self._show_calibration_status()
            elif choice == '5':
                print("\n✓ Guardando calibración...")
                self.calibrator.save_calibration('config/camera_params.yml')
                print("✓ Calibración guardada exitosamente")
            elif choice == '6':
                print("\n👋 Saliendo del asistente...")
                break
            else:
                print("\n❌ Opción inválida")

    def _detect_cameras(self):
        """Detecta cámaras disponibles"""
        print("\n🔍 Detectando cámaras...")

        available = []
        camera_names = ['front', 'right', 'back', 'left']

        for i in range(4):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    cam_name = camera_names[i] if i < len(camera_names) else f'cam{i}'
                    available.append({'id': i, 'name': cam_name})
                    print(f"  ✓ Cámara {i}: {cam_name}")
                cap.release()

        return available

    def _show_main_menu(self, camera_ids):
        """Muestra menú principal"""
        print("\n" + "=" * 70)
        print("MENÚ PRINCIPAL")
        print("=" * 70)
        print("\n1. Calibración individual de cámaras")
        print("2. Calibración estéreo (pares de cámaras)")
        print("3. Validar calibración")
        print("4. Mostrar estado de calibración")
        print("5. Guardar calibración")
        print("6. Salir")

        choice = input("\nSelecciona una opción (1-6): ").strip()
        return choice

    def _calibrate_individual_cameras(self, camera_ids):
        """Calibración individual de cada cámara"""
        print("\n" + "=" * 70)
        print("CALIBRACIÓN INDIVIDUAL")
        print("=" * 70)

        print("\nCámaras disponibles:")
        for i, cam in enumerate(camera_ids):
            status = "✓" if cam['name'] in self.calibrator.camera_matrices else "○"
            print(f"  {status} {i + 1}. {cam['name']} (USB {cam['id']})")

        print("\n0. Volver al menú principal")

        choice = input("\nSelecciona cámara a calibrar (0 para volver): ").strip()

        if choice == '0':
            return

        try:
            idx = int(choice) - 1
            if 0 <= idx < len(camera_ids):
                cam = camera_ids[idx]
                self._calibrate_single_camera(cam['name'], cam['id'])
            else:
                print("\n❌ Opción inválida")
        except ValueError:
            print("\n❌ Opción inválida")

    def _calibrate_single_camera(self, cam_name, usb_id, num_images=25):
        """Calibra una cámara individual"""
        print("\n" + "-" * 70)
        print(f"CALIBRANDO: {cam_name.upper()}")
        print("-" * 70)

        # Verificar si ya existe calibración
        if cam_name in self.calibrator.camera_matrices:
            overwrite = input(f"\n⚠ {cam_name} ya está calibrada. ¿Recalibrar? (s/n): ").lower()
            if overwrite != 's':
                return

        # Crear directorio para imágenes
        output_dir = f'calibration_data/{cam_name}'
        os.makedirs(output_dir, exist_ok=True)

        # Verificar si ya hay imágenes capturadas
        existing_images = glob.glob(f'{output_dir}/calib_*.jpg')

        if len(existing_images) >= 10:
            print(f"\n✓ Se encontraron {len(existing_images)} imágenes existentes")
            use_existing = input("¿Usar imágenes existentes? (s/n): ").lower()

            if use_existing == 's':
                images = [cv2.imread(f) for f in sorted(existing_images)]
            else:
                # Limpiar directorio
                for f in existing_images:
                    os.remove(f)
                images = self._capture_calibration_images(cam_name, usb_id, num_images)
        else:
            images = self._capture_calibration_images(cam_name, usb_id, num_images)

        if images is None or len(images) < 10:
            print("\n❌ Calibración cancelada (imágenes insuficientes)")
            return

        # Calibrar
        print(f"\n📊 Procesando {len(images)} imágenes...")

        try:
            result = self.calibrator.calibrate_camera(
                images,
                pattern_size=self.pattern_size,
                square_size=self.square_size
            )

            # Guardar resultados
            self.calibrator.camera_matrices[cam_name] = result['camera_matrix']
            self.calibrator.dist_coeffs[cam_name] = result['dist_coeffs']

            print("\n" + "=" * 70)
            print("✅ CALIBRACIÓN EXITOSA")
            print("=" * 70)
            print(f"RMS Error: {result['rms_error']:.4f} píxeles")
            print(f"Error de reproyección medio: {result['mean_reproj_error']:.4f} píxeles")
            print(f"Imágenes usadas: {result['images_used']}/{len(images)}")

            # Mostrar ejemplo de corrección de distorsión
            self._show_undistortion_example(images[0], result)

            input("\n✓ Presiona ENTER para continuar...")

        except Exception as e:
            print(f"\n❌ Error durante calibración: {e}")

    def _capture_calibration_images(self, cam_name, usb_id, num_images):
        """Captura imágenes para calibración"""
        print("\n" + "-" * 70)
        print("CAPTURA DE IMÁGENES")
        print("-" * 70)
        print(f"\nObjetivo: Capturar {num_images} imágenes del tablero")
        print("\nInstrucciones:")
        print("  1. Mueve el tablero a diferentes posiciones")
        print("  2. Varía el ángulo y la distancia")
        print("  3. Cubre todas las áreas de la imagen")
        print("  4. Incluye: esquinas, centro, cerca, lejos")
        print("\nControles:")
        print("  ESPACIO - Capturar (cuando el patrón sea detectado)")
        print("  Q - Salir/Cancelar")
        print("-" * 70)

        input("\n✓ Presiona ENTER para iniciar la captura...")

        # Inicializar cámara
        cap = cv2.VideoCapture(usb_id)

        if not cap.isOpened():
            print(f"\n❌ No se pudo abrir la cámara {usb_id}")
            return None

        # Configurar cámara para alta resolución (mejor para calibración)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        output_dir = f'calibration_data/{cam_name}'
        images = []
        count = 0
        last_capture_time = 0

        print(f"\n🎥 Capturando desde cámara {usb_id}...")

        while count < num_images:
            ret, frame = cap.read()

            if not ret:
                print("\n❌ Error al capturar frame")
                break

            display = frame.copy()

            # Detectar tablero
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_pattern, corners = cv2.findChessboardCorners(
                gray, self.pattern_size, None
            )

            # Indicador visual
            if ret_pattern:
                cv2.drawChessboardCorners(display, self.pattern_size, corners, ret_pattern)
                status_text = "PATRON DETECTADO - Presiona ESPACIO"
                status_color = (0, 255, 0)
            else:
                status_text = "Buscando patron..."
                status_color = (0, 0, 255)

            # Información en pantalla
            cv2.putText(display, status_text, (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

            cv2.putText(display, f"Capturadas: {count}/{num_images}", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            cv2.putText(display, f"Camara: {cam_name}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            # Barra de progreso
            bar_width = display.shape[1] - 40
            bar_height = 30
            bar_x, bar_y = 20, display.shape[0] - 60

            cv2.rectangle(display, (bar_x, bar_y),
                          (bar_x + bar_width, bar_y + bar_height),
                          (100, 100, 100), -1)

            progress = int((count / num_images) * bar_width)
            cv2.rectangle(display, (bar_x, bar_y),
                          (bar_x + progress, bar_y + bar_height),
                          (0, 255, 0), -1)

            cv2.imshow(f'Calibration - {cam_name}', display)

            key = cv2.waitKey(1) & 0xFF
            current_time = time.time()

            # Capturar
            if key == ord(' ') and ret_pattern and (current_time - last_capture_time > 0.5):
                filename = f"{output_dir}/calib_{count:03d}.jpg"
                cv2.imwrite(filename, frame)
                images.append(frame)
                count += 1
                last_capture_time = current_time

                print(f"  ✓ Imagen {count}/{num_images} guardada")

                # Feedback visual
                cv2.imshow(f'Calibration - {cam_name}', frame)
                cv2.waitKey(200)

            elif key == ord('q') or key == ord('Q'):
                print("\n⚠ Captura cancelada")
                break

        cap.release()
        cv2.destroyAllWindows()

        if count < num_images:
            print(f"\n⚠ Solo se capturaron {count}/{num_images} imágenes")
            if count < 10:
                return None

        return images

    def _calibrate_stereo_pairs(self, camera_ids):
        """Calibración estéreo entre pares"""
        print("\n" + "=" * 70)
        print("CALIBRACIÓN ESTÉREO")
        print("=" * 70)

        # Verificar que haya al menos 2 cámaras calibradas
        calibrated = [cam['name'] for cam in camera_ids
                      if cam['name'] in self.calibrator.camera_matrices]

        if len(calibrated) < 2:
            print("\n❌ Se necesitan al menos 2 cámaras calibradas individualmente")
            print(f"   Calibradas: {len(calibrated)}")
            input("\n✓ Presiona ENTER para continuar...")
            return

        print(f"\n✓ Cámaras calibradas: {', '.join(calibrated)}")
        print("\nPares comunes para estéreo:")
        print("  1. front - right")
        print("  2. front - left")
        print("  3. Otro par personalizado")
        print("\n0. Volver al menú principal")

        choice = input("\nSelecciona opción: ").strip()

        if choice == '0':
            return
        elif choice == '1':
            self._calibrate_stereo_pair('front', 'right')
        elif choice == '2':
            self._calibrate_stereo_pair('front', 'left')
        elif choice == '3':
            print("\nCámaras disponibles:", ', '.join(calibrated))
            left = input("Cámara izquierda: ").strip()
            right = input("Cámara derecha: ").strip()

            if left in calibrated and right in calibrated:
                self._calibrate_stereo_pair(left, right)
            else:
                print("\n❌ Una o ambas cámaras no están calibradas")
        else:
            print("\n❌ Opción inválida")

    def _calibrate_stereo_pair(self, cam_left, cam_right, num_pairs=20):
        """Calibra un par estéreo específico"""
        print("\n" + "-" * 70)
        print(f"CALIBRACIÓN ESTÉREO: {cam_left.upper()} - {cam_right.upper()}")
        print("-" * 70)

        pair_id = f"{cam_left}_{cam_right}"
        stereo_dir = f'calibration_data/stereo/{pair_id}'
        os.makedirs(stereo_dir, exist_ok=True)

        # Verificar imágenes existentes
        left_images = sorted(glob.glob(f'{stereo_dir}/left_*.jpg'))
        right_images = sorted(glob.glob(f'{stereo_dir}/right_*.jpg'))

        if len(left_images) >= 10 and len(right_images) >= 10:
            print(f"\n✓ Se encontraron {len(left_images)} pares de imágenes")
            use_existing = input("¿Usar imágenes existentes? (s/n): ").lower()

            if use_existing == 's':
                images_left = [cv2.imread(f) for f in left_images]
                images_right = [cv2.imread(f) for f in right_images]
            else:
                images_left, images_right = self._capture_stereo_images(
                    cam_left, cam_right, num_pairs
                )
        else:
            images_left, images_right = self._capture_stereo_images(
                cam_left, cam_right, num_pairs
            )

        if images_left is None or len(images_left) < 10:
            print("\n❌ Calibración cancelada (imágenes insuficientes)")
            return

        # Calibrar
        print(f"\n📊 Procesando {len(images_left)} pares de imágenes...")

        try:
            K_left = self.calibrator.camera_matrices[cam_left]
            K_right = self.calibrator.camera_matrices[cam_right]
            dist_left = self.calibrator.dist_coeffs[cam_left]
            dist_right = self.calibrator.dist_coeffs[cam_right]

            result = self.calibrator.calibrate_stereo_pair(
                images_left, images_right,
                K_left, dist_left, K_right, dist_right,
                pattern_size=self.pattern_size,
                square_size=self.square_size
            )

            # Guardar resultados
            self.calibrator.extrinsics[pair_id] = result

            print("\n" + "=" * 70)
            print("✅ CALIBRACIÓN ESTÉREO EXITOSA")
            print("=" * 70)
            print(f"RMS Error: {result['rms_error']:.4f}")
            print(f"Baseline: {result['baseline']:.4f} m ({result['baseline'] * 100:.2f} cm)")

            input("\n✓ Presiona ENTER para continuar...")

        except Exception as e:
            print(f"\n❌ Error durante calibración estéreo: {e}")

    def _capture_stereo_images(self, cam_left, cam_right, num_pairs):
        """Captura pares de imágenes simultáneas"""
        print("\n⚠ IMPORTANTE: Captura de pares estéreo")
        print("-" * 70)
        print("Las dos cámaras deben capturar el MISMO tablero SIMULTÁNEAMENTE")
        print("Asegúrate de que:")
        print("  - Ambas cámaras ven el tablero completo")
        print("  - El tablero está en la misma posición para ambas")
        print("  - Capturas cada posición solo una vez")
        print("-" * 70)

        input("\n✓ Presiona ENTER para iniciar...")

        # Por ahora, instrucciones para captura manual
        # En el futuro, esto podría automatizarse con CameraCapture

        print("\n📝 INSTRUCCIONES:")
        print(f"1. Captura {num_pairs} pares de imágenes del tablero")
        print("2. Usa 'capture_calibration_images.py' para cada cámara")
        print("3. Guarda las imágenes en:")
        print(f"   - calibration_data/stereo/{cam_left}_{cam_right}/left_XX.jpg")
        print(f"   - calibration_data/stereo/{cam_left}_{cam_right}/right_XX.jpg")
        print("\n⚠ Por ahora, esta función no está completamente implementada")
        print("   Vuelve al menú principal y usa la opción de calibración individual")

        input("\n✓ Presiona ENTER para continuar...")

        return None, None

    def _validate_calibration(self, camera_ids):
        """Valida la calibración mostrando corrección de distorsión"""
        print("\n" + "=" * 70)
        print("VALIDACIÓN DE CALIBRACIÓN")
        print("=" * 70)

        calibrated = [cam for cam in camera_ids
                      if cam['name'] in self.calibrator.camera_matrices]

        if len(calibrated) == 0:
            print("\n❌ No hay cámaras calibradas para validar")
            input("\n✓ Presiona ENTER para continuar...")
            return

        print("\nCámaras calibradas:")
        for i, cam in enumerate(calibrated):
            print(f"  {i + 1}. {cam['name']}")

        print("\n0. Volver")

        choice = input("\nSelecciona cámara a validar: ").strip()

        if choice == '0':
            return

        try:
            idx = int(choice) - 1
            if 0 <= idx < len(calibrated):
                cam = calibrated[idx]
                self._show_live_undistortion(cam['name'], cam['id'])
            else:
                print("\n❌ Opción inválida")
        except ValueError:
            print("\n❌ Opción inválida")

    def _show_live_undistortion(self, cam_name, usb_id):
        """Muestra corrección de distorsión en vivo"""
        print(f"\n🎥 Mostrando corrección para {cam_name}")
        print("Presiona Q para salir")

        cap = cv2.VideoCapture(usb_id)

        if not cap.isOpened():
            print(f"\n❌ No se pudo abrir cámara {usb_id}")
            return

        while True:
            ret, frame = cap.read()

            if not ret:
                break

            # Aplicar corrección
            undistorted = self.calibrator.undistort_frame(frame, cam_name)

            # Mostrar lado a lado
            comparison = np.hstack([frame, undistorted])

            # Agregar etiquetas
            h, w = frame.shape[:2]
            cv2.putText(comparison, "ORIGINAL", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(comparison, "CORREGIDA", (w + 10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow(f'Validation - {cam_name}', comparison)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def _show_undistortion_example(self, image, calib_result):
        """Muestra ejemplo de corrección de distorsión"""
        K = calib_result['camera_matrix']
        dist = calib_result['dist_coeffs']

        h, w = image.shape[:2]
        new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
        undistorted = cv2.undistort(image, K, dist, None, new_K)

        # Mostrar lado a lado
        comparison = np.hstack([image, undistorted])

        cv2.putText(comparison, "ORIGINAL", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(comparison, "CORREGIDA", (w + 10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Undistortion Example', comparison)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()

    def _show_calibration_status(self):
        """Muestra estado actual de la calibración"""
        print("\n" + "=" * 70)
        print("ESTADO DE CALIBRACIÓN")
        print("=" * 70)

        # Calibración individual
        print("\n📷 CALIBRACIÓN INDIVIDUAL:")
        if len(self.calibrator.camera_matrices) == 0:
            print("  ○ Ninguna cámara calibrada")
        else:
            for cam_id in self.calibrator.camera_matrices.keys():
                print(f"  ✓ {cam_id}")

        # Calibración estéreo
        print("\n👁 CALIBRACIÓN ESTÉREO:")
        if len(self.calibrator.extrinsics) == 0:
            print("  ○ Ningún par calibrado")
        else:
            for pair_id, data in self.calibrator.extrinsics.items():
                baseline = data.get('baseline', 0)
                print(f"  ✓ {pair_id}: baseline={baseline * 100:.2f}cm")

        input("\n✓ Presiona ENTER para continuar...")


def main():
    """Función principal"""
    wizard = CalibrationWizard()

    try:
        wizard.run()
    except KeyboardInterrupt:
        print("\n\n⚠ Asistente interrumpido por el usuario")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

    print("\n" + "=" * 70)
    print("Asistente de calibración finalizado")
    print("=" * 70 + "\n")


if __name__ == '__main__':
    main()
