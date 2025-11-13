
"""
Script para capturar imágenes de calibración con tablero de ajedrez
"""

import cv2
import os
import time
from src.camera_capture import CameraCapture


def capture_calibration_set(cam_id='front', num_images=25, pattern_size=(9, 6)):
    """
    Captura imágenes para calibración de una cámara

    Args:
        cam_id: ID de la cámara a calibrar
        num_images: Número de imágenes a capturar
        pattern_size: (cols-1, rows-1) del tablero
    """
    output_dir = f'calibration_data/{cam_id}'
    os.makedirs(output_dir, exist_ok=True)

    # Inicializar captura
    capture = CameraCapture(n_cameras=1, resolution=(1280, 720))  # Mayor resolución para calibración
    capture.start()
    time.sleep(1)

    count = 0
    last_capture_time = 0

    print("\n" + "=" * 60)
    print(f"  CAPTURA DE IMÁGENES DE CALIBRACIÓN - Cámara: {cam_id}")
    print("=" * 60)
    print(f"\nObjetivo: Capturar {num_images} imágenes del tablero de ajedrez")
    print(f"Patrón: {pattern_size[0]}x{pattern_size[1]} esquinas internas")
    print("\nInstrucciones:")
    print("  1. Mueve el tablero a diferentes posiciones y ángulos")
    print("  2. Cubre todo el campo de visión de la cámara")
    print("  3. Incluye esquinas, centro, cerca y lejos")
    print("\nControles:")
    print("  ESPACIO - Capturar imagen (cuando el patrón sea detectado)")
    print("  Q - Salir")
    print("=" * 60 + "\n")

    while count < num_images:
        frames = capture.get_synchronized_frames()

        if frames is None or cam_id not in frames:
            time.sleep(0.01)
            continue

        frame = frames[cam_id].copy()
        display = frame.copy()

        # Intentar detectar patrón
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        # Dibujar esquinas si se detectan
        if ret:
            cv2.drawChessboardCorners(display, pattern_size, corners, ret)
            cv2.putText(display, "PATRON DETECTADO - Presiona ESPACIO",
                        (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            status_color = (0, 255, 0)
        else:
            cv2.putText(display, "Buscando patron...", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            status_color = (0, 0, 255)

        # Información
        cv2.putText(display, f"Capturadas: {count}/{num_images}", (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Barra de progreso
        bar_width = display.shape[1] - 40
        bar_height = 30
        bar_x = 20
        bar_y = display.shape[0] - 60

        cv2.rectangle(display, (bar_x, bar_y),
                      (bar_x + bar_width, bar_y + bar_height),
                      (100, 100, 100), -1)

        progress = int((count / num_images) * bar_width)
        cv2.rectangle(display, (bar_x, bar_y),
                      (bar_x + progress, bar_y + bar_height),
                      (0, 255, 0), -1)

        cv2.imshow(f'Calibration - {cam_id}', display)

        key = cv2.waitKey(1) & 0xFF

        # Capturar si se presiona ESPACIO y el patrón es detectado
        current_time = time.time()
        if key == ord(' ') and ret and (current_time - last_capture_time > 0.5):
            filename = f"{output_dir}/calib_{count:03d}.jpg"
            cv2.imwrite(filename, frame)
            count += 1
            last_capture_time = current_time
            print(f"✓ Imagen {count}/{num_images} guardada")

            # Feedback visual
            cv2.imshow(f'Calibration - {cam_id}', frame)
            cv2.waitKey(200)

        elif key == ord('q'):
            print("\nCaptura cancelada por el usuario")
            break

    capture.release()
    cv2.destroyAllWindows()

    print(f"\n{'=' * 60}")
    print(f"✓ Calibración completada: {count} imágenes guardadas en {output_dir}")
    print(f"{'=' * 60}\n")


if __name__ == '__main__':
    # Capturar para la cámara disponible
    capture_calibration_set(cam_id='front', num_images=25)