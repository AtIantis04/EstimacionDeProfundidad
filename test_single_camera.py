"""
Test básico con una sola cámara - Medición correcta de FPS
"""

from src.camera_capture import CameraCapture
import cv2
import time


def test_single_camera():
    """Prueba con una cámara"""

    print("\n" + "=" * 60)
    print("  TEST CON UNA CÁMARA")
    print("=" * 60 + "\n")

    # Crear capturador (con n_cameras=1)
    capture = CameraCapture(n_cameras=1, resolution=(640, 480))
    capture.start()
    time.sleep(1)

    print("✓ Cámara iniciada")
    print("\nControles:")
    print("  Q - Salir")
    print("  S - Guardar frame")
    print("=" * 60 + "\n")

    # Variables para FPS
    frame_count = 0
    fps_frame_count = 0
    fps_start_time = time.time()
    current_fps = 0.0
    num_frames_to_count = 30  # Calcular FPS cada 30 frames

    while True:
        # Capturar frames
        frames = capture.get_synchronized_frames()

        if frames is None:
            time.sleep(0.001)
            continue

        # Obtener el frame
        frame = frames['front']
        frame_count += 1
        fps_frame_count += 1

        # Calcular FPS cada N frames (como el código de internet)
        if fps_frame_count >= num_frames_to_count:
            fps_end_time = time.time()
            time_elapsed = fps_end_time - fps_start_time

            if time_elapsed > 0:
                current_fps = num_frames_to_count / time_elapsed

            # Reiniciar contador
            fps_frame_count = 0
            fps_start_time = time.time()

        # Determinar color según FPS
        if current_fps >= 28:
            fps_color = (0, 255, 0)  # Verde - Excelente
        elif current_fps >= 20:
            fps_color = (0, 255, 255)  # Amarillo - Bueno
        elif current_fps >= 15:
            fps_color = (0, 165, 255)  # Naranja - Aceptable
        else:
            fps_color = (0, 0, 255)  # Rojo - Bajo

        # Agregar información al frame
        cv2.putText(frame, f"Frame: {frame_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.putText(frame, f"FPS: {current_fps:.1f}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, fps_color, 2)

        cv2.putText(frame, "Q=Salir | S=Guardar", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Mostrar
        cv2.imshow('Single Camera Test', frame)

        # Control de teclado
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q'):
            break
        elif key == ord('s') or key == ord('S'):
            filename = f'output/frame_{int(time.time())}.jpg'
            cv2.imwrite(filename, frame)
            print(f"✓ Frame guardado: {filename}")

    # Limpieza
    capture.release()
    cv2.destroyAllWindows()

    print(f"\n{'=' * 60}")
    print(f"✓ Test completado")
    print(f"  Frames procesados: {frame_count}")
    print(f"  FPS final: {current_fps:.1f}")
    print(f"{'=' * 60}\n")


if __name__ == '__main__':
    test_single_camera()