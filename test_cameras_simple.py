"""
Prueba simple de captura con las 4 cámaras
"""

from src.camera_capture import CameraCapture
import cv2
import numpy as np
import time


def test_camera_capture():
    """Prueba básica del sistema de captura"""

    print("\n" + "=" * 60)
    print("  TEST DE CAPTURA MULTI-CÁMARA")
    print("=" * 60 + "\n")

    # Crear capturador
    try:
        capture = CameraCapture(n_cameras=4, resolution=(640, 480))
    except Exception as e:
        print(f"\n❌ Error al inicializar cámaras: {e}")
        print("\n💡 Consejo: Ejecuta primero 'test_camera_ids.py'")
        print("   para identificar los índices correctos de tus cámaras\n")
        return

    # Iniciar captura
    capture.start()
    time.sleep(1)  # Esperar estabilización

    print("\n✓ Sistema iniciado")
    print("\nControles:")
    print("  Q - Salir")
    print("  S - Guardar frame actual")
    print("=" * 60 + "\n")

    frame_count = 0

    while True:
        # Obtener frames sincronizados
        frames = capture.get_synchronized_frames()

        if frames is None:
            time.sleep(0.01)
            continue

        # Crear grid 2x2
        grid = np.zeros((960, 1280, 3), dtype=np.uint8)

        positions = {
            'front': (0, 0),
            'right': (0, 640),
            'back': (480, 0),
            'left': (480, 640)
        }

        for cam_id, (y, x) in positions.items():
            if cam_id in frames:
                frame = cv2.resize(frames[cam_id], (640, 480))

                # Agregar etiqueta
                cv2.putText(frame, cam_id.upper(), (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                grid[y:y + 480, x:x + 640] = frame

        # Agregar contador de frames
        cv2.putText(grid, f"Frame: {frame_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow('Multi-Camera Test', grid)
        frame_count += 1

        # Control de teclado
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q'):
            break
        elif key == ord('s') or key == ord('S'):
            filename = f'output/test_frame_{int(time.time())}.jpg'
            cv2.imwrite(filename, grid)
            print(f"✓ Frame guardado: {filename}")

    # Limpieza
    capture.release()
    cv2.destroyAllWindows()

    print(f"\n✓ Test completado. Frames procesados: {frame_count}\n")


if __name__ == '__main__':
    test_camera_capture()