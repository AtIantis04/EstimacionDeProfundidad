"""
Script para identificar los índices USB de las cámaras conectadas
"""

import cv2


def find_cameras(max_tested=10):
    """Encuentra todas las cámaras disponibles"""
    print("Buscando cámaras conectadas...\n")
    available = []

    for i in range(max_tested):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                available.append(i)
                h, w = frame.shape[:2]
                print(f"✓ Cámara encontrada en índice {i} - Resolución: {w}x{h}")

                # Mostrar frame
                cv2.putText(frame, f"Camera {i}", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
                cv2.imshow(f'Camera {i}', frame)
                cv2.waitKey(1000)  # Mostrar por 1 segundo
            cap.release()

    cv2.destroyAllWindows()
    return available


if __name__ == '__main__':
    print("=" * 50)
    print("  TEST DE IDENTIFICACIÓN DE CÁMARAS")
    print("=" * 50)

    cameras = find_cameras()

    print(f"\n{'=' * 50}")
    print(f"Total de cámaras encontradas: {len(cameras)}")
    print(f"Índices: {cameras}")
    print(f"{'=' * 50}\n")

    if len(cameras) > 0:
        print("Camera_ids")
        print("Esto lo debo editar en el archivo src/camera_capture.py cada que cambie el setup de prueba para evitar errores futuros.")
        print("\nEjemplo:")
        print("self.camera_ids = {")
        for i, idx in enumerate(cameras):
            cam_name = ['front', 'right', 'back', 'left'][i] if i < 4 else f'cam{i}'
            print(f"    '{cam_name}': {idx},")
        print("}")
    else:
        print("⚠ No se encontraron cámaras. Verifica las conexiones USB.")