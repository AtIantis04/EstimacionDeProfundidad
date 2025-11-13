# test_fps_diagnosis.py
"""
Diagnóstico detallado de FPS
"""

import cv2
import time


def diagnose_camera_fps():
    """Diagnóstico directo de la cámara sin nuestro sistema"""

    print("\n" + "=" * 60)
    print("  DIAGNÓSTICO DE FPS DE CÁMARA")
    print("=" * 60 + "\n")

    # Abrir cámara directamente
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ Error: No se pudo abrir la cámara")
        return

    # Configurar
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Leer FPS reportado por la cámara
    reported_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"FPS reportado por la cámara: {reported_fps}")

    # Descartar primeros frames (buffer inicial)
    print("\nDescartando buffer inicial...")
    for _ in range(30):
        cap.read()

    print("Midiendo FPS real...\n")

    # Medir FPS real
    frame_times = []
    frame_count = 0
    start_time = time.time()

    print("Presiona Q para salir\n")

    while frame_count < 300:  # Medir 300 frames
        loop_start = time.time()

        ret, frame = cap.read()

        if not ret:
            print("⚠ Warning: Frame no capturado")
            continue

        frame_time = time.time()

        if len(frame_times) > 0:
            delta = frame_time - frame_times[-1]
            frame_times.append(frame_time)

            if delta > 0:
                instant_fps = 1.0 / delta

                # Mostrar cada 30 frames
                if frame_count % 30 == 0:
                    avg_fps = len(frame_times) / (frame_time - frame_times[0])
                    print(f"Frame {frame_count}: Instantáneo={instant_fps:.1f} fps, "
                          f"Promedio={avg_fps:.1f} fps, Delta={delta * 1000:.1f}ms")
        else:
            frame_times.append(frame_time)

        # Agregar info al frame
        cv2.putText(frame, f"Frame: {frame_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if len(frame_times) > 1:
            current_fps = len(frame_times) / (frame_time - frame_times[0])
            cv2.putText(frame, f"FPS: {current_fps:.1f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('FPS Diagnosis', frame)
        frame_count += 1

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # Calcular estadísticas finales
    total_time = time.time() - start_time
    final_fps = frame_count / total_time

    cap.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 60)
    print("RESULTADOS:")
    print("=" * 60)
    print(f"Frames capturados: {frame_count}")
    print(f"Tiempo total: {total_time:.2f} segundos")
    print(f"FPS promedio: {final_fps:.2f}")
    print(f"FPS reportado por cámara: {reported_fps}")
    print("=" * 60 + "\n")

    # Análisis
    if abs(final_fps - 30) < 2:
        print("✅ FPS normales (cerca de 30 fps)")
    elif abs(final_fps - 15) < 2:
        print("⚠ FPS reducidos (cerca de 15 fps) - puede ser limitación de hardware")
    elif final_fps > 50:
        print("❌ FPS anormalmente altos - problema en la medición o buffer")
    else:
        print(f"⚠ FPS inusuales: {final_fps:.1f}")


if __name__ == '__main__':
    diagnose_camera_fps()