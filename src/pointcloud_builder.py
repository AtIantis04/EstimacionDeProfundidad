# src/pointcloud_builder.py
"""
Generador de nube de puntos 3D desde mapa de profundidad
"""

import cv2
import numpy as np
import os


class PointCloudBuilder:
    """Generador de nube de puntos 3D"""

    def __init__(self, camera_matrix, max_points=100000):
        """
        Args:
            camera_matrix: Matriz K de la cámara (3x3)
            max_points: Límite de puntos para optimización
        """
        self.K = camera_matrix
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]

        self.max_points = max_points

        # Transformación al sistema global (default: identidad)
        self.T_global = np.eye(4)

        print(f"\n{'=' * 60}")
        print("INICIALIZANDO POINT CLOUD BUILDER")
        print(f"{'=' * 60}")
        print(f"✓ Parámetros intrínsecos:")
        print(f"  fx: {self.fx:.2f} px")
        print(f"  fy: {self.fy:.2f} px")
        print(f"  cx: {self.cx:.2f} px")
        print(f"  cy: {self.cy:.2f} px")
        print(f"✓ Límite de puntos: {max_points:,}")
        print(f"{'=' * 60}\n")

    def set_global_transform(self, R, t):
        """
        Establece transformación al sistema de referencia global

        Args:
            R: Matriz de rotación 3x3
            t: Vector de traslación 3x1 o (3,)
        """
        self.T_global = np.eye(4)
        self.T_global[:3, :3] = R
        self.T_global[:3, 3] = t.flatten()

        print("✓ Transformación global actualizada")

    def build(self, depth_map, color_image=None, downsample=1):
        """
        Genera nube de puntos desde mapa de profundidad

        Args:
            depth_map: Mapa de profundidad (H, W) en metros
            color_image: Imagen BGR opcional para colorear puntos
            downsample: Factor de submuestreo (1=sin submuestreo, 2=mitad, etc.)

        Returns:
            pointcloud: Dict con 'points' (Nx3) y 'colors' (Nx3)
        """
        h, w = depth_map.shape

        # Submuestreo para optimización
        if downsample > 1:
            depth_ds = depth_map[::downsample, ::downsample]
            if color_image is not None:
                color_ds = color_image[::downsample, ::downsample]
            else:
                color_ds = None
            h_ds, w_ds = depth_ds.shape
        else:
            depth_ds = depth_map
            color_ds = color_image
            h_ds, w_ds = h, w

        # Crear grilla de coordenadas píxel
        u, v = np.meshgrid(np.arange(w_ds), np.arange(h_ds))

        # Ajustar coordenadas si hay submuestreo
        if downsample > 1:
            u = u * downsample
            v = v * downsample

        # Máscara de píxeles válidos
        valid_mask = depth_ds > 0

        # Extraer coordenadas válidas
        u_valid = u[valid_mask]
        v_valid = v[valid_mask]
        z_valid = depth_ds[valid_mask]

        # Limitar número de puntos si es necesario
        if len(z_valid) > self.max_points:
            indices = np.random.choice(len(z_valid), self.max_points, replace=False)
            u_valid = u_valid[indices]
            v_valid = v_valid[indices]
            z_valid = z_valid[indices]

        # Proyección inversa: píxel → 3D (sistema de cámara)
        x_cam = (u_valid - self.cx) * z_valid / self.fx
        y_cam = (v_valid - self.cy) * z_valid / self.fy
        z_cam = z_valid

        # Apilar como matriz Nx3
        points_cam = np.stack([x_cam, y_cam, z_cam], axis=1)

        # Transformar a sistema global
        points_homogeneous = np.hstack([points_cam, np.ones((len(points_cam), 1))])
        points_global = (self.T_global @ points_homogeneous.T).T[:, :3]

        # Extraer colores si están disponibles
        colors = None
        if color_ds is not None:
            if downsample > 1:
                v_ds = v_valid // downsample
                u_ds = u_valid // downsample
                # Asegurar que índices estén dentro de límites
                v_ds = np.clip(v_ds, 0, h_ds - 1)
                u_ds = np.clip(u_ds, 0, w_ds - 1)
                colors_bgr = color_ds[v_ds, u_ds]
            else:
                colors_bgr = color_ds[valid_mask]

            # Convertir BGR a RGB y normalizar a [0, 1]
            colors = colors_bgr[:, ::-1] / 255.0  # BGR -> RGB

            if len(z_valid) > self.max_points:
                colors = colors[indices]

        return {
            'points': points_global.astype(np.float32),
            'colors': colors.astype(np.float32) if colors is not None else None
        }

    def filter_pointcloud(self, pointcloud,
                          x_range=(-10, 10),
                          y_range=(-5, 5),
                          z_range=(0, 30)):
        """
        Filtra nube de puntos por rango espacial

        Args:
            pointcloud: Dict con 'points' y 'colors'
            x_range, y_range, z_range: Tuplas (min, max) en metros

        Returns:
            filtered_pointcloud: Nube de puntos filtrada
        """
        points = pointcloud['points']
        colors = pointcloud['colors']

        # Crear máscara de rango
        mask = (
                (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
                (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
                (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
        )

        filtered = {
            'points': points[mask],
            'colors': colors[mask] if colors is not None else None
        }

        return filtered

    def save_ply(self, pointcloud, filename='output/pointcloud.ply'):
        """
        Guarda nube de puntos en formato PLY

        Args:
            pointcloud: Dict con 'points' (Nx3) y 'colors' (Nx3)
            filename: Ruta del archivo de salida
        """
        points = pointcloud['points']
        colors = pointcloud['colors']

        n_points = len(points)

        # Crear directorio si no existe
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        # Escribir encabezado PLY
        with open(filename, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {n_points}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")

            if colors is not None:
                f.write("property uchar red\n")
                f.write("property uchar green\n")
                f.write("property uchar blue\n")

            f.write("end_header\n")

            # Escribir datos
            for i in range(n_points):
                x, y, z = points[i]
                f.write(f"{x} {y} {z}")

                if colors is not None:
                    r, g, b = (colors[i] * 255).astype(int)
                    f.write(f" {r} {g} {b}")

                f.write("\n")

        print(f"✓ Nube de puntos guardada: {filename} ({n_points:,} puntos)")

    def save_pcd(self, pointcloud, filename='output/pointcloud.pcd'):
        """
        Guarda en formato PCD (Point Cloud Data)

        Args:
            pointcloud: Dict con 'points' y 'colors'
            filename: Ruta del archivo de salida
        """
        points = pointcloud['points']
        colors = pointcloud['colors']

        n_points = len(points)

        # Crear directorio si no existe
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        with open(filename, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z rgb\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
            f.write(f"WIDTH {n_points}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {n_points}\n")
            f.write("DATA ascii\n")

            for i in range(n_points):
                x, y, z = points[i]

                if colors is not None:
                    r, g, b = (colors[i] * 255).astype(int)
                    rgb = (r << 16) | (g << 8) | b
                else:
                    rgb = 0

                f.write(f"{x} {y} {z} {rgb}\n")

        print(f"✓ Nube de puntos guardada: {filename} ({n_points:,} puntos)")