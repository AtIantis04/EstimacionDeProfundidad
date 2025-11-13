# src/visualizer.py
"""
Visualización de nubes de puntos y mapas de profundidad
"""

import cv2
import numpy as np


class Simple2DVisualizer:
    """Visualización 2D de nube de puntos (vista desde arriba - bird's eye view)"""

    def __init__(self, canvas_size=(800, 800), scale=20):
        """
        Args:
            canvas_size: Tamaño del canvas en píxeles (width, height)
            scale: Píxeles por metro
        """
        self.canvas_size = canvas_size
        self.scale = scale
        self.canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)

        print(f"\n{'=' * 60}")
        print("INICIALIZANDO VISUALIZADOR 2D")
        print(f"{'=' * 60}")
        print(f"✓ Tamaño canvas: {canvas_size[0]}x{canvas_size[1]} px")
        print(f"✓ Escala: {scale} px/m")
        print(f"✓ Área visualizable: {canvas_size[0] / scale:.1f}x{canvas_size[1] / scale:.1f} m")
        print(f"{'=' * 60}\n")

    def update(self, pointcloud):
        """
        Actualiza visualización con nueva nube de puntos

        Args:
            pointcloud: Dict con 'points' (Nx3) y 'colors' (Nx3)

        Returns:
            canvas: Imagen con la visualización
        """
        # Limpiar canvas
        self.canvas.fill(0)

        points = pointcloud['points']
        colors = pointcloud['colors']

        # Centro del canvas
        cx, cy = self.canvas_size[0] // 2, self.canvas_size[1] // 2

        # Proyectar puntos 3D → 2D (vista desde arriba: X-Z)
        for i, (x, y, z) in enumerate(points):
            # Convertir coordenadas mundo → píxeles
            px = int(cx + x * self.scale)
            pz = int(cy - z * self.scale)  # Invertir Z para que adelante sea arriba

            # Verificar límites
            if 0 <= px < self.canvas_size[0] and 0 <= pz < self.canvas_size[1]:
                # Color del punto
                if colors is not None:
                    # RGB → BGR para OpenCV
                    color = tuple((colors[i][::-1] * 255).astype(int).tolist())
                else:
                    # Verde por defecto
                    color = (0, 255, 0)

                # Dibujar punto
                cv2.circle(self.canvas, (px, pz), 2, color, -1)

        # Dibujar ejes de referencia
        self._draw_reference_axes(cx, cy)

        # Dibujar grid
        self._draw_grid(cx, cy)

        # Agregar información
        self._draw_info(len(points))

        return self.canvas

    def _draw_reference_axes(self, cx, cy):
        """Dibuja ejes X y Z de referencia"""
        axis_length = 50

        # Eje X (rojo) - horizontal
        cv2.arrowedLine(self.canvas, (cx, cy), (cx + axis_length, cy),
                        (0, 0, 255), 2, tipLength=0.3)
        cv2.putText(self.canvas, "X", (cx + axis_length + 10, cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Eje Z (azul) - vertical (hacia arriba = adelante)
        cv2.arrowedLine(self.canvas, (cx, cy), (cx, cy - axis_length),
                        (255, 0, 0), 2, tipLength=0.3)
        cv2.putText(self.canvas, "Z", (cx + 5, cy - axis_length - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # Punto de origen
        cv2.circle(self.canvas, (cx, cy), 5, (255, 255, 255), -1)

    def _draw_grid(self, cx, cy):
        """Dibuja grid de referencia"""
        grid_spacing = self.scale  # 1 metro
        color = (50, 50, 50)

        # Líneas verticales
        for x in range(0, self.canvas_size[0], grid_spacing):
            cv2.line(self.canvas, (x, 0), (x, self.canvas_size[1]), color, 1)

        # Líneas horizontales
        for y in range(0, self.canvas_size[1], grid_spacing):
            cv2.line(self.canvas, (0, y), (self.canvas_size[0], y), color, 1)

    def _draw_info(self, n_points):
        """Dibuja información de la visualización"""
        info_panel = np.zeros((60, self.canvas_size[0], 3), dtype=np.uint8)

        info_text = f"Points: {n_points:,} | View: Top-Down (Bird's Eye) | Scale: {self.scale}px/m"
        cv2.putText(info_panel, info_text, (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Combinar con canvas
        self.canvas = np.vstack([self.canvas, info_panel])


class PointCloudVisualizer:
    """Visualización 3D de nube de puntos usando Open3D (opcional)"""

    def __init__(self):
        """Inicializa visualizador 3D con Open3D"""
        try:
            import open3d as o3d
            self.o3d = o3d
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(window_name='Point Cloud 3D', width=1024, height=768)

            # Configurar vista
            opt = self.vis.get_render_option()
            opt.point_size = 2.0
            opt.background_color = np.asarray([0.1, 0.1, 0.1])
            opt.show_coordinate_frame = True

            self.pcd = o3d.geometry.PointCloud()
            self.vis.add_geometry(self.pcd)
            self.initialized = True

            print("\n✓ Visualizador 3D (Open3D) inicializado")

        except ImportError:
            print("\n⚠ Open3D no está instalado. Visualización 3D no disponible.")
            print("  Instalación: pip install open3d")
            self.initialized = False

    def update(self, pointcloud):
        """
        Actualiza visualización 3D con nueva nube de puntos

        Args:
            pointcloud: Dict con 'points' (Nx3) y 'colors' (Nx3)
        """
        if not self.initialized:
            return

        points = pointcloud['points']
        colors = pointcloud['colors']

        # Actualizar geometría
        self.pcd.points = self.o3d.utility.Vector3dVector(points)

        if colors is not None:
            self.pcd.colors = self.o3d.utility.Vector3dVector(colors)

        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

    def close(self):
        """Cierra ventana de visualización"""
        if self.initialized:
            self.vis.destroy_window()


def visualize_depth_map(depth_map, title="Depth Map"):
    """
    Función auxiliar para visualizar mapa de profundidad

    Args:
        depth_map: Mapa de profundidad (metros)
        title: Título de la ventana
    """
    # Normalizar para visualización
    depth_vis = depth_map.copy()
    valid_mask = depth_vis > 0

    if valid_mask.any():
        min_depth = np.min(depth_vis[valid_mask])
        max_depth = np.max(depth_vis[valid_mask])

        depth_vis[valid_mask] = ((depth_vis[valid_mask] - min_depth) /
                                 (max_depth - min_depth) * 255)

    depth_vis = depth_vis.astype(np.uint8)

    # Aplicar colormap
    depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    depth_colored[~valid_mask] = 0

    # Agregar barra de escala
    h, w = depth_colored.shape[:2]
    scale_bar = np.zeros((50, w, 3), dtype=np.uint8)

    # Crear gradiente
    gradient = np.linspace(0, 255, w).astype(np.uint8)
    for i in range(w):
        scale_bar[10:40, i] = cv2.applyColorMap(
            np.array([[gradient[i]]], dtype=np.uint8),
            cv2.COLORMAP_JET
        )[0, 0]

    # Agregar etiquetas
    if valid_mask.any():
        cv2.putText(scale_bar, f"{min_depth:.1f}m", (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(scale_bar, f"{max_depth:.1f}m", (w - 60, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Combinar
    result = np.vstack([depth_colored, scale_bar])

    cv2.imshow(title, result)

    return result