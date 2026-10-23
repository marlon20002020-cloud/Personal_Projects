import cv2
import matplotlib.pyplot as plt

map_path = "/home/marlon/ros2_ws/src/my_maps/maps/map_a.png"

img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)

if img is None:
    raise FileNotFoundError(f"No se pudo cargar el mapa: {map_path}")

plt.figure(figsize=(8, 8))
plt.imshow(img, cmap="gray", origin="lower")
plt.title("Map Visualization")
plt.xlabel("x [pixels]")
plt.ylabel("y [pixels]")
plt.grid(False)
plt.show()

