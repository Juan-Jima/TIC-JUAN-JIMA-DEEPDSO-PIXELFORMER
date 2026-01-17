#!/usr/bin/env python3
import cv2
import numpy as np

# Configuración del tablero (esquinas INTERNAS)
CHECKERBOARD = (9, 6)  # 9 columnas x 6 filas de esquinas
square_size = 0.025    # 25mm por cuadro (ajustar si es diferente)

# Preparar puntos 3D del mundo real
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # Puntos 3D
imgpoints = []  # Puntos 2D

# Abrir cámara
camera_id = 0
cap = cv2.VideoCapture(camera_id)

if not cap.isOpened():
    print(f"❌ No se pudo abrir cámara {camera_id}")
    exit(1)

# Configurar resolución
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("=" * 60)
print("📸 CALIBRACIÓN DE CÁMARA USB")
print("=" * 60)
print(f"\n🎯 Instrucciones:")
print("  1. Apunta la cámara al tablero en pantalla")
print("  2. Cuando veas líneas VERDES en el patrón")
print("  3. Presiona ESPACIO para capturar")
print("  4. Captura 10-20 imágenes variando:")
print("     - Distancia (cerca/lejos)")
print("     - Ángulo (recto/inclinado)")
print("     - Posición (centro/esquinas)")
print("  5. Presiona ESC cuando tengas suficientes")
print("\n" + "=" * 60 + "\n")

img_count = 0
target_count = 15

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Error leyendo frame")
        break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Buscar esquinas del tablero
    ret_corners, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD, 
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    
    display = frame.copy()
    
    # Si encuentra el patrón
    if ret_corners:
        # Dibujar esquinas
        cv2.drawChessboardCorners(display, CHECKERBOARD, corners, ret_corners)
        
        # Texto en verde
        cv2.putText(display, "PATRON DETECTADO - ESPACIO para capturar", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        # Texto en rojo
        cv2.putText(display, "Buscando patron...", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    # Info de captura
    color = (0, 255, 0) if img_count >= 10 else (255, 255, 255)
    cv2.putText(display, f"Imagenes: {img_count}/{target_count}", 
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    
    if img_count >= 10:
        cv2.putText(display, "Suficientes! ESC para calibrar", 
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    cv2.imshow('Calibracion USB Camera', display)
    
    key = cv2.waitKey(1) & 0xFF
    
    if key == 27:  # ESC
        break
    elif key == 32 and ret_corners:  # SPACE
        # Refinar esquinas
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        objpoints.append(objp)
        imgpoints.append(corners2)
        img_count += 1
        
        print(f"✅ Imagen {img_count}/{target_count} capturada")
        
        # Flash verde
        flash = display.copy()
        cv2.rectangle(flash, (0, 0), (display.shape[1], display.shape[0]), 
                     (0, 255, 0), 30)
        cv2.imshow('Calibracion USB Camera', flash)
        cv2.waitKey(200)

cap.release()
cv2.destroyAllWindows()

# Validar imágenes
if img_count < 10:
    print(f"\n❌ Necesitas al menos 10 imágenes (tienes {img_count})")
    print("   Ejecuta el script de nuevo y captura más imágenes")
    exit(1)

# CALIBRAR
print(f"\n🔧 Calibrando cámara con {img_count} imágenes...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

if not ret:
    print("❌ Calibración fallida")
    exit(1)

# Extraer parámetros
h, w = gray.shape[:2]
fx, fy = mtx[0, 0], mtx[1, 1]
cx, cy = mtx[0, 2], mtx[1, 2]
k1, k2, p1, p2, k3 = dist[0]

# Guardar en formato DSO
with open('camera_usb.txt', 'w') as f:
    f.write(f"Pinhole {fx:.6f} {fy:.6f} {cx:.6f} {cy:.6f} {k1:.6f}\n")
    f.write(f"{w} {h}\n")
    f.write("none\n")
    f.write(f"{w} {h}\n")

# Calcular error de reproyección
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error
mean_error /= len(objpoints)

# Resumen
print("\n" + "=" * 60)
print("✅ CALIBRACIÓN COMPLETADA")
print("=" * 60)
print(f"\n📁 Archivo guardado: camera_usb.txt")
print(f"\n📊 Parámetros:")
print(f"   Resolución: {w}x{h}")
print(f"   fx = {fx:.2f} px")
print(f"   fy = {fy:.2f} px")
print(f"   cx = {cx:.2f} px")
print(f"   cy = {cy:.2f} px")
print(f"   k1 = {k1:.6f}")
print(f"\n📏 Error de reproyección: {mean_error:.3f} px")
if mean_error < 0.5:
    print("   ✅ Excelente calibración!")
elif mean_error < 1.0:
    print("   ✅ Buena calibración")
else:
    print("   ⚠️  Calibración aceptable (considera recalibrar)")
print("\n" + "=" * 60 + "\n")
