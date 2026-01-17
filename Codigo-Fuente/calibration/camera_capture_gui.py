#!/usr/bin/env python3
"""
Capturador de frames con GUI para DSO
Muestra cámara en vivo con controles visuales
"""
import cv2
import os
import time
from datetime import datetime

def main():
    # Configuración
    output_dir = "live_frames"
    camera_id = 0
    
    # Crear directorio
    os.makedirs(output_dir, exist_ok=True)
    
    # Abrir cámara
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"❌ No se pudo abrir cámara {camera_id}")
        return
    
    # Configurar
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print("=" * 60)
    print("📹 CAPTURA EN VIVO CON GUI - CÁMARA USB")
    print("=" * 60)
    print(f"\n📊 Configuración:")
    print(f"   Resolución: {w}x{h}")
    print(f"   Directorio: {output_dir}/")
    print("\n🎮 CONTROLES:")
    print("   's' o ESPACIO = Iniciar/Pausar grabación")
    print("   'q' o ESC     = Terminar")
    print("\n💡 CONSEJOS:")
    print("   - Muévete lento y suave")
    print("   - Captura al menos 50 frames")
    print("   - Evita movimientos bruscos")
    print("=" * 60 + "\n")
    
    frame_count = 0
    capturing = False
    times_file = None
    start_time = None
    
    # Crear ventana
    cv2.namedWindow('DSO Camera Capture', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('DSO Camera Capture', 800, 600)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Error leyendo frame")
            break
        
        # Crear display
        display = frame.copy()
        h_disp, w_disp = display.shape[:2]
        
        # Panel de información (fondo semi-transparente)
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w_disp, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, display, 0.3, 0, display)
        
        # Texto de estado
        if capturing:
            status_text = "● REC"
            status_color = (0, 0, 255)  # Rojo
            cv2.circle(display, (30, 35), 10, status_color, -1)
        else:
            status_text = "⏸ PAUSADO"
            status_color = (0, 255, 255)  # Amarillo
        
        cv2.putText(display, status_text, (50, 45),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, status_color, 3)
        
        # Contador de frames
        cv2.putText(display, f"Frames: {frame_count}", (50, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Instrucciones
        if not capturing:
            cv2.putText(display, "Presiona 's' para INICIAR grabacion", 
                       (50, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                       (255, 255, 255), 1)
        else:
            cv2.putText(display, "Presiona 's' para PAUSAR | 'q' para TERMINAR", 
                       (50, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                       (255, 255, 255), 1)
        
        # Indicador de suficientes frames
        if frame_count >= 50:
            cv2.putText(display, "✓ Suficientes frames!", 
                       (w_disp - 250, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                       (0, 255, 0), 2)
        
        # Mostrar
        cv2.imshow('DSO Camera Capture', display)
        
        # Guardar si está capturando
        if capturing:
            filename = os.path.join(output_dir, f"{frame_count:06d}.jpg")
            cv2.imwrite(filename, frame)
            
            # Timestamp
            if times_file:
                current_time = time.time()
                timestamp = current_time - start_time
                times_file.write(f"{timestamp:.6f} {filename}\n")
            
            frame_count += 1
        
        # Control de teclado
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q') or key == 27:  # q o ESC
            break
        elif key == ord('s') or key == 32:  # s o ESPACIO
            capturing = not capturing
            
            if capturing:
                # Iniciar captura
                if times_file is None:
                    times_file = open(os.path.join(output_dir, "times.txt"), 'w')
                    start_time = time.time()
                    frame_count = 0
                print("▶️  GRABACIÓN INICIADA")
            else:
                # Pausar
                print(f"⏸️  GRABACIÓN PAUSADA (Total: {frame_count} frames)")
    
    # Limpiar
    if times_file:
        times_file.close()
    cap.release()
    cv2.destroyAllWindows()
    
    print("\n" + "=" * 60)
    print(f"✅ Captura finalizada: {frame_count} frames guardados")
    print(f"📁 Ubicación: {output_dir}/")
    print("=" * 60 + "\n")

if __name__ == '__main__':
    main()
