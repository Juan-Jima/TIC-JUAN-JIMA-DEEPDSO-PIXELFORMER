#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import numpy as np
import cv2
import torch
import torch.nn.functional as F
from PIL import Image

# Añadir PixelFormer al path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'PixelFormer'))

from pixelformer.networks.PixelFormer import PixelFormer

print("[INFO] Iniciando servidor PixelFormer...")

# Configuración
WEIGHTS_PATH = "../weights/pixelformer_nyu.pth"
INPUT_IMAGE = "../logs/test.jpg"
OUTPUT_DEPTH = "../logs/depthcrfs.txt"
DEBUG_IMAGE = "../logs/depth_debug.png"
DEVICE = "cpu" 

# Cargar modelo
print(f"[INFO] Cargando PixelFormer en {DEVICE}...")
model = PixelFormer(
    version='large07',      # Large model con window_size=7
    inv_depth=False,        # Profundidad directa (no inversa)
    pretrained=None,        # Sin backbone preentrenado
    frozen_stages=-1,       # Sin congelar capas
    min_depth=0.1,         # Profundidad mínima (metros)
    max_depth=100.0        # Profundidad máxima (metros)
)

checkpoint = torch.load(WEIGHTS_PATH, map_location=DEVICE)

# Remover prefijo "module." del state_dict (guardado con DataParallel)
state_dict = checkpoint['model']
new_state_dict = {}
for key, value in state_dict.items():
    if key.startswith('module.'):
        new_key = key[7:]  # Remover "module."
    else:
        new_key = key
    new_state_dict[new_key] = value

model.load_state_dict(new_state_dict)
model.to(DEVICE)
model.eval()
print("[INFO] Modelo cargado correctamente")

# Normalización ImageNet
mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1).to(DEVICE)
std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1).to(DEVICE)

def normalize_depth(depth, target_mean=2.4):
    """Normalización adaptativa según Ecuación 1.11"""
    # Escalar a [0, 1]
    d_min, d_max = depth.min(), depth.max()
    depth_norm = (depth - d_min) / (d_max - d_min + 1e-8)
    
    # Inverse log transform
    depth_scaled = 1.0 / (depth_norm + 1e-8)
    
    # Reescalar a target_mean
    current_mean = depth_scaled.mean()
    depth_final = depth_scaled * (target_mean / current_mean)
    
    return depth_final

print("[INFO] Servidor en modo polling...")
print(f"[INFO] Esperando imágenes en: {INPUT_IMAGE}")

while True:
    # Esperar a que aparezca test.jpg
    if not os.path.exists(INPUT_IMAGE):
        time.sleep(0.01)  # 10ms polling
        continue
    
    try:
        # Leer imagen
        img = cv2.imread(INPUT_IMAGE)
        if img is None:
            print("[WARN] Imagen corrupta, esperando...")
            time.sleep(0.01)
            continue
        
        h, w = img.shape[:2]
        
        # Convertir a RGB y tensor
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img_rgb)
        img_tensor = torch.from_numpy(np.array(img_pil)).permute(2, 0, 1).float() / 255.0
        img_tensor = img_tensor.unsqueeze(0).to(DEVICE)
        
        # Normalizar
        img_tensor = (img_tensor - mean) / std
        
        # Inferencia
        with torch.no_grad():
            depth_pred = model(img_tensor)
        
        # Redimensionar a tamaño original
        depth_pred = F.interpolate(depth_pred, size=(h, w), mode='bilinear', align_corners=True)
        depth_map = depth_pred.squeeze().cpu().numpy()
        
        # Normalización adaptativa
        depth_final = normalize_depth(depth_map, target_mean=2.4)
        
        # Guardar depth map
        np.savetxt(OUTPUT_DEPTH, depth_final, fmt='%.6f')
        
        # Crear visualización debug
        depth_vis = (depth_final - depth_final.min()) / (depth_final.max() - depth_final.min())
        depth_vis = (depth_vis * 255).astype(np.uint8)
        depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_PLASMA)
        cv2.imwrite(DEBUG_IMAGE, depth_colored)
        
        print(f"[OK] Depth generado: {depth_final.shape}, rango [{depth_final.min():.2f}, {depth_final.max():.2f}]")
        
        # Eliminar test.jpg para señalar completado
        os.remove(INPUT_IMAGE)
        
    except Exception as e:
        print(f"[ERROR] {e}")
        if os.path.exists(INPUT_IMAGE):
            os.remove(INPUT_IMAGE)
        time.sleep(0.01)
