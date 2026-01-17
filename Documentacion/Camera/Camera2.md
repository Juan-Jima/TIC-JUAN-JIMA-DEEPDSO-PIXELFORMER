```markdown
# Informe Técnico Final de Implementación y Optimización  
**Sistema de SLAM Denso en Tiempo Real (DeepDSO + MiDaS)**  
**Proyecto:** Reconstrucción 3D Monocular en Tiempo Real mediante DeepDSO y Estimación de Profundidad Profunda  
**Autor:** Juan José Jima Estrada  
**Fecha:** 2 de diciembre de 2025  
**Estado del Sistema:** OPERATIVO Y ESTABILIZADO  

---

## 1. Resumen Ejecutivo

El presente informe documenta el proceso completo de ingeniería inversa, depuración, optimización y estabilización del sistema **DeepDSO** para su funcionamiento en tiempo real con cámara web en entornos de interiores.  

El sistema original, diseñado para datasets estáticos de exteriores (KITTI, Cityscapes), presentaba fallos críticos en condiciones reales: desbordamiento de memoria por acumulación masiva de puntos, corrupción visual, incoherencia de escala de profundidad y race conditions en la comunicación inter-proceso.

Tras múltiples iteraciones de diagnóstico y corrección, se logró una versión **estable, robusta y reproducible** mediante una arquitectura híbrida C++/Python, integración del modelo **MiDaS (Intel-ISL)** optimizado para interiores, límites estrictos de memoria, escritura atómica de archivos y preprocesamiento avanzado de imagen (CLAHE).

---

## 2. Arquitectura del Sistema y Rutas de Trabajo

### 2.1 Estructura de Directorios
```
~/cnn-dso/DeepDSO/
├── build/                  ← Binarios C++, puente de archivos (test.jpg, depthcrfs.txt)
├── src/                    ← Código fuente modificado (main_live.cpp, FullSystem.cpp)
├── newcrfs/                ← Servidor de inferencia Python (infer_flask_pixelformer.py)
└── ...
```

### 2.2 Flujo de Datos (Productor-Consumidor Asíncrono)
1. **C++ (Productor)** → Captura frame → Aplica CLAHE → Escribe `test.jpg`
2. **Python (Consumidor)** → Detecta imagen → Infiere profundidad con MiDaS → Normaliza escala (promedio 1.5 m) → Escribe `depthcrfs.txt` (escritura atómica)
3. **C++** → Lee profundidad → Inyecta en DSO → Realiza tracking y mapeo denso → Visualiza en Pangolin

---

## 3. Desafíos Técnicos Resueltos

| # | Problema Detectado                          | Causa Raíz                                      | Solución Implementada                                      |
|---|---------------------------------------------|-------------------------------------------------|------------------------------------------------------------|
| 1 | Imagen dividida/corrupta en Pangolin        | Stride misalignment al usar `memcpy` directo   | Copiado fila por fila (`for y memcpy`)                     |
| 2 | Segmentation Fault tras 20-160 frames       | Acumulación ilimitada de puntos (>100k)         | **Hard Cap de 2000 puntos** en inicialización              |
| 3 | Tracking inestable y reinicios constantes   | Escala de profundidad incoherente (modelo entrenado en exteriores) | Migración a **MiDaS Small** + normalización dinámica a 1.5 m promedio |
| 4 | Race conditions y archivos corruptos        | Escritura/lectura simultánea                    | **Escritura atómica** (`.tmp` → `rename`)                   |
| 5 | Pérdida de tracking por contraluz           | Baja contraste en zonas oscuras                 | Aplicación de **CLAHE** antes del copiado de imagen        |

---

## 4. Código Fuente Final (Versión Estable)

### 4.1 Servidor de Profundidad – `infer_flask_pixelformer.py`
**Ubicación:** `~/cnn-dso/DeepDSO/newcrfs/infer_flask_pixelformer.py`

```python
import torch
import cv2
import os
import time
import numpy as np

# CONFIGURACIÓN
BUILD_PATH = '/home/lasinac/cnn-dso/DeepDSO/build'
print(f"--- SISTEMA MiDaS (Optimizado para Interiores) ---")
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Modelo ligero y rápido
model_type = "MiDaS_small"
model = torch.hub.load("intel-isl/MiDaS", model_type)
model.to(device)
model.eval()

midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
transform = midas_transforms.small_transform

while True:
    try:
        image_path = os.path.join(BUILD_PATH, 'test.jpg')
        out_path = os.path.join(BUILD_PATH, 'depthcrfs.txt')
        
        if os.path.exists(image_path) and not os.path.exists(out_path):
            time.sleep(0.01)
            img = cv2.imread(image_path)
            if img is None: 
                continue
                
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            input_batch = transform(img_rgb).to(device)
            
            with torch.no_grad():
                prediction = model(input_batch)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=img_rgb.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
                
            depth_map = prediction.cpu().numpy()
            
            # NORMALIZACIÓN DE ESCALA PARA INTERIORES
            depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min() + 1e-6)
            depth_meters = 1.0 / (depth_map + 0.1)
            depth_meters = depth_meters * (1.5 / np.mean(depth_meters))
            depth_meters = np.clip(depth_meters, 0.1, 5.0)
            
            # ESCRITURA ATÓMICA
            temp_path = out_path + ".tmp"
            fs = cv2.FileStorage(temp_path, cv2.FILE_STORAGE_WRITE)
            fs.write('mat1', depth_meters.astype(np.float32))
            fs.release()
            os.rename(temp_path, out_path)
            
            print(f"[OK] Avg Depth: {np.mean(depth_meters):.2f}m")
            
            # Limpieza
            os.remove(image_path)
            
        else:
            time.sleep(0.005)
            
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(0.1)
```

### 4.2 Cliente Principal – `main_live.cpp`
**Ubicación:** `~/cnn-dso/DeepDSO/src/main_live.cpp`

```cpp
#include <stdio.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "FullSystem/FullSystem.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"

using namespace dso;
std::atomic<bool> system_running(true);
const std::string BRIDGE_PATH = "./";

bool fileExists(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

bool loadCameraParams(const std::string&) {
    fxG[0] = fyG[0] = 600.0f;
    cxG[0] = 320.0f; cyG[0] = 240.0f;
    wG[0] = 640; hG[0] = 480;
    pyrLevelsUsed = 4;
    printf(">> CALIBRACION LIVE: fx=600.0\n");
    return true;
}

float* loadGamma() {
    float* G = new float[65536];
    for(int i = 0; i < 65536; i++) G[i] = (float)i;
    return G;
}

void run_dso_thread(FullSystem* system) {
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    if(!cap.isOpened()) { system_running = false; return; }

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(4.0, cv::Size(8,8));
    cv::Mat frame, gray;
    int id = 0;

    printf("[INFO] Iniciando... MANTENGA CÁMARA ESTABLE 2s\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    while(system_running) {
        cap >> frame;
        if(frame.empty()) break;
        if(frame.cols != 640) cv::resize(frame, frame, cv::Size(640,480));

        // --- PUENTE CON PYTHON ---
        std::string depthFile = BRIDGE_PATH + "depthcrfs.txt";
        remove(depthFile.c_str());
        cv::imwrite(BRIDGE_PATH + "test.jpg", frame);

        int timeout = 150;
        while(timeout-- > 0 && !fileExists(depthFile)) usleep(10000);

        float* depthData = nullptr;
        if(fileExists(depthFile)) {
            cv::FileStorage fs(depthFile, cv::FileStorage::READ);
            cv::Mat depth_mat;
            if(fs.isOpened()) { fs["mat1"] >> depth_mat; fs.release(); }
            if(!depth_mat.empty() && depth_mat.type() == CV_32F) {
                depthData = new float[640*480];
                memcpy(depthData, depth_mat.data, 640*480*sizeof(float));
            }
            remove(depthFile.c_str());
        }

        // --- PREPROCESAMIENTO ---
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        clahe->apply(gray, gray);  // Mejora contraste

        MinimalImageB* img = new MinimalImageB(640, 480);
        for(int y = 0; y < 480; y++)
            memcpy(img->data + y*640, gray.ptr<unsigned char>(y), 640);

        ImageAndExposure* imgExp = new ImageAndExposure(640, 480, id * 0.05f);
        std::copy(img->data, img->data + 640*480, imgExp->image);

        system->addActiveFrame(imgExp, id, depthData);
        if(depthData) delete[] depthData;
        id++;
    }
    cap.release();
    if(system_running) system->blockUntilMappingIsFinished();
}

int main(int argc, char** argv) {
    printf("=== DEEP-DSO: VERSIÓN FINAL ESTABLE ===\n");

    setting_desiredPointDensity = 1200;
    setting_minGradHistAdd = 2.0f;
    setting_photometricCalibration = 0;
    setting_affineOptModeA = setting_affineOptModeB = 0;

    loadCameraParams("");
    FullSystem* system = new FullSystem("");
    system->setGammaFunction(loadGamma());
    system->linearizeOperation = true;

    IOWrap::PangolinDSOViewer* viewer = new IOWrap::PangolinDSOViewer(640, 480, false);
    system->outputWrapper.push_back(viewer);

    std::thread dsoThread(run_dso_thread, system);
    viewer->run();

    system_running = false;
    if(dsoThread.joinable()) dsoThread.join();

    delete viewer; delete system;
    return 0;
}
```

### 4.3 Modificación Clave en `FullSystem.cpp`
En `initializeFromInitializerCNN(...)` se añadió:

```cpp
int addedPoints = 0;
const int MAX_POINTS_INIT = 2000;  // ← PROTECCIÓN CRÍTICA
...
for(int i = 0; i < coarseInitializer->numPoints[0]; i++) {
    if(addedPoints >= MAX_POINTS_INIT) break;
    ...
    addedPoints++;
}
```

---

## 5. Instrucciones de Ejecución (Procedimiento Oficial)

```bash
# Terminal 1 – Servidor de Profundidad
cd ~/cnn-dso/DeepDSO/newcrfs
conda activate pixelformer
python3 infer_flask_pixelformer.py

# Terminal 2 – SLAM + Visualización
cd ~/cnn-dso/DeepDSO/build
make -j4
./bin/dso_live camera.txt
```

**Consejos para éxito en inicialización:**
- Iluminación frontal (evitar ventana detrás)
- Movimiento lateral suave de 10-15 cm apuntando a zona texturizada
- Mantener cámara estable durante los primeros 2 segundos

---

**Sistema completamente funcional, estable y reproducible.**  
Listo para captura de resultados experimentales y redacción final de tesis.

**Fin del Informe Técnico**
```

Puedes copiar todo este bloque directamente a un archivo `INFORME_TECNICO_DEEPDSO.md` y tendrás el documento oficial con historial completo del proyecto. ¡Éxito con tu defensa!
```
