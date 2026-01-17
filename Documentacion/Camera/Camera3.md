Aquí tienes el texto completo convertido al formato Markdown correcto y optimizado para su lectura/renderizado en cualquier visor .md (GitHub, Obsidian, VS Code, etc.):

```markdown
# Informe Técnico de Implementación: Sistema SLAM Denso Híbrido (DeepDSO + PixelFormer)

**Proyecto:** Reconstrucción 3D Monocular Asistida por Estimación de Profundidad Profunda (PixelFormer)  
**Autor:** Juan José Jima Estrada  
**Fecha:** 8 de diciembre de 2025  
**Entorno:** CPU (Sin aceleración GPU), Webcam USB Genérica  
**Estado:** Configuración Sincronizada (Stop-and-Go)

---

## 1. Resumen de la Arquitectura

El sistema integra dos componentes desacoplados que se comunican mediante archivos temporales para lograr la reconstrucción 3D:

1. **Módulo de Profundidad (Python/PixelFormer):** Ejecuta una red neuronal Transformer (`PixelFormer`) entrenada en KITTI. Al carecer de GPU dedicada, este módulo tiene una latencia de ~0.8 a 1.2 segundos por imagen.
2. **Módulo de SLAM (C++/DeepDSO):** Realiza el rastreo de cámara (Tracking) y mapeo (Mapping). Se ha modificado para operar en modo **"Sincronizado por Demanda"**, esperando a que la CPU termine el procesamiento antes de capturar el siguiente cuadro, evitando así el desbordamiento de memoria y la pérdida de sincronía.

---

## 2. Configuración de Rutas y Dependencias

Basado en la auditoría del sistema de archivos del usuario, estas son las rutas críticas que conectan los módulos:

| Componente                  | Ruta en el Sistema (`lasinac@28580231D20P4`)                              |
|-----------------------------|---------------------------------------------------------------------------|
| **Código Base DSO**         | `/home/lasinac/cnn-dso/DeepDSO/`                                          |
| **Ejecutables (Build)**     | `/home/lasinac/cnn-dso/DeepDSO/build/`                                    |
| **Librería PixelFormer**    | `/home/lasinac/ticDSO/Paper20/PixelFormer/pixelformer`                    |
| **Pesos del Modelo**        | `/home/lasinac/ticDSO/Paper20/PixelFormer/checkpoints/kitti.pth`          |
| **Intercambio de Datos**    | `test.jpg` y `depthcrfs.txt` (en la carpeta `build`)                      |

---

## 3. Implementación de Software (Códigos Fuente Finales)

### 3.1 Servidor de Profundidad: `infer_flask_pixelformer.py`

**Ubicación:** `~/cnn-dso/DeepDSO/newcrfs/infer_flask_pixelformer.py`  
**Función:** Carga el modelo en CPU, normaliza la escala para objetos cercanos (escritorio/teclado) y gestiona la latencia.

```python
import torch
import cv2
import os
import time
import numpy as np
import sys

# --- 1. VINCULACIÓN DE LIBRERÍAS ---
REPO_ROOT = '/home/lasinac/ticDSO/Paper20/PixelFormer/pixelformer'
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

# Rutas de trabajo
CHECKPOINT_PATH = '/home/lasinac/ticDSO/Paper20/PixelFormer/checkpoints/kitti.pth'
BUILD_PATH = '/home/lasinac/cnn-dso/DeepDSO/build'

print(f"--- SISTEMA PIXELFORMER: CPU MODE (Escala Escritorio) ---")
device = torch.device("cpu")  # Forzado a CPU para estabilidad

# --- 2. CARGA DEL MODELO ---
try:
    from networks.PixelFormer import PixelFormer
    # Instancia del modelo (Configuración Large07 estándar)
    model = PixelFormer(version='large07', inv_depth=False, max_depth=10)
    
    # Carga de pesos con mapeo a CPU y limpieza de prefijos 'module.'
    if os.path.exists(CHECKPOINT_PATH):
        checkpoint = torch.load(CHECKPOINT_PATH, map_location=device)
        state_dict = checkpoint['model']
        new_state_dict = {k.replace("module.", ""): v for k, v in state_dict.items()}
        model.load_state_dict(new_state_dict)
        model.to(device)
        model.eval()
        print("[INIT] Modelo cargado correctamente en CPU.")
        MODEL_READY = True
    else:
        print(f"[ERROR] No se encontró el archivo .pth en: {CHECKPOINT_PATH}")
        MODEL_READY = False
except Exception as e:
    print(f"[ERROR CRÍTICO] Fallo al cargar PixelFormer: {e}")
    MODEL_READY = False

# --- 3. BUCLE DE INFERENCIA ---
while True:
    try:
        image_path = os.path.join(BUILD_PATH, 'test.jpg')
        out_path = os.path.join(BUILD_PATH, 'depthcrfs.txt')
        
        # Esperar a que C++ solicite profundidad
        if os.path.exists(image_path) and not os.path.exists(out_path):
            time.sleep(0.05)  # Pequeña pausa de escritura
            img = cv2.imread(image_path)
            if img is None: continue
            
            # Preprocesamiento (Normalización específica de PixelFormer)
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            input_image = img_rgb.astype(np.float32)
            input_image[:, :, 0] = (input_image[:, :, 0] - 123.68) * 0.017
            input_image[:, :, 1] = (input_image[:, :, 1] - 116.78) * 0.017
            input_image[:, :, 2] = (input_image[:, :, 2] - 103.94) * 0.017
            
            input_tensor = torch.from_numpy(np.transpose(input_image, (2, 0, 1))).unsqueeze(0).to(device)
            
            with torch.no_grad():
                if MODEL_READY:
                    depth_est = model(input_tensor)
                    depth_map = depth_est.squeeze().cpu().numpy()
                else:
                    depth_map = np.ones((480, 640), dtype=np.float32)  # Fallback
            
            # Post-procesamiento y Ajuste de Escala para Escritorio (0.35m)
            depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min() + 1e-6)
            depth_meters = 1.0 / (depth_map + 0.1)
            # Forzamos la media a 35cm para coincidir con la distancia al teclado
            depth_meters = depth_meters * (0.35 / np.mean(depth_meters))
            depth_meters = np.clip(depth_meters, 0.1, 5.0)
            
            # Escritura Atómica (Evita lectura parcial en C++)
            temp_path = out_path + ".tmp"
            fs = cv2.FileStorage(temp_path, cv2.FILE_STORAGE_WRITE)
            fs.write('mat1', depth_meters.astype(np.float32))
            fs.release()
            os.rename(temp_path, out_path)
            
            print(f"[CPU-IA] Profundidad generada. Promedio: {np.mean(depth_meters):.2f}m")
            os.remove(image_path)  # Señal de "Listo" para C++
        
        else:
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"Error en bucle: {e}")
        time.sleep(0.1)
```

### 3.2 Cliente SLAM: `main_live.cpp`

**Ubicación:** `~/cnn-dso/DeepDSO/src/main_live.cpp`  
**Función:** Captura imágenes, vacía el buffer para evitar lag, sincroniza con Python y visualiza en Pangolin.  
**Calibración:** `fx=519.0` (Extraído de `demo.py`).

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
    // CALIBRACIÓN CRÍTICA: Basada en el código fuente de PixelFormer (demo.py)
    fxG[0] = fyG[0] = 519.0f;
    cxG[0] = 320.0f; cyG[0] = 240.0f;
    wG[0] = 640; hG[0] = 480;
    pyrLevelsUsed = 4;
    printf(">> SISTEMA CALIBRADO: fx=519.0\n");
    return true;
}

float* loadGamma() {
    float* G = new float[65536];
    for(int i = 0; i < 65536; i++) G[i] = (float)i;
    return G;
}

void run_dso_thread(FullSystem* system) {
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    if(!cap.isOpened()) {
        printf("[ERROR CRITICO] Cámara no disponible.\n");
        system_running = false; return;
    }
    
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8,8));
    cv::Mat frame, gray;
    int id = 0;
    
    printf("=================================================\n");
    printf(" PROTOCOLO DE EJECUCIÓN: STOP-AND-GO (CPU) \n");
    printf(" 1. Enfoque el TECLADO. \n");
    printf(" 2. Mantenga ESTÁTICO durante 3 segundos. \n");
    printf(" 3. Mueva 1cm -> PAUSE -> Mueva 1cm -> PAUSE \n");
    printf("=================================================\n");
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    while(system_running) {
        // --- GESTIÓN DE BUFFER (SOLUCIÓN AL LAG) ---
        for(int i=0; i<5; i++) cap.grab();
        cap >> frame;
        if(frame.empty()) break;
        if(frame.cols != 640) cv::resize(frame, frame, cv::Size(640,480));
        
        // Comunicación con Python
        std::string depthFile = BRIDGE_PATH + "depthcrfs.txt";
        remove(depthFile.c_str());
        cv::imwrite(BRIDGE_PATH + "test.jpg", frame);
        
        // Espera adaptable (Hasta 3.5s para permitir inferencia en CPU)
        int timeout = 350;
        while(timeout-- > 0 && !fileExists(depthFile)) usleep(10000);
        
        float* depthData = nullptr;
        if(fileExists(depthFile)) {
            cv::FileStorage fs(depthFile, cv::FileStorage::READ);
            cv::Mat depth_mat;
            if(fs.isOpened()) { fs["mat1"] >> depth_mat; fs.release(); }
            if(!depth_mat.empty() && depth_mat.cols == 640) {
                depthData = new float[640*480];
                memcpy(depthData, depth_mat.data, 640*480*sizeof(float));
            }
            remove(depthFile.c_str());
        } else {
            printf("[LAG] Sincronizando...\n");
            continue;
        }
        
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        clahe->apply(gray, gray);
        
        MinimalImageB* img = new MinimalImageB(640, 480);
        for(int y = 0; y < 480; y++)
            memcpy(img->data + y*640, gray.ptr<unsigned char>(y), 640);
        
        ImageAndExposure* imgExp = new ImageAndExposure(640, 480, id * 0.5f);
        std::copy(img->data, img->data + 640*480, imgExp->image);
        
        try {
            system->addActiveFrame(imgExp, id, depthData);
        } catch(...) {}
        
        if(depthData) delete[] depthData;
        id++;
    }
    cap.release();
    if(system_running) system->blockUntilMappingIsFinished();
}

int main(int argc, char** argv) {
    // --- PARÁMETROS DE ROBUSTEZ ---
    setting_affineOptModeA = 1e5;
    setting_affineOptModeB = 1e5;
    setting_overallEnergyTHWeight = 30.0f;
    setting_outlierTH = 30.0f;
    setting_minGradHistAdd = 0.0f;
    setting_maxShiftWeightT = 1.0f;
    setting_desiredPointDensity = 2000;
    setting_photometricCalibration = 0;
    
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
    delete viewer;
    return 0;
}
```

---

## 4. Procedimiento de Ejecución (Protocolo Oficial)

### Paso 1: Compilación
```bash
cd ~/cnn-dso/DeepDSO/build
make -j4
```

### Paso 2: Ejecución del Módulo de IA (Terminal 1)
```bash
cd ~/cnn-dso/DeepDSO/newcrfs
conda activate pixelformer    # Si usas entorno conda
python3 infer_flask_pixelformer.py
```
**Verificación:** Debe imprimir `[CPU-IA] Profundidad generada...`

### Paso 3: Ejecución del Módulo SLAM (Terminal 2)
```bash
cd ~/cnn-dso/DeepDSO/build
./bin/dso_live camera.txt
```

### Paso 4: Configuración Visual en Tiempo Real (¡CRÍTICO!)
Active en el menú izquierdo de Pangolin:
- [x] **ActiveConst** (puntos activos - verde)
- [x] **AllConst** (historial de puntos - negro)
- [x] **show3D** (vista 3D)

### Paso 5: Maniobra de Inicialización (Técnica Stop-and-Go)
1. Apunte la cámara al **teclado**
2. Mantenga **inmóvil** 3 segundos
3. Desplace **1 cm** → **pare** 1 segundo
4. Repita: mover 1 cm → parar → mover 1 cm → parar

---

## 5. Solución de Errores Comunes

| Error en Consola                  | Causa                                              | Solución                                                                 |
|-----------------------------------|----------------------------------------------------|--------------------------------------------------------------------------|
| `OpenGL Error: XX (1282)`         | Tracking falló al inicio, no hay mapa              | Reinicie y use movimientos más lentos. Asegure `fx=519.0`                |
| `Segmentation fault`              | Acumulación de frames viejos                       | El código ya incluye vaciado de buffer (`cap.grab`)                      |
| Pantalla Negra                    | Visualización oculta                              | Active **ActiveConst** y **show3D** en Pangolin                          |
| `VIDEOIO ERROR: V4L2`             | Aviso inofensivo de drivers                        | **Ignorar**, el sistema funciona                                         |

**Fin del Informe Técnico**
```

¡Listo! Solo copia todo lo anterior en un archivo `Informe_Tecnico_Final.md` y tendrás el documento perfectamente formateado y listo para tu tesis o repositorio. Éxitos con la defensa! 🚀
