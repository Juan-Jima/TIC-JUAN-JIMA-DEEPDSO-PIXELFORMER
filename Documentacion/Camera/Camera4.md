# Integración técnica de DSO con inicialización por CNN (PixelFormer) usando cámara en tiempo real

## 1. Contexto y objetivo

Este documento describe de forma **formal y técnica** el proceso realizado para integrar **Direct Sparse Odometry (DSO)** con un **inicializador de profundidad basado en redes neuronales (PixelFormer)**, ejecutándose **en tiempo real con cámara monocular**.

El objetivo final del sistema es:

- Capturar imágenes desde una **cámara real (V4L2 / OpenCV)**
- Estimar profundidad inicial mediante una **CNN (PixelFormer)**
- Inicializar y ejecutar **DSO**
- Visualizar en **Pangolin**:
  - puntos activos
  - reconstrucción 3D
  - trayectoria de la cámara
  - actualización del mapa al moverse

Este trabajo forma parte de un **desarrollo de nivel tesis**, por lo que se documentan también errores estructurales encontrados, decisiones de diseño y pendientes críticos.

---

## 2. Estructura del proyecto

### 2.1 Rutas principales

```text
/home/lasinac/
├── dso/                         # DSO original
│   └── build/
│       └── bin/
│           └── dso_dataset
│
├── cnn-dso/
│   └── DeepDSO/
│       ├── src/
│       ├── include/
│       ├── build/
│       │   └── bin/
│       │       └── dso_live
│       └── settings.h
│
├── datasets/
│   └── kitti/
│       └── camera.txt
│
└── envs/
    └── pixelformer/             # entorno con CNN
```

---

## 3. Configuración de cámara y calibración

### 3.1 Archivo de calibración (`camera.txt`)

Ruta:
```bash
/home/lasinac/datasets/kitti/camera.txt
```

Contenido usado inicialmente:
```text
718.856 718.856 607.1928 185.2157 0 0 0 0
1241 376
RadTan
1241 376
crop
1241 376
```

### 3.2 Problema identificado

- Esta calibración corresponde a **KITTI (1241×376)**
- La **cámara real entrega resoluciones distintas** (ej. 640×480)
- DSO **no reescala automáticamente**

📌 **Conclusión:** usar esta calib con cámara real provoca:
- incoherencia geométrica
- fallos en el initializer
- segmentation fault

### 3.3 Requisito obligatorio

La resolución de cámara **DEBE coincidir exactamente** con la calibración:

```cpp
if(frame.cols != wG[0] || frame.rows != hG[0]) {
    printf("[ERROR] Resolución incorrecta %dx%d\n", frame.cols, frame.rows);
    continue;
}
```

---

## 4. Captura de cámara (OpenCV + V4L2)

### 4.1 Inicialización recomendada

Archivo: `dso_live.cpp`

```cpp
cv::VideoCapture cap(0, cv::CAP_V4L2);
cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
cap.set(cv::CAP_PROP_FPS, 30);
cap.set(cv::CAP_PROP_CONVERT_RGB, false);
```

### 4.2 Validación de frame (CRÍTICO)

```cpp
cv::Mat frame;
if(!cap.read(frame) || frame.empty()) {
    printf("[ERROR] Frame vacío de cámara\n");
    continue;
}
```

Sin esta validación, el sistema entra en **segmentation fault**.

### 4.3 Conversión obligatoria a escala de grises

DSO **NO acepta imágenes RGB**:

```cpp
if(frame.channels() == 3)
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
```

---

## 5. Configuración de DSO (`settings.h`)

Ruta:
```bash
/home/lasinac/cnn-dso/DeepDSO/settings.h
```

### 5.1 Visualización (necesaria para ver puntos)

```cpp
extern bool setting_render_display3D;
extern bool setting_render_displayDepth;
extern bool setting_render_displayVideo;
```

Asegurar en inicialización:

```cpp
setting_render_display3D = true;
setting_render_displayDepth = true;
setting_render_displayVideo = true;
```

### 5.2 Parámetros relevantes

- `setting_desiredPointDensity`
- `setting_minFrames`
- `setting_maxFrames`
- `setting_photometricCalibration`

Estos controlan estabilidad y densidad del mapa.

---

## 6. Inicialización con CNN (PixelFormer)

### 6.1 Flujo esperado

1. Captura de frame
2. Inferencia CNN → mapa de profundidad
3. Conversión a estructura interna DSO
4. Inicialización geométrica
5. Activación del tracking

### 6.2 Error estructural detectado

Código problemático:

```cpp
FrameHessian *firstFrame = newFrame;
```

❌ Esto rompe el modelo de DSO, que asume:
- un **primer frame de referencia**
- un **segundo frame** para estimar geometría

📌 Usar el mismo frame como primero y segundo produce:
- tracking inválido
- corrupción de memoria
- segmentation fault diferido

### 6.3 Consecuencia observable

```text
INITIALIZE FROM INITIALIZER CNN (1010 pts)!
Segmentation fault
```

---

## 7. Pangolin (visualización)

### 7.1 Síntoma

- Ventana blanca
- Sin puntos
- Sin reconstrucción

### 7.2 Causa real

No se generan **KeyFrames**:

```cpp
makeKeyFrame(fh);
```

Esto solo ocurre si:
- tracking es válido
- initializer finaliza correctamente

Sin KeyFrames → Pangolin no tiene nada que dibujar.

---

## 8. Errores comunes observados

### 8.1 Error V4L2

```text
VIDEOIO ERROR: V4L2: setting property #38 is not supported
```

✔️ No es fatal
❌ Indica que la cámara no acepta algún parámetro

Si no se validan frames → crash

---

## 9. Estado actual del sistema

### Funciona:
- Compilación
- CNN (PixelFormer)
- Lanzamiento Pangolin

### No funciona aún:
- Inicialización correcta DSO
- Tracking estable
- Reconstrucción 3D

---

## 10. Trabajo pendiente (recomendado para tesis)

1. Crear **calibración real** de la cámara
2. Forzar resolución coherente
3. Separar claramente:
   - captura
   - CNN
   - DSO
4. Implementar inicialización CNN correcta:
   - frame 0: referencia
   - frame 1: CNN depth + motion
5. Validar memoria antes de activar tracking

---

## 11. Conclusión

El sistema no falló por un bug puntual, sino por una **ruptura del pipeline geométrico original de DSO** al integrar una CNN de profundidad sin respetar sus supuestos fundamentales.

El trabajo realizado permitió:
- identificar los puntos críticos
- aislar errores estructurales
- establecer una hoja de ruta clara para una integración correcta

Este documento sirve como **base técnica formal** para continuar el desarrollo dentro del contexto de una tesis de investigación.

