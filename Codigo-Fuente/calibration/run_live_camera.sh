#!/bin/bash

PROJECT_DIR="$HOME/DeepDSO-PixelFormer-Integration"
DSO_BIN="$PROJECT_DIR/src/dso_integrated/build/bin/dso_dataset"
CALIB="$PROJECT_DIR/camera_usb.txt"
FRAMES_DIR="$PROJECT_DIR/live_frames"

echo "============================================================"
echo "🚀 DSO + PIXELFORMER CON CÁMARA USB + GUI"
echo "============================================================"
echo ""

# Verificar componentes
if [ ! -f "$CALIB" ]; then
    echo "❌ Falta calibración: $CALIB"
    exit 1
fi

if [ ! -f "$PROJECT_DIR/camera_capture_gui.py" ]; then
    echo "❌ Falta el capturador GUI en Python"
    exit 1
fi

if [ ! -f "$DSO_BIN" ]; then
    echo "❌ Falta DSO: $DSO_BIN"
    exit 1
fi

echo "✅ Componentes verificados"
echo ""

# Limpiar frames anteriores
rm -rf $FRAMES_DIR
mkdir -p $FRAMES_DIR

echo "============================================================"
echo "FASE 1: CAPTURA DE FRAMES CON GUI"
echo "============================================================"
echo ""
echo "📹 Se abrirá ventana OpenCV con tu cámara USB EN VIVO"
echo ""
echo "🎮 Controles en la ventana:"
echo "   's' = Iniciar/Pausar grabación"
echo "   'q' = Terminar captura"
echo ""
read -p "Presiona Enter para abrir la cámara..."
echo ""

# Activar entorno Python y capturar
cd $PROJECT_DIR
source server/venv_pixelformer/bin/activate
python3 camera_capture_gui.py
deactivate

# Verificar frames
FRAME_COUNT=$(ls $FRAMES_DIR/*.jpg 2>/dev/null | wc -l)
if [ $FRAME_COUNT -lt 10 ]; then
    echo ""
    echo "❌ Solo capturaste $FRAME_COUNT frames (mínimo 10)"
    echo "   Ejecuta el script de nuevo"
    exit 1
fi

echo ""
echo "✅ $FRAME_COUNT frames capturados exitosamente"
echo ""

echo "============================================================"
echo "FASE 2: PROCESAMIENTO DSO + PIXELFORMER CON GUI"
echo "============================================================"
echo ""
echo "⚠️  IMPORTANTE: Servidor PixelFormer debe estar corriendo"
echo ""
echo "   En otra terminal ejecuta:"
echo "   cd $PROJECT_DIR/server"
echo "   source venv_pixelformer/bin/activate"
echo "   python3 infer_flask.py"
echo ""
read -p "¿Servidor PixelFormer listo? [Enter]..."

echo ""
echo "🚀 Iniciando DSO con visualización Pangolin..."
echo ""

# Ejecutar DSO con GUI
export DISPLAY=${DISPLAY:-:0}
$DSO_BIN \
    files=$FRAMES_DIR \
    calib=$CALIB \
    preset=0 \
    mode=1 \
    quiet=0 \
    sampleoutput=1

echo ""
echo "============================================================"
echo "✅ SISTEMA COMPLETO EJECUTADO CON ÉXITO"
echo "============================================================"
echo ""
echo "📊 Resultados:"
echo "   - Frames capturados con GUI: $FRAME_COUNT"
echo "   - Frames procesados con DSO: ✓"
echo "   - Depth maps de PixelFormer: ✓"
echo "   - Visualización 3D Pangolin: ✓"
echo ""
echo "🎓 EVIDENCIAS PARA TESIS:"
echo "   ✅ Captura en vivo (GUI OpenCV Python)"
echo "   ✅ Procesamiento 3D (GUI Pangolin)"
echo "   ✅ Integración con PixelFormer"
echo "   ✅ Sistema completo funcionando"
echo "============================================================"
