#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

int main(int argc, char** argv) {
    // Configuración
    std::string output_dir = "live_frames";
    int camera_id = 0;
    
    if (argc > 1) camera_id = std::stoi(argv[1]);
    if (argc > 2) output_dir = argv[2];
    
    // Crear directorio
    system(("mkdir -p " + output_dir).c_str());
    
    // Abrir cámara
    cv::VideoCapture cap(camera_id);
    if (!cap.isOpened()) {
        std::cerr << "❌ Error: No se pudo abrir cámara " << camera_id << std::endl;
        return -1;
    }
    
    // Configurar resolución
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    
    int w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    
    std::cout << "============================================================" << std::endl;
    std::cout << "📹 CAPTURA EN VIVO - CÁMARA USB" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "\n📊 Configuración:" << std::endl;
    std::cout << "   Cámara: " << camera_id << std::endl;
    std::cout << "   Resolución: " << w << "x" << h << std::endl;
    std::cout << "   Directorio: " << output_dir << std::endl;
    std::cout << "\n🎮 Controles:" << std::endl;
    std::cout << "   's' o ESPACIO = Iniciar/Pausar grabación" << std::endl;
    std::cout << "   'q' o ESC     = Terminar y procesar" << std::endl;
    std::cout << "\n============================================================\n" << std::endl;
    
    cv::Mat frame;
    int frame_count = 0;
    bool capturing = false;
    
    // Crear archivo de timestamps
    std::ofstream times_file(output_dir + "/times.txt");
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        
        // Crear display
        cv::Mat display = frame.clone();
        
        // Información en pantalla
        if (capturing) {
            cv::putText(display, "● GRABANDO", cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        } else {
            cv::putText(display, "⏸ PAUSADO - Presiona 's' para grabar", cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        }
        
        cv::putText(display, "Frames: " + std::to_string(frame_count), 
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                   cv::Scalar(255, 255, 255), 2);
        
        cv::putText(display, "ESC = Terminar", 
                   cv::Point(10, display.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   cv::Scalar(200, 200, 200), 1);
        
        cv::imshow("Camera Feed - DSO Capture", display);
        
        // Guardar si está capturando
        if (capturing) {
            std::ostringstream filename;
            filename << output_dir << "/" 
                    << std::setw(6) << std::setfill('0') << frame_count 
                    << ".jpg";
            cv::imwrite(filename.str(), frame);
            
            // Guardar timestamp
            auto now = std::chrono::high_resolution_clock::now();
            double timestamp = std::chrono::duration<double>(now - start_time).count();
            times_file << timestamp << " " << filename.str() << std::endl;
            
            frame_count++;
        }
        
        // Control
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) {  // q o ESC
            break;
        }
        if (key == 's' || key == 32) {  // s o ESPACIO
            capturing = !capturing;
            if (capturing) {
                std::cout << "▶️  GRABACIÓN INICIADA..." << std::endl;
                start_time = std::chrono::high_resolution_clock::now();
                frame_count = 0;
            } else {
                std::cout << "⏸️  GRABACIÓN PAUSADA (Total: " << frame_count << " frames)" << std::endl;
            }
        }
    }
    
    times_file.close();
    cap.release();
    cv::destroyAllWindows();
    
    std::cout << "\n============================================================" << std::endl;
    std::cout << "✅ Captura finalizada: " << frame_count << " frames" << std::endl;
    std::cout << "============================================================\n" << std::endl;
    return 0;
}
