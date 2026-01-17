#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>

namespace dso {

class DepthCommunicator {
public:
    static cv::Mat requestDepthMap(const cv::Mat& image, 
                                const std::string& logDir = "/home/lasinac/DeepDSO-PixelFormer-Integration/logs")
    {
        std::string inputPath = logDir + "/test.jpg";
        std::string outputPath = logDir + "/depthcrfs.txt";
        
        // 1. Guardar imagen
        if(!cv::imwrite(inputPath, image))
        {
            printf("❌ Error: No se pudo guardar imagen en %s\n", inputPath.c_str());
            return cv::Mat();
        }
        
        // 2. Esperar a que el servidor procese (máximo 10 segundos)
        int maxWait = 100; // 10 segundos (100 * 100ms)
        bool processed = false;
        
        for(int i = 0; i < maxWait; i++)
        {
            // El servidor elimina test.jpg cuando termina
            struct stat buffer;
            if(stat(inputPath.c_str(), &buffer) != 0)
            {
                processed = true;
                break;
            }
            usleep(100000); // 100ms
        }
        
        if(!processed)
        {
            printf("⏱️  Timeout esperando al servidor PixelFormer\n");
            remove(inputPath.c_str());
            return cv::Mat();
        }
        
        // 3. Leer depth map
        std::ifstream depthFile(outputPath);
        if(!depthFile.is_open())
        {
            printf("❌ Error: No se pudo leer %s\n", outputPath.c_str());
            return cv::Mat();
        }
        
        // Leer dimensiones (asumiendo que sabemos el tamaño)
        std::vector<float> depthData;
        float value;
        while(depthFile >> value)
        {
            depthData.push_back(value);
        }
        depthFile.close();
        
        if(depthData.empty())
        {
            printf("❌ Error: Depth map vacío\n");
            return cv::Mat();
        }
        
        // Determinar dimensiones (asumiendo width = cols del input)
        int width = image.cols;
        int height = depthData.size() / width;
        
        if(width * height != depthData.size())
        {
            printf("❌ Error: Dimensiones inconsistentes (%d x %d != %zu)\n", 
                   width, height, depthData.size());
            return cv::Mat();
        }
        
        cv::Mat depthMap(height, width, CV_32F, depthData.data());
        
        printf("✅ Depth map recibido: %dx%d\n", width, height);
        
        return depthMap.clone();
    }
};

} // namespace dso
