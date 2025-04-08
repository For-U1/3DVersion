//#include "ProjectControl.h"
//
//// 定义全局变量
//std::mutex mutex1;
//std::mutex mutex2;
//
//// const std::string leftImagePath = "D:/PCLtset/CameraDlp4710/lmage/L";
//// const std::string rightImagePath = "D:/PCLtset/CameraDlp4710/lmage/R";
//
//// 宏定义
//#define MAX_WIDTH                         DLP2010_WIDTH
//#define MAX_HEIGHT                        DLP2010_HEIGHT
//
//#define NUM_PATTERN_SETS                  4
//#define NUM_PATTERN_ORDER_TABLE_ENTRIES   4
//#define NUM_ONE_BIT_HORIZONTAL_PATTERNS   4
//#define NUM_EIGHT_BIT_HORIZONTAL_PATTERNS 4
//#define NUM_ONE_BIT_VERTICAL_PATTERNS     4
//#define NUM_EIGHT_BIT_VERTICAL_PATTERNS   4
//#define TOTAL_HORIZONTAL_PATTERNS         (NUM_ONE_BIT_HORIZONTAL_PATTERNS + NUM_EIGHT_BIT_HORIZONTAL_PATTERNS)
//#define TOTAL_VERTICAL_PATTERNS           (NUM_ONE_BIT_VERTICAL_PATTERNS + NUM_EIGHT_BIT_VERTICAL_PATTERNS)
//
//#define FLASH_WRITE_BLOCK_SIZE            1024
//#define FLASH_READ_BLOCK_SIZE             256
//
//#define MAX_WRITE_CMD_PAYLOAD             (FLASH_WRITE_BLOCK_SIZE + 8)
//#define MAX_READ_CMD_PAYLOAD              (FLASH_READ_BLOCK_SIZE  + 8)
//
//uint8_t s_HorizontalPatternData[TOTAL_HORIZONTAL_PATTERNS][MAX_HEIGHT];
//uint8_t s_VerticalPatternData[TOTAL_VERTICAL_PATTERNS][MAX_WIDTH];
//DLPC34XX_INT_PAT_PatternData_s s_Patterns[TOTAL_HORIZONTAL_PATTERNS + TOTAL_VERTICAL_PATTERNS];
//DLPC34XX_INT_PAT_PatternSet_s s_PatternSets[NUM_PATTERN_SETS];
//DLPC34XX_INT_PAT_PatternOrderTableEntry_s s_PatternOrderTable[NUM_PATTERN_ORDER_TABLE_ENTRIES];
//uint8_t s_WriteBuffer[MAX_WRITE_CMD_PAYLOAD];
//uint8_t s_ReadBuffer[MAX_READ_CMD_PAYLOAD];
//bool s_StartProgramming = false;
//uint8_t s_FlashProgramBuffer[FLASH_WRITE_BLOCK_SIZE];
//uint16_t s_FlashProgramBufferPtr = 0;
//FILE* s_FilePointer = nullptr;
//
//// 函数实现
//uint32_t WriteI2C(uint16_t WriteDataLength, uint8_t* WriteData, DLPC_COMMON_CommandProtocolData_s* ProtocolData)
//{
//    bool Status = CYPRESS_I2C_WriteI2C(WriteDataLength, WriteData);
//    if (!Status)
//    {
//        return FAIL;
//    }
//    return SUCCESS;
//}
//
//uint32_t ReadI2C(uint16_t WriteDataLength, uint8_t* WriteData, uint16_t ReadDataLength, uint8_t* ReadData, DLPC_COMMON_CommandProtocolData_s* ProtocolData)
//{
//    bool Status = CYPRESS_I2C_WriteI2C(WriteDataLength, WriteData);
//    if (!Status)
//    {
//        return FAIL;
//    }
//
//    Status = CYPRESS_I2C_ReadI2C(ReadDataLength, ReadData);
//    if (!Status)
//    {
//        return FAIL;
//    }
//
//    return SUCCESS;
//}
//
//void InitConnectionAndCommandLayer()
//{
//    DLPC_COMMON_InitCommandLibrary(
//        s_WriteBuffer,
//        sizeof(s_WriteBuffer),
//        s_ReadBuffer,
//        sizeof(s_ReadBuffer),
//        WriteI2C,
//        ReadI2C
//    );
//
//    CYPRESS_I2C_ConnectToCyI2C();
//}
//
//// 类方法实现
//
//// CSampleCaptureEventHandler1 实现
//CSampleCaptureEventHandler1::CSampleCaptureEventHandler1() : image_count(0) {}
//
//void CSampleCaptureEventHandler1::setSavePath(const std::string& path)
//{
//    savePath = path;
//}
//
//void CSampleCaptureEventHandler1::DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam)
//{
//    std::lock_guard<std::mutex> lock(mutex1);
//    int width = objImageDataPointer->GetWidth();
//    int height = objImageDataPointer->GetHeight();
//    void* pBuffer = objImageDataPointer->GetBuffer();
//    GX_PIXEL_FORMAT_ENTRY pixelFormat = objImageDataPointer->GetPixelFormat();
//    image_count++;
//    cv::Mat img;
//
//    if (pixelFormat == GX_PIXEL_FORMAT_MONO8)
//    {
//        img = cv::Mat(height, width, CV_8UC1, pBuffer).clone();
//    }
//    else if (pixelFormat == GX_PIXEL_FORMAT_BAYER_RG8)
//    {
//        std::vector<unsigned char> rgbBuffer(width * height * 3);
//        VxInt32 status = DxRaw8toRGB24(
//            static_cast<unsigned char*>(pBuffer),
//            rgbBuffer.data(),
//            width,
//            height,
//            RAW2RGB_NEIGHBOUR,
//            DX_PIXEL_COLOR_FILTER(BAYERRG),
//            false
//        );
//        if (status != DX_OK)
//        {
//            std::cerr << "Camera 1 image conversion failed!" << std::endl;
//            return;
//        }
//        img = cv::Mat(height, width, CV_8UC3, rgbBuffer.data()).clone();
//    }
//    else
//    {
//        std::cerr << "Camera 1 unsupported pixel format!" << std::endl;
//        return;
//    }
//
//    // 确保保存目录存在
//    if (!savePath.empty()) {
//        fs::create_directories(savePath);
//    }
//    else {
//        std::cerr << "Camera 1 save path not set!" << std::endl;
//        return;
//    }
//
//    std::ostringstream fileName;
//    fileName << savePath << "/" << image_count << ".bmp";
//    if (!cv::imwrite(fileName.str(), img))
//    {
//        std::cerr << "Failed to save Camera 1 image: " << fileName.str() << std::endl;
//    }
//    else
//    {
//        std::cout << "Camera 1 image saved to:" << fileName.str() << std::endl;
//    }
//}
//
//// CSampleCaptureEventHandler2 实现
//CSampleCaptureEventHandler2::CSampleCaptureEventHandler2() : image_count(0) {}
//
//void CSampleCaptureEventHandler2::setSavePath(const std::string& path)
//{
//    savePath = path;
//}
//
//void CSampleCaptureEventHandler2::DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam)
//{
//    std::lock_guard<std::mutex> lock(mutex2);
//    int width = objImageDataPointer->GetWidth();
//    int height = objImageDataPointer->GetHeight();
//    void* pBuffer = objImageDataPointer->GetBuffer();
//    GX_PIXEL_FORMAT_ENTRY pixelFormat = objImageDataPointer->GetPixelFormat();
//    image_count++;
//    cv::Mat img;
//
//    if (pixelFormat == GX_PIXEL_FORMAT_MONO8)
//    {
//        img = cv::Mat(height, width, CV_8UC1, pBuffer).clone();
//    }
//    else if (pixelFormat == GX_PIXEL_FORMAT_BAYER_RG8)
//    {
//        std::vector<unsigned char> rgbBuffer(width * height * 3);
//        VxInt32 status = DxRaw8toRGB24(
//            static_cast<unsigned char*>(pBuffer),
//            rgbBuffer.data(),
//            width,
//            height,
//            RAW2RGB_NEIGHBOUR,
//            DX_PIXEL_COLOR_FILTER(BAYERRG),
//            false
//        );
//        if (status != DX_OK)
//        {
//            std::cerr << "Camera 2 image conversion failed!" << std::endl;
//            return;
//        }
//        img = cv::Mat(height, width, CV_8UC3, rgbBuffer.data()).clone();
//    }
//    else
//    {
//        std::cerr << "Camera 2 unsupported pixel format!" << std::endl;
//        return;
//    }
//
//    // 确保保存目录存在
//    if (!savePath.empty()) {
//        fs::create_directories(savePath);
//    }
//    else {
//        std::cerr << "Camera 2 save path not set!" << std::endl;
//        return;
//    }
//
//    std::ostringstream fileName;
//    fileName << savePath << "/" << image_count << ".bmp";
//    if (!cv::imwrite(fileName.str(), img))
//    {
//        std::cerr << "Failed to save Camera 2 image: " << fileName.str() << std::endl;
//    }
//    else
//    {
//        std::cout << "Camera 2 image saved to: " << fileName.str() << std::endl;
//    }
//}
//
