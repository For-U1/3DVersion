//#ifndef PROJECTCONTROL_H
//#define PROJECTCONTROL_H
//
//#include <iostream>
//#include <stdio.h>
//#include <mutex>
//#include <filesystem>
//#include <opencv2/opencv.hpp>
//#include "dlpc_common.h"
//#include "dlpc34xx.h"
//#include "dlpc347x_internal_patterns.h"
//#include "cypress_i2c.h"
////#include "ICaptureEventHandler.h"
////#include "IDeviceOfflineEventHandler.h"
////#include "IFeatureEventHandler.h"
//#include "IGXFactory.h"
//#include "DxImageProc.h"
//
//#ifdef _WIN32
//#include <direct.h>   // For _mkdir on Windows
//#else
//#include <sys/stat.h> // For mkdir on Unix/Linux
//#endif
//namespace fs = std::filesystem;
//
//// 避免在头文件中使用 `using namespace std;`
//
//// 外部声明 mutex 对象
//extern std::mutex mutex1;
//extern std::mutex mutex2;
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
//// 外部声明全局变量
//extern uint8_t s_HorizontalPatternData[TOTAL_HORIZONTAL_PATTERNS][MAX_HEIGHT];
//extern uint8_t s_VerticalPatternData[TOTAL_VERTICAL_PATTERNS][MAX_WIDTH];
//extern DLPC34XX_INT_PAT_PatternData_s s_Patterns[TOTAL_HORIZONTAL_PATTERNS + TOTAL_VERTICAL_PATTERNS];
//extern DLPC34XX_INT_PAT_PatternSet_s s_PatternSets[NUM_PATTERN_SETS];
//extern DLPC34XX_INT_PAT_PatternOrderTableEntry_s s_PatternOrderTable[NUM_PATTERN_ORDER_TABLE_ENTRIES];
//extern uint8_t s_WriteBuffer[MAX_WRITE_CMD_PAYLOAD];
//extern uint8_t s_ReadBuffer[MAX_READ_CMD_PAYLOAD];
//extern bool s_StartProgramming;
//extern uint8_t s_FlashProgramBuffer[FLASH_WRITE_BLOCK_SIZE];
//extern uint16_t s_FlashProgramBufferPtr;
//extern FILE* s_FilePointer;
//
//// 函数声明
//void InitConnectionAndCommandLayer();
//uint32_t WriteI2C(uint16_t WriteDataLength, uint8_t* WriteData, DLPC_COMMON_CommandProtocolData_s* ProtocolData);
//uint32_t ReadI2C(uint16_t WriteDataLength, uint8_t* WriteData, uint16_t ReadDataLength, uint8_t* ReadData, DLPC_COMMON_CommandProtocolData_s* ProtocolData);
//
//// 类声明
//class CSampleCaptureEventHandler1 : public ICaptureEventHandler
//{
//public:
//    int image_count;
//    CSampleCaptureEventHandler1();
//    void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam) override;
//
//    // 设置保存路径
//    void setSavePath(const std::string& path);
//
//private:
//    std::string savePath;
//};
//
//class CSampleCaptureEventHandler2 : public ICaptureEventHandler
//{
//public:
//    int image_count;
//    CSampleCaptureEventHandler2();
//    void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam) override;
//
//    // 设置保存路径
//    void setSavePath(const std::string& path);
//
//private:
//    std::string savePath;
//};
//#endif  // PROJECTCONTROL_H
//
//
//
//
//
//
//
