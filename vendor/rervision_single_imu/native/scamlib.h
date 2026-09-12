#ifndef SCAM_SDK_H
#define SCAM_SDK_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#if defined(_MSC_VER)
#  ifdef SCAM_LIB_LIB
#    define SCAM_API __declspec(dllexport)
#  else
#    define SCAM_API __declspec(dllimport)
#  endif
#else
#  define SCAM_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 数据结构定义 ==================== */
#pragma pack(push,8)
typedef enum CamFormat {
    FORMAT_UNKNOWN = 0,
    FORMAT_MJPG = 1,
    FORMAT_RGB24 = 2,
    FORMAT_YUV422 = 3,
    FORMAT_NV12 = 4
}CamFormat;

typedef struct DeviceInfo {
    char name[256];
    char vidpid[16];
    uint32_t number;
    bool isExist;
    bool isOpened;
}DeviceInfo;
typedef struct {
    uint64_t  uTime;
    float     fAccData_X;
    float     fAccData_Y;
    float     fAccData_Z;
    float     fGyroData_X;
    float     fGyroData_Y;
    float     fGyroData_Z;
}sICM42688_XYZ_float;
typedef struct sAK09940_XYZ_int {
    uint64_t	uTime;
    int32_t		iX;
    int32_t		iY;
    int32_t		iZ;
    float		Temp;
    int32_t		iStatusBit;
}sAK09940_XYZ_int;
typedef struct FormatInfo {
    CamFormat fmt;
    int width;
    int height;
    int fps;
}FormatInfo;
typedef struct CamData {
    uint8_t* data;
    int width;
    int height;
    int bufSize;
    CamFormat format;
    uint64_t startExpouseTime;
    uint64_t endExpouseTime;
    sICM42688_XYZ_float imu_data[11];
    sAK09940_XYZ_int mtt_data[5];

}CamData;
#pragma pack(pop)
typedef void (*CamCallback)(const CamData* image, void* userData);

/* ==================== SDK 接口 ==================== */
SCAM_API int SCAM_Initialize();
SCAM_API void SCAM_Release();
SCAM_API const char* SCAM_GetVersion();
SCAM_API const int SCAM_GetLastError();
SCAM_API const char* SCAM_GetErrorText(int errorCode);

SCAM_API int SCAM_EnumDevices(DeviceInfo* devices, int maxCount, int* count);
SCAM_API int SCAM_OpenDevice(uint32_t deviceIndex, CamCallback callback=NULL, void* userData=NULL);
SCAM_API int SCAM_CloseDevice(uint32_t deviceIndex);
SCAM_API int SCAM_GetDeviceFormats(uint32_t deviceIndex, FormatInfo* formats, int maxCount, int* count);
SCAM_API int SCAM_SetDeviceFormat(uint32_t deviceIndex, int formatIndex);
SCAM_API int SCAM_GetDeviceFormatIndex(uint32_t deviceIndex);

SCAM_API void SCAM_SetImageFormat(uint32_t devIndex,CamFormat format);
SCAM_API bool SCAM_SaveToJPG(const uint8_t* buf, CamFormat fmt,int w, int h, const char* filename, int quality = 85);
SCAM_API float SCAM_GetFrameRate(uint32_t deviceIndex);
#ifdef __cplusplus
}
#endif

#endif // SCAM_SDK_H
