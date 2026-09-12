#include <cstdio>
#include <cstdint>
#include <cstring>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <csignal>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <ctime>
#include <vector>
#include "scamlib.h"

// ======================== 工具函数 ========================
static const char* format_name(CamFormat fmt) {
    switch (fmt) {
    case FORMAT_MJPG:   return "MJPG";
    case FORMAT_YUV422: return "YUV422";
    case FORMAT_RGB24:  return "RGB24";
    case FORMAT_NV12:   return "NV12";
    default:            return "UNKNOWN";
    }
}

// ======================== 全局变量 ========================
std::atomic<bool> g_running{true};          // 程序运行标志
std::mutex g_frameMutex;                   // 保护最新帧
std::mutex g_printMutex;                  // 保护终端输出
static int g_targetDevice = -1;            // 目标设备逻辑号

// 存储最新帧的深拷贝
struct StoredFrame {
    std::vector<uint8_t> buffer;
    CamData meta;
};
StoredFrame g_latestFrame;
bool g_hasFrame = false;                  // 是否已收到过帧
std::atomic<bool> g_frameUpdated{false};  // 帧更新标志，供主线程异步刷新使用
// ======================== 显示函数 ========================
// 打印当前帧信息到屏幕（动态刷新，覆盖之前内容）
static void display_frame(const StoredFrame& frame) {
    std::lock_guard<std::mutex> lock(g_printMutex);

    const CamData& meta = frame.meta;
    // 移动光标到屏幕左上角，实现覆盖刷新
    printf("\033[H");

    float fps = SCAM_GetFrameRate(g_targetDevice);
    printf("Camera %d: %s %dx%d, size=%d bytes, fps=%.2f\n",
           g_targetDevice,
           format_name(meta.format),
           meta.width, meta.height,
           meta.bufSize,
           fps);
    printf("exposure: start=%12lu us end=%12lu us, duration=%8lu us\n",
           meta.startExpouseTime, meta.endExpouseTime,
           meta.endExpouseTime - meta.startExpouseTime);

    // 打印全部11组 IMU 数据
    for (int imu_idx = 0; imu_idx < 11; ++imu_idx) {
        const sICM42688_XYZ_float& imu = meta.imu_data[imu_idx];
        printf("[g] 0x%X Acc=(%12.6f,%12.6f,%12.6f) Gyro=(%12.6f,%12.6f,%12.6f) time=%11lu us\n",
               imu_idx,
               imu.fAccData_X, imu.fAccData_Y, imu.fAccData_Z,
               imu.fGyroData_X, imu.fGyroData_Y, imu.fGyroData_Z,
               imu.uTime);
    }
    printf("\n");
    fflush(stdout);
}

// ======================== 保存函数 ========================
// 保存当前帧为 JPG 并写入文本文件
static void save_current_frame() {
    std::lock_guard<std::mutex> lock(g_frameMutex);
    if (!g_hasFrame) {
        fprintf(stderr, "\n[警告] 尚未收到任何帧，无法保存\n");
        fflush(stderr);
        return;
    }

    // 1. 使用 chrono 获取当前系统时间（精确到毫秒）
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    time_t time_now = std::chrono::system_clock::to_time_t(now);
    struct tm* tm_info = localtime(&time_now);

    // 2. 构造包含毫秒的时间戳字符串 (格式如: 20260617_205837123)
    char timeStr[64];
    size_t len = strftime(timeStr, sizeof(timeStr), "%Y%m%d_%H%M%S", tm_info);
    snprintf(timeStr + len, sizeof(timeStr) - len, "%03d", static_cast<int>(now_ms.count()));

    // 创建文件夹路径
    char dirPath[256];
    snprintf(dirPath, sizeof(dirPath), "./%s", timeStr);
    if (mkdir(dirPath, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "\n[保存] 创建目录失败: %s\n", dirPath);
        fflush(stderr);
        return;
    }

    char jpgPath[512];
    snprintf(jpgPath, sizeof(jpgPath), "%s/%s.jpg", dirPath, timeStr);

    const CamData& meta = g_latestFrame.meta;
    printf("\n[保存] 正在保存图像至 %s ...\n", jpgPath);
    bool ret = SCAM_SaveToJPG(g_latestFrame.buffer.data(), meta.format, meta.width, meta.height, jpgPath, 90);

    if (ret) {
        char txtPath[512];
        snprintf(txtPath, sizeof(txtPath), "%s/%s.txt", dirPath, timeStr);

        FILE* fp = fopen(txtPath, "w");
        if (fp) {
            fprintf(fp, "[camera_1]\n");
            fprintf(fp, "width=%d\n", meta.width);
            fprintf(fp, "height=%d\n", meta.height);
            fprintf(fp, "bufSize=%d\n", meta.bufSize);
            fprintf(fp, "format=%s\n", format_name(meta.format));
            fprintf(fp, "startExpouseTime=%lu\n", meta.startExpouseTime);
            fprintf(fp, "endExpouseTime=%lu\n", meta.endExpouseTime);
            fprintf(fp, "duration=%lu us\n", meta.endExpouseTime - meta.startExpouseTime);
            fprintf(fp, "deviceIdx=%d\n", g_targetDevice);
            fprintf(fp, "\n");

            fprintf(fp, "┌──────┬────────────┬────────────┬────────────┬────────────┬────────────┬────────────┬──────────────────┐\n");
            fprintf(fp, "│ 序号 │ 加速度(X)  │ 加速度(Y)  │ 加速度(Z)  │ 陀螺仪(X)  │ 陀螺仪(Y)  │ 陀螺仪(Z)  │ 时间戳           │\n");
            fprintf(fp, "├──────┼────────────┼────────────┼────────────┼────────────┼────────────┼────────────┼──────────────────┤\n");

            for (int imuIndex = 0; imuIndex < 11; ++imuIndex) {
                const sICM42688_XYZ_float& imu = meta.imu_data[imuIndex];
                fprintf(fp,
                        "│ %-4d │ %10.6f │ %10.6f │ %10.6f │ %10.6f │ %10.6f │ %10.6f │ %16lu │\n",
                        imuIndex + 1,
                        imu.fAccData_X, imu.fAccData_Y, imu.fAccData_Z,
                        imu.fGyroData_X, imu.fGyroData_Y, imu.fGyroData_Z,
                        imu.uTime);
                if (imuIndex != 10)
                    fprintf(fp, "├──────┼────────────┼────────────┼────────────┼────────────┼────────────┼────────────┼──────────────────┤\n");
            }
            fprintf(fp, "└──────┴────────────┴────────────┴────────────┴────────────┴────────────┴────────────┴──────────────────┘\n");
            fprintf(fp, "\n");

            fclose(fp);
            printf("[保存] 图像保存成功！\n");
            printf("[保存] 已保存文本: %s\n", txtPath);
        } else {
            printf("[保存] 图像保存成功，但创建文本失败！\n");
        }
    } else {
        printf("[保存] 保存失败: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
    }
    fflush(stdout);
}

// ======================== 按键检测线程 ========================
static void keypress_thread_func() {
    // 设置终端为 raw 模式
    struct termios orig_termios, raw_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    raw_termios = orig_termios;
    raw_termios.c_lflag &= ~(ECHO | ICANON);
    raw_termios.c_lflag |= ISIG;           // 保留 Ctrl+C 信号
    raw_termios.c_cc[VMIN] = 1;
    raw_termios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw_termios);

    while (g_running) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        struct timeval tv = {0, 100000};   // 100ms 超时
        int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
            char ch;
            if (read(STDIN_FILENO, &ch, 1) == 1 && ch == ' ') {
                save_current_frame();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

// ======================== 信号处理 ========================
static void signal_handler(int /*sig*/) {
    g_running = false;
    // 恢复终端（防止残留 raw 模式）
    system("stty cooked echo");
    fprintf(stderr, "\n收到终止信号，正在退出...\n");
    fflush(stderr);
}

// ======================== 回调函数 ========================
void OnFrameCaptured(const CamData* image, void* userData) {
    if (!image || !g_running) return;

    // 深拷贝数据
    {
        std::lock_guard<std::mutex> lock(g_frameMutex);
        g_latestFrame.buffer.assign(image->data, image->data + image->bufSize);
        g_latestFrame.meta = *image;
        g_latestFrame.meta.data = g_latestFrame.buffer.data();
        g_hasFrame = true;
        g_frameUpdated.store(true);
    }
}
int main() {
    // 注册信号
    std::signal(SIGINT, signal_handler);

    // 清屏并初始化显示
    printf("\033[2J\033[H");
    printf("--- SCAM 单路实时采集演示 (按空格保存当前帧) ---\n");
    printf("SDK 版本: %s\n", SCAM_GetVersion());
    fflush(stdout);

    // 1. 初始化 SDK
    if (!SCAM_Initialize()) {
        fprintf(stderr, "错误: SCAM_Initialize 失败: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
        return 1;
    }

    // 2. 枚举设备
    DeviceInfo devices[10];
    memset(devices, 0, sizeof(devices));
    int count = 0;
    if (!SCAM_EnumDevices(devices, 10, &count)) {
        fprintf(stderr, "错误: SCAM_EnumDevices 失败: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
        SCAM_Release();
        return 1;
    }

    g_targetDevice = -1;
    char targetVidPid[32] = {0};   // 新增：保存目标设备的 VIDPID

    printf("检测到 %d 个设备:\n", count);
    for (int i = 0; i < count; ++i) {
        if (devices[i].isExist) {
            printf("  设备逻辑号 %d: %s, VIDPID: %s\n", devices[i].number, devices[i].name, devices[i].vidpid);
            if (g_targetDevice == -1) {
                g_targetDevice = devices[i].number;
                // 新增：复制 VIDPID
                strncpy(targetVidPid, devices[i].vidpid, sizeof(targetVidPid) - 1);
                targetVidPid[sizeof(targetVidPid) - 1] = '\0';
            }
        }
    }
    if (g_targetDevice == -1) {
        fprintf(stderr, "错误: 未找到可用相机\n");
        SCAM_Release();
        return 1;
    }
    printf("选择设备逻辑号: %d\n", g_targetDevice);

    fflush(stdout);

    // 3. 获取并设置格式
    FormatInfo formats[100];
    int formatCount = 0;
    if (!SCAM_GetDeviceFormats(g_targetDevice, formats, 100, &formatCount)) {
        fprintf(stderr, "错误: SCAM_GetDeviceFormats 失败: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
        SCAM_Release();
        return 1;
    }
    printf("支持的格式:\n");
    for (int i = 0; i < formatCount; ++i) {
        printf("  [%d] %s %dx%d %dfps\n", i, format_name(formats[i].fmt),
               formats[i].width, formats[i].height, formats[i].fps);
    }
    if (formatCount == 0) {
        fprintf(stderr, "错误: 无可用格式\n");
        SCAM_Release();
        return 1;
    }
    // 选择第一个格式
    int fmtIdx = 0;
    if (!SCAM_SetDeviceFormat(g_targetDevice, fmtIdx)) {
        fprintf(stderr, "错误: SCAM_SetDeviceFormat 失败: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
        SCAM_Release();
        return 1;
    }
    printf("已设置格式索引 %d (%dx%d)\n", fmtIdx, formats[fmtIdx].width, formats[fmtIdx].height);

    // 4. 设置输出图像格式（NV12）
    SCAM_SetImageFormat(g_targetDevice, FORMAT_NV12);

    // 5. 打开设备并注册回调
    printf("\033[2J\033[H");
    if (!SCAM_OpenDevice(g_targetDevice, OnFrameCaptured, nullptr)) {
        fprintf(stderr, "错误: SCAM_OpenDevice 失败: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
        SCAM_Release();
        return 1;
    }
    printf("相机已打开，开始接收帧...\n");
    fflush(stdout);

    // 6. 启动按键检测线程
    std::thread keyThread(keypress_thread_func);

    // 7. 主循环等待退出信号并刷新终端
    while (g_running) {
        bool expected = true;
        if (g_frameUpdated.compare_exchange_strong(expected, false)) {
            StoredFrame localFrame;
            {
                std::lock_guard<std::mutex> lock(g_frameMutex);
                localFrame = g_latestFrame;
            }
            display_frame(localFrame);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 8. 等待按键线程结束
    keyThread.join();

    // 9. 关闭设备并释放资源
    printf("\n正在关闭设备...\n");
    SCAM_CloseDevice(g_targetDevice);
    SCAM_Release();
    printf("程序正常退出。\n");
    return 0;
}
