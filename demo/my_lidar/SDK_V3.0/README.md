# 教程

# HC SDK — C++ Reference (SDK_V3.0/README.md)

注意：这是 C++ 原生 SDK 的参考文档（源码、编译与示例）。
若要查看 Python 绑定与使用说明，请参考示例目录中的 Python 文档：
[my_scan/ros2_src/example/README.md](my_scan/ros2_src/example/README.md)

# 教程

## 1. 编译命令（需要支持 C++11 的编译器）

### 1) Windows 系统
```bash
mkdir build
cd build
cmake -G "Visual Studio 11 2012" ..
```

## 雷达相关数据结构与字段（技术文档）

下面列出 SDK 中与激光雷达点云与节点相关的所有主要数据结构、字段含义、单位以及常用转换：

- `tsNodeInfo` / `node_info`（低级原始节点格式，通常在轮询模式下返回 `std::list<node_info>`）
    - `syn_quality` (uint16): 同步质量或信号强度（原始值，具体范围随硬件而异）
    - `angle_q6_checkbit` (uint16): 角度值，已放大 64 倍（度 * 64），转换为度：`angle_deg = angle_q6_checkbit / 64.0`
    - `distance_q2` (uint16): 距离值，已放大 4 倍（mm * 4），转换为毫米：`distance_mm = distance_q2 / 4.0`
    - `isValid` / `is_valid` (uint16): 有效标记，`1` = 有效，`0` = 无效
    - `speed` (uint16): 电机速度（RPM）

- `tsPointCloud` / `PointCloud`（高层补偿后点云格式，SDK 通过回调或点云接口返回 `LstPointCloud`）
    - `bValid` / `valid` (bool): 点是否有效
    - `dAngle` / `angle` (double): 补偿后角度（度，可直接使用）
    - `dAngleRaw` / `angle_raw` (double): 原始角度（度）
    - `u16Dist` / `distance` (uint16): 补偿后距离（毫米，可直接使用）
    - `u16DistRaw` / `distance_raw` (uint16): 原始距离（毫米）
    - `u16Speed` / `speed` (uint16): 电机转速（RPM）
    - `u16Gray` / `gray` (uint16): 亮度（灰度）
    - `u64TimeStampNs` / `timestamp_ns` (uint64): 时间戳（纳秒）
    - 备注：`PointCloud` 结构包含已做角度/距离补偿的高层信息，回调接口返回该结构更便于直接使用。

常用示例（C/C++）：
    - NodeInfo -> 度/mm
        - `double angle = node.angle_q6_checkbit / 64.0;`
        - `double dist_mm = node.distance_q2 / 4.0;`

    - PointCloud（直接使用）
        - `double angle = pc.dAngle;`
        - `uint16_t dist = pc.u16Dist;`

错误码与状态（概要）
- SDK 定义了若干 `LiDarErrorCode`，常见的如 `LIDAR_SUCCESS = 0`、`ERR_NO_DATA`、`ERR_DISCONNECTED` 等。调用接口时请同时检查 `GetLastErrCode()` 返回值以获得设备状态。

常见型号（宏定义，位于 `hchead.h`）：
- X1B, X1D, X1E, X1F, X1G, X1K, X1L, X1M, X1N, X1S
- X2A, X2B, X2C, X2D, X2E, X2F, X2M, X2N, X2BF, X2BZ, X2B2, X2DE, X2MF, X2MX, X2MM, X2YE
- D2A, D2B, D2M8, D2A8, D2B8, D2S8, D2SA, D2P8, D2PD, D2M1, D2AE
- T3B

注：不同型号在分辨率（角度分辨率）、帧率（FPS）、最大循环点数（CIRCLE_MAX）以及是否支持 TOF/灰度等方面存在差异，开发时请参考 `hchead.h` 中的分辨率与常量（例如 `ANGLE_RESOLV_2000`、`FPS_3000_NOR` 等）以及具体硬件手册。

使用建议：
- 轮询模式（`GetScanData` 返回 `NodeInfo` 列表）：适用于需要底层 q 格式进行自定义后处理或兼容旧工具链的场景。请使用上述换算公式转换为度与毫米。
- 回调/高层模式（`PointCloud`）：适用于需要即时使用补偿后角度/距离的场景，推荐用于可视化与上层算法（如建图、避障）。

如果需要，我可以将此技术文档进一步细化为每个型号的差异表（例如分辨率、最大点数、支持的帧率范围），或者把示例转换为更易用的 Python helper 函数并合并到 SDK 的示例目录中。

---

## 角度与距离：单位说明与代码证据（快速参考）

结论：SDK 在内部以“度”为角度单位，高层 `tsPointCloud` 中的 `dAngle`/`dAngleRaw` 为度；低级 `node_info` 中的 `angle_q6_checkbit` 为度乘以 64；距离在 `node_info` 中以 `distance_q2` 存为 mm*4，`
`tsPointCloud` 的 `u16Dist` 为毫米。

直接转换示例：

- C/C++：
    - `double angle_deg = node.angle_q6_checkbit / 64.0;`
    - `double dist_mm = node.distance_q2 / 4.0;`

- Python：
    - `angle_deg = node.angle_q6_checkbit / 64.0`
    - `distance_mm = node.distance_q2 / 4.0`
    - 用于 `math.sin`/`math.cos` 前转换为弧度：`angle_rad = math.radians(angle_deg)`。

重要源码位置（便于复核）:

- `derive/src/base/HcData.h`：`tsPointCloud` 字段注释（`dAngle` 为度，`u16Dist` 为 mm）。
- `derive/src/base/hchead.cpp`：`HCHead::getAngleFromXY` 与 `HCHead::getAngleFromAB` 将 atan/atan2 结果由弧度转换为度（乘以 `180/PI`）。
- `derive/src/base/lidar.cpp`：角度补偿使用 `atan(...) / PI * 180`，并把补偿后角度写入 `angle_q6_checkbit = angle * 64`，距离写入 `distance_q2 = dist * 4`。
- `derive/src/bindings.cpp`：Python 绑定中 `NodeInfo` 的 `__repr__` 以 `angle_q6_checkbit/64.0` 显示角度，证明绑定与文档一致。

建议：在上层处理链路（如 SLAM、坐标变换）中显式标注字段单位并在需要时做度↔弧度转换，避免直接把 q 格式数值当作度或弧度混用。

（注意：如果使用其他编译器，请修改 cmake 命令）

### 2) Linux 系统
```bash
mkdir build
cd build
cmake ..
make
```

## 2. 修改端口号

1. 请根据激光雷达连接的 USB 端口号修改程序中的端口号。具体请参考代码及注释。
2. （Linux 系统）修改 USB 端口号后，执行 `make` 命令重新生成所有可执行文件（*.exe 文件）。

## 3. 打印输出说明

### 1) sdk_demo.exe （输出激光雷达单次数据包）
```c
printf("%d, dist: %d, angle: %f, is_invalid: %d\n", i, dataPack[i].dist, dataPack[i].angle, dataPack[i].flag);
```
从左到右依次为：
- `i`            → 数据包索引号（0-7）
- `dist`         → 距离信息（单位：毫米）
- `angle`        → 角度信息（单位：度）
- `is_invalid`   → 当前数据包的有效性（0：有效，1：无效）

### 2) sdk_scanData.exe （输出激光雷达单圈数据包）
```c
printf("data size=%d, FD=[%.2f], LD=[%.2f], TC=%.3fms, stamp=%llu\n",
    count,
    nodebuffer[0].angle_q6_checkbit / 64.0f,
    nodebuffer[count-1].angle_q6_checkbit / 64.0f,
    timer.Duation_ms(),
    data_stamp_new);
```
从左到右依次为：
- `data size` → 单圈数据包总数（无效数据包的距离值设为 0）
- `FD`        → 单圈数据包的起始角度
- `LD`        → 单圈数据包的结束角度
- `TC`        → 单圈数据包的时间花费（单位：毫秒）
- `stamp`     → 单圈数据包的时间戳（逐次递增）

# SDK_V3.0 — 函数与接口索引（derive/src）

## 文件：base/HcSDK.h（C API / SDK 顶层接口）

- `bool hcSDKInitialize(const char* chPort, const char* chLidarModel, int iBaud, int iReadTimeoutMs, bool bDistQ2, bool bGetLoopData, bool bPollMode)`
  - 初始化 SDK 并打开串口，参数包含串口、型号、波特率、超时与模式选项。
- `bool hcSDKUnInit()`
  - 反初始化并释放资源。
- `void setSDKCallBackFunErrorCode(CallBackFunErroCode fun)`
  - 注册错误码回调。
- `void setSDKCallBackFunSecondInfo(CallBackFunSecondInfo fun)`
  - 注册统计信息（每秒）回调。
- `void setSDKCallBackFunPointCloud(CallBackFunPointCloud fun)`
  - 注册点云（补偿后）回调。
- `void setSDKCallBackFunDistQ2(CallBackFunDistQ2 fun)`
  - 注册原始距离 Q2 数据回调。
- `int getSDKLastErrCode()`
  - 获取并清除最后一次错误码。
- `bool getSDKLidarInfo()`
  - 请求并读取设备信息。
- `int getSDKStatus()`
  - 查询 SDK 当前状态（enum `enSDKStatus`）。
- `char* getSDKVersion()` / `char* getSDKLidarID()` / `char* getSDKLidarModel()` / `char* getSDKFirmwareVersion()` / `char* getSDKHardwareVersion()`
  - 获取各类描述字符串（版本、ID、型号、固件、硬件）。
- `void setSDKWorkPara(tsSDKPara& sSDKPara)`
  - 设置运行参数（超时、缓冲等，见 `tsSDKPara`）。
- `bool getSDKRxPointClouds(LstPointCloud& lstG)` / `bool getSDKScanData(std::list<tsNodeInfo>& dataList, bool bReverse = true)`
  - 轮询接口：分别获取补偿后点云列表或原始 `node_info` 列表（Q2 格式）。
- 工厂/测试：`setSDKFactoryMode`, `startSDKFactoryModeRun`, `setSDKCircleDataMode`, `setSDKLidarPowerOn`, `setSDKLidarLowSpeed`。

## 文件：base/hclidar.h（HCLidar 类 — SDK 的 C++ 核心管理类）

类 `HCLidar`（单例模式或普通构造）对外主要方法：

- `BOOL initialize(const char* chPort, const char* chLidarModel, int iBaud, int iReadTimeoutMs, bool bDistQ2,bool bGetLoopData, bool bPollMode)`
  - 初始化并启动内部线程，参数含串口/型号/模式。
- `BOOL unInit()`
  - 停止线程并释放资源。
- 回调注册：`setCallBackFunErrorCode`, `setCallBackFunSecondInfo`, `setCallBackFunPointCloud`, `setCallBackFunDistQ2`。
- `int getLastErrCode()`
  - 读取并清零最后错误码。
- 查询类信息：`getLidarInfo()`, `getSDKStatus()`, `getSDKVersion()`, `getLidarID()`, `getFactoryInfo()`, `getLidarModel()`, `getFirmwareVersion()`, `getHardwareVersion()`。
- 参数/数据接口：`setWorkPara(tsSDKPara&)`, `getRxPointClouds(LstPointCloud&)`, `getScanData(std::list<tsNodeInfo>&, bool)`。
- 工厂/调试：`setFactoryMode`, `startFactoryModeRun`, `setCircleDataMode`, `setLidarPowerOn`, `setLidarLowSpeed`。

内部私有方法（核心工作流程相关，便于定位实现）：
- `threadWork()`, `threadParse()`, `readData()`, `processMain()`, `processData()`
- 数据解析与补偿：`getPointCloud`, `parserRangeEX`, `compensate(double &angle, UINT16 &dist, const double theta_d, const double baseline_mm)`
- 各类检查函数：`checkDataCal`, `checkInvalidFPS`, `checkSharkBlocked`, `checkEncoderError` 等。

## 文件：base/lidar.h（低级打包/设备抽象 — `Dev` 类 与 `node_info`）

- 结构体 `node_info`：
  - `syn_quality`、`angle_q6_checkbit`（角度 * 64）、`distance_q2`（距离 * 4）、`isValid`、`speed`。
- 类 `Dev`（设备接口）主要方法：
  - `int Initialize(char* port, unsigned int baud, bool isGetLoopData = true)` — 打开串口并开始读取。
  - `void GetScanData(node_info * nodebuffer,size_t buffLen, size_t &count, bool isReverse = true)` — 非阻塞从设备获取 `node_info` 缓冲数据。
  - `void GetScanData(std::list<node_info>& dataList, bool isReverse = true)` — 列表形式获取数据。
  - `int Uninit()` — 关闭并清理。

内部辅助方法包括：`PushDataWithNoLoopMode`、`PushDataWithLoopMode`、`GrabScanData*`、`CheckBufferIsSorted` 等。

## 文件：base/ReadParsePackage.h（串口读取与包解析）

- 构造/析构：`ReadParsePackage()` / `~ReadParsePackage()`。
- 串口与解析接口：
  - `int OpenSerial(char* port, unsigned int baud)` — 打开串口。
  - `void CloseSerial()` — 关闭串口。
  - `int ReadDataSerial(unsigned int TimeOut_ms = 60)` — 读取原始数据到缓冲区。
  - `int ParseDataSerial(rangedata* dataPack, int& fps)` — 将缓冲解析为 `rangedata` 数组并返回 fps 信息。
  - `int GetLastErrCode()`, `void SetLastErrCode(int)`。

## 文件：base/HC_serial.h（底层串口封装）

- 类 `HC_serial` 方法：
  - `int64_t GetTimeStamp()` — 返回时间戳（内部实现平台相关）。
  - `char openDevice(const char *chPort, unsigned int iBauds)` / `void closeDevice()` — 打开/关闭串口设备。
  - 写入：`char writeChar(const char Byte)`, `int writeData2(unsigned char * pData, int iLen)`。
  - 读取：`int readChar(unsigned char *pByte,const unsigned int TimeOut_ms=0)`, `int readChars(unsigned char *Buffer, unsigned int MaxNbBytes, const unsigned int TimeOut_ms = 0)`, `int readData(unsigned char *Buffer, unsigned int expectedBytes,int iTimeoutms=10)`。
  - 缓冲/控制：`flushReceiver()`, `peekReceiver()`, `DTR(bool)`, `RTS(bool)`, `isCTS()`, `isDTR()`, `isRTS()`。

## 文件：base/hchead.h（工具函数与数据结构定义）

- 静态工具：`HCHead::getCurrentTimestampNs/Us/Ms()`。
- 缓冲/数组帮助：`eraseBuff`, `eraseRangeData`。
- 角度/距离转换：
  - `getAngleFromXY(x,y)`、`getAngleFromAB(a,b)` — 基于 atan/atan2，返回**度**。
  - `getDistFromAB(a,b)` — 计算距离。
- 类型/常量：大量 `#define`（型号常量、分辨率、FPS、ANGLE_RESOLV 等）和数据结构（`tsLidarAttr`, `tsPointCloudHead`, `tsPointCloud` 等）。

## 关键数据结构（来自 `HcData.h`）

- `tsNodeInfo`：原始节点，含 `angle_q6_checkbit`（角度 *64）、`distance_q2`（距离 *4）。
- `tsPointCloud`：补偿后点云，字段含 `dAngle`/`dAngleRaw`（度）、`u16Dist`（mm）、`u16Speed`、`u16Gray`、时间戳等。
- `tsSDKPara`：SDK 参数集合（超时/缓冲等），`tsSDKStatistic`：运行统计信息。