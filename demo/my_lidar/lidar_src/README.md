# HC SDK — Python Usage (example/README.md)

注意：这是针对 Python 绑定（`sdk_py` 模块）的使用说明与示例。若需要查看 C++ 原生 SDK（源码、编译和完整参数），请参阅：
[SDK_V3.0/README.md](../../SDK_V3.0/README.md)

SDK — Python 使用说明（点云与节点字段说明）

本文件是针对 Python 绑定（`sdk_py` 模块）使用的快速参考，说明 SDK 暴露的数据结构、字段含义以及常用转换示例，便于在 `demo.py` 中直接使用。

## 暴露的主要类型

- `NodeInfo`（低级节点格式，来自 C++ `node_info` / `tsNodeInfo`）
  - `syn_quality` (int)
  - `angle_q6_checkbit` (int)
  - `distance_q2` (int)
  - `is_valid` (int)  # 1=有效，0=无效
  - `speed` (int)

- `PointCloud`（高层点云，来自 C++ `tsPointCloud`）
  - `valid` (bool)
  - `angle` (float)         # 补偿后角度（度）
  - `angle_raw` (float)     # 原始角度（度）
  - `distance` (int)        # 补偿后距离（毫米）
  - `distance_raw` (int)    # 原始距离（毫米）
  - `speed` (int)           # 电机转速（RPM）
  - `gray` (int)            # 亮度/灰度
  - `timestamp_ns` (int)    # 纳秒时间戳

## 常用转换

- 从 `NodeInfo` 转换为可读值：
```python
angle_deg = node.angle_q6_checkbit / 64.0
distance_mm = node.distance_q2 / 4.0
is_valid = bool(node.is_valid)
```

- 直接使用 `PointCloud`：
```python
angle = pc.angle        # 度
distance = pc.distance  # 毫米
ts_ns = pc.timestamp_ns
```

## Python 示例（写 CSV）
```python
with open('scan_data_nodes.csv', 'w') as f:
    f.write('angle_deg,distance_mm,is_valid,syn_quality,speed\n')
    for n in node_list:
        f.write(f"{n.angle_q6_checkbit/64.0:.4f},{n.distance_q2/4.0:.2f},{int(n.is_valid)},{n.syn_quality},{n.speed}\n")

with open('scan_data_pointcloud.csv', 'w') as f:
    f.write('angle_deg,distance_mm,valid,gray,timestamp_ns\n')
    for p in pointcloud_list:
        f.write(f"{p.angle:.4f},{p.distance},{int(p.valid)},{p.gray},{p.timestamp_ns}\n")
```

## 推荐实践
- 若项目上层算法（SLAM、避障）需要精确角度与距离，优先使用 `PointCloud`（已做补偿）。
- 若需要最大兼容性或想自己实现补偿/滤波，可使用 `NodeInfo` 并按上述公式换算。
- 使用 `GetLastErrCode()` 跟踪错误码以便更好地诊断连接或解包问题。

如果你希望，我可以把这些常用转换和 CSV 导出封装为 `sdk_py` 之上的一个小工具模块（例如 `sdk_py.helpers`），并在 `demo.py` 中直接调用。

---

## 角度与距离：单位说明（Python 用户快速参考）

结论与示例（直接适用于 `sdk_py` 绑定对象）:

- `NodeInfo`:
  - 角度（度）: `angle_deg = node.angle_q6_checkbit / 64.0`
  - 距离（mm）: `distance_mm = node.distance_q2 / 4.0`
  - 有效性: `node.isValid` 为布尔值

- `PointCloud`:
  - `dAngle` / `dAngleRaw`：度
  - `u16Dist` / `u16DistRaw`：毫米

注意：库内部使用 trig 函数（返回弧度），然后将结果乘以 `180/PI` 转为度后存储；若需要传入 `math.sin`/`math.cos`，请先用 `math.radians(angle_deg)` 转为弧度。

主要源码证据（便于查证）:

- `derive/src/base/HcData.h`：`tsPointCloud` 字段注释说明单位。
- `derive/src/base/hchead.cpp`：`getAngleFromXY` / `getAngleFromAB` 把 atan/atan2 结果转为度。
- `derive/src/base/lidar.cpp`：角度补偿处使用 `atan(...) / PI * 180` 并按 q 格式（*64、*4）存储。
- `derive/src/bindings.cpp`：Python 绑定的 `NodeInfo`/`PointCloud` 字段名与 `__repr__` 中的显示方式。

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