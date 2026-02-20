# 教程

## 1. 编译命令（需要支持 C++11 的编译器）

### 1) Windows 系统
```bash
mkdir build
cd build
cmake -G "Visual Studio 11 2012" ..
```
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