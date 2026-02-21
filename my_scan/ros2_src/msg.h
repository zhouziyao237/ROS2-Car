#include <vector>
#include <string>
#include "stdint.h"
#include <sensor_msgs/msg/laser_scan.hpp>

/*
#
# 测量的激光扫描角度，逆时针为正
# 设备坐标帧的0度面向前（沿着X轴方向）
#
 
Header header
float32 angle_min        # scan的开始角度 [弧度]
float32 angle_max        # scan的结束角度 [弧度]
float32 angle_increment  # 测量的角度间的距离 [弧度]
float32 time_increment   # 测量间的时间 [秒]
float32 scan_time        # 扫描间的时间 [秒]
float32 range_min        # 最小的测量距离 [米]
float32 range_max        # 最大的测量距离 [米]
float32[] ranges         # 测量的距离数据 [米] (注意: 值 < range_min 或 > range_max 应当被丢弃)
float32[] intensities    # 强度数据 [device-specific units]
*/

namespace msg
{
    // 内嵌的 Header 结构（来自 std_msgs/msg/Header.hpp）
    struct Header
    {
        builtin_interfaces::msg::Time stamp; // 采集时间（第一束光的时间戳）
        std::string frame_id;                // 坐标系名称，如 "laser_frame"
    };

    struct LaserScan
    {
        Header header; // 消息头

        float angle_min;       // 扫描起始角度 [rad]
        float angle_max;       // 扫描结束角度 [rad]
        float angle_increment; // 角度分辨率 [rad]（相邻测量点间的角度差）

        float time_increment; // 相邻测量点间的时间间隔 [s]（用于运动补偿）
        float scan_time;      // 两次扫描之间的时间间隔 [s]（即扫描周期）

        float range_min; // 最小有效距离 [m]
        float range_max; // 最大有效距离 [m]

        std::vector<float> ranges;      // 距离数据数组 [m]（长度等于扫描点数）
                                        // 无效点通常设为 +inf 或 NaN
        std::vector<float> intensities; // 强度数据数组（可选，若无则留空）
    };
} // namespace msg
