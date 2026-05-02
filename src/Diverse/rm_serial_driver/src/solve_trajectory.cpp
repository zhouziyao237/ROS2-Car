/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力

//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算

#include "rm_serial_driver/solve_trajectory.hpp"

#include <cmath>

struct tar_pos tar_position[4];  //最多只有四块装甲板
float t = 0.1f;                  // 飞行时间
struct SolveTrajectoryParams st;

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
  // 使用快速近似计算
  const float cos_angle = cosf(angle);
  const float sin_angle = sinf(angle);

  if (cos_angle < 1e-6f) {
    return 0.0f;
  }

  t = (expf(st.k * s) - 1.0f) / (st.k * v * cos_angle);

  if (t < 0.0f || t > 10.0f) {  // 添加合理的时间上限
    t = 0.0f;
    return 0.0f;
  }

  return v * sin_angle * t - 0.5f * GRAVITY * t * t;
}

/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
float completeAirResistanceModel(float s, float v, float angle) { return 0; }

/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
  if (s < 0.1f || v < 0.1f) {
    return 0.0f;
  }

  float z_temp = z;
  float angle_pitch = 0.0f;
  const float tolerance = 0.00001f;
  const int max_iterations = 10;

  // 使用牛顿迭代法
  for (int i = 0; i < max_iterations; i++) {
    angle_pitch = atan2f(z_temp, s);
    float z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);

    if (z_actual == 0.0f) {
      return 0.0f;
    }

    float dz = 0.3f * (z - z_actual);
    z_temp += dz;

    if (fabsf(dz) < tolerance) {
      break;
    }
  }

  return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
uint8_t autoSolveTrajectory(float * pitch, float * yaw, float * aim_x, float * aim_y, float * aim_z)
{
  // 线性预测优化
  const float time_delay = st.bias_time * 0.001f + t;
  st.tar_yaw += st.v_yaw * time_delay;

  // 使用栈内存而不是全局变量
  struct tar_pos positions[4];
  int idx = 0;

  // 根据装甲板类型优化计算
  if (st.armor_num == ARMOR_NUM_BALANCE) {
    // 平衡步兵只需计算两块装甲板
    for (int i = 0; i < 2; i++) {
      const float tmp_yaw = st.tar_yaw + i * PI;
      const float r = st.r1;
      positions[i] = {st.xw - r * cosf(tmp_yaw), st.yw - r * sinf(tmp_yaw), st.zw, tmp_yaw};
    }

    // 选择yaw角度差最小的装甲板
    float min_yaw_diff = fabsf(*yaw - positions[0].yaw);
    float temp_yaw_diff = fabsf(*yaw - positions[1].yaw);
    if (temp_yaw_diff < min_yaw_diff) {
      idx = 1;
    }
  } else {
    // 其他情况计算4块装甲板
    bool use_r1 = true;
    for (int i = 0; i < 4; i++) {
      float tmp_yaw = st.tar_yaw + i * PI * 0.5f;
      const float r = use_r1 ? st.r1 : st.r2;
      while (tmp_yaw > PI) tmp_yaw -= 2 * PI;
      while (tmp_yaw < -PI) tmp_yaw += 2 * PI;
      positions[i] = {
        st.xw - r * cosf(tmp_yaw), st.yw - r * sinf(tmp_yaw), use_r1 ? st.zw : st.zw + st.dz,
        tmp_yaw};
      use_r1 = !use_r1;
    }

    // 选择距离最近的装甲板
    float min = INFINITY;
    float yaw_to_shooter = atan2f(st.yw, st.xw);
    for (int i = 0; i < 4; i++) {
       if (fabsf(positions[i].yaw - yaw_to_shooter) > 0.1 * PI) {
       continue;
      }

      const float dist = positions[i].x * positions[i].x + positions[i].y * positions[i].y;
      const float yaw_diff = fabsf(*yaw - positions[i].yaw);
      if (dist * 0.5 + yaw_diff * 0.5 < min) {
        min = dist * 0.5 + yaw_diff * 0.5;
        idx = i;
      }
    }
    if (min == INFINITY) {
      return 0;
    }
  }

  // 计算预测位置
  *aim_z = positions[idx].z + st.vzw * time_delay;
  *aim_x = positions[idx].x + st.vxw * time_delay;
  *aim_y = positions[idx].y + st.vyw * time_delay;

  // 计算pitch和yaw
  const float temp_pitch = -pitchTrajectoryCompensation(
    sqrtf((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias, *aim_z + st.z_bias, st.current_v);

  if (temp_pitch != 0.0f) {
    *pitch = temp_pitch;
  }

  if (*aim_x != 0.0f || *aim_y != 0.0f) {
    *yaw = atan2f(*aim_y, *aim_x);
  }


  // 关闭弹道结算
  // // 1. 计算水平距离 (Horizontal Distance)
  // // 使用勾股定理计算水平面距离，并减去枪口水平偏移 s_bias
  // float horizontal_dist = sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias;

  // // 2. 计算垂直高度 (Vertical Height)
  // // 加上枪口垂直偏移 z_bias
  // float vertical_height = *aim_z + st.z_bias;

  // // 3. 几何解算 (Direct Aiming)
  // // 使用 atan2 直接计算几何角度，不调用 pitchTrajectoryCompensation
  // // 注意：原代码中有负号 "-"，为了保持坐标系一致，这里保留负号
  // if (horizontal_dist != 0) {
  //     *pitch = -(float)(atan2(vertical_height, horizontal_dist));
  // } else {
  //     *pitch = 0;
  // }

  // // Yaw 轴计算保持不变 (Yaw 通常不需要弹道补偿)
  // if (*aim_x || *aim_y) {
  //     *yaw = (float)(atan2(*aim_y, *aim_x));
  // }

  // ----------- 修改结束 -----------

  return 1;
}

// 从坐标轴正向看向原点，逆时针方向为正

// #include <cstdio>
// int main()
// {
//   float aim_x = 0, aim_y = 0, aim_z = 0;  // aim point 落点，传回上位机用于可视化
//   float pitch = 0;                        //输出控制量 pitch绝对角度 弧度
//   float yaw = 0;                          //输出控制量 yaw绝对角度 弧度

//   //定义参数
//   st.k = 0.092;
//   st.bullet_type = BULLET_17;
//   st.current_v = 18;
//   st.current_pitch = 0;
//   st.current_yaw = 0;
//   st.xw = 3.0;
//   // st.yw = 0.0159;
//   st.yw = 0;
//   // st.zw = -0.2898;
//   st.zw = 1.5;

//   st.vxw = 0;
//   st.vyw = 0;
//   st.vzw = 0;
//   st.v_yaw = PI*0.05;
//   st.tar_yaw = 0.0;
//   st.r1 = 0.5;
//   st.r2 = 0.5;
//   st.dz = 0.1;
//   st.bias_time = 0;
//   st.s_bias = 0.19133;
//   st.z_bias = 0.21265;
//   st.armor_id = ARMOR_INFANTRY3;
//   st.armor_num = ARMOR_NUM_NORMAL;

//   for (int i = 0; i < 10; i++) {
//     int x = autoSolveTrajectory(&pitch, &yaw, &aim_x, &aim_y, &aim_z);

//     printf("main pitch:%f° yaw:%f° fire:%d", pitch * 180 / PI, yaw * 180 / PI, x);
//     printf("\npitch:%frad yaw:%frad aim_x:%f aim_y:%f aim_z:%f\n", pitch, yaw, aim_x, aim_y, aim_z);
//   }

//   return 0;
// }