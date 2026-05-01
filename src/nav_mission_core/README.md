# nav_mission_core（任务调度层）

## 一、概述
nav_mission_core 是基于 Nav2 之上的任务调度系统，用于实现：

- 多点导航（RViz2 打点）
- 分区任务执行（Zone）
- 到点自动触发搜索/视觉/操作任务
- 任务完成后返回入口点
- 再前往下一个目标点
- 仿真与实机统一架构

---

## 二、系统架构

RViz2 / Tag 输入  
→ nav_mission_core  
→ Nav2（/navigate_to_pose）  
→ cmd_vel_mode_mux  
→ /cmd_vel_selected  
→ 底盘驱动  

任务阶段：

→ /start_shelf_search  
→ 任务节点（视觉 / 搜索）  
→ /cmd_vel_search  
→ mux → /cmd_vel_selected  

---

## 三、核心接口

### 输入

- /waypoint_pose（RViz 打点）
- /start_mission（开始任务）
- /search_done（任务完成）
- /tag_zone_id（Tag 分区）

### 输出

- /control_mode（auto / search / manual）
- /cmd_vel_search（搜索控制速度）
- /mission_state（任务状态）

---

## 四、状态机

IDLE  
→ NAV_TO_ANCHOR  
→ SEARCHING  
→ RETURN_TO_ANCHOR  
→ NEXT / FINISHED  

---

## 五、使用流程

### 1 启动导航系统
ros2 launch nav_stack_core mapping_nav_with_serial_wheel_heading.launch.py

### 2 启动任务系统
ros2 launch nav_mission_core mission_stack.launch.py

### 3 RViz 打点
使用 2D Goal Pose → /waypoint_pose

### 4 开始任务
ros2 topic pub /start_mission std_msgs/msg/Empty "{}" -1

---

## 六、实机融合

替换：

- Gazebo diff_drive → rm_serial_driver
- Gazebo 雷达 → 实际雷达
- Gazebo 相机 → 实际相机

保持不变：

- /navigate_to_pose
- /cmd_vel_selected
- /cmd_vel_search
- /control_mode

---

## 七、速度控制架构

manual → /cmd_vel_manual  
auto → /cmd_vel  
search → /cmd_vel_search  

输出：

/cmd_vel_selected  

底盘只订阅：

/cmd_vel_selected  

---

## 八、视觉任务接入

订阅：

- /start_shelf_search
- 相机数据

发布：

- /cmd_vel_search
- /search_done

---

## 九、调试

查看 Nav2 状态：

ros2 lifecycle get /planner_server

查看速度：

ros2 topic echo /cmd_vel_selected

---

## 十、核心原则

nav_mission_core：任务调度  
Nav2：导航规划  
任务节点：感知与局部控制  
底盘：执行速度  
