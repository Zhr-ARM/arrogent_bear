/**
 * @file simple_path_tracker.h
 * @brief 迷宫路径追踪器 - 专为Keil和迷宫导航设计
 * @author Generated for STM32
 * @date 2025-01-09
 * 
 * 特点：
 * - 专为迷宫导航和深度搜索设计
 * - 10cm网格单位，适合小车路径规划
 * - 高封装度API，100ms循环只需1-2行代码
 * - 支持开始/结束记录，路径点自动连接
 * - 为DFS算法优化的数据结构
 */

#ifndef SIMPLE_PATH_TRACKER_H
#define SIMPLE_PATH_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ==================== 迷宫导航配置 (可根据需求修改) ==================== */
#define MAX_MAP_POINTS_MAP      200     // 最大路径点数量 (稀疏)
#define GRID_SIZE_CM_MAP        10.0f   // 网格大小(cm)
#define MAP_SIZE_CM_MAP         200.0f  // 地图大小(cm) 200x200
#define UPDATE_INTERVAL_MS_MAP  100     // 调用更新函数的时间间隔(ms)
#define MAX_SHORTEST_PATH_MAP   50      // 最短路径最大点数
#define MAX_SPEED_CM_S_MAP      25      // 最大速度(cm/s)   

/* ==================== 内部计算常量 (一般无需修改) ==================== */
#define MAP_GRIDS_MAP           ((int16_t)(MAP_SIZE_CM_MAP / GRID_SIZE_CM_MAP)) // 地图网格数

/* ==================== 迷宫路径数据结构 ==================== */

/**
 * @brief 迷宫路径点结构体 (专为路径记录和寻路设计)
 */
typedef struct {
    int16_t grid_x;         // 网格X坐标
    int16_t grid_y;         // 网格Y坐标
    float x;                // 实际X坐标(cm) - 网格中心点
    float y;                // 实际Y坐标(cm) - 网格中心点
    uint16_t connected[4];  // 相邻点索引 (0xFFFF表示无连接)
    uint8_t connection_count; // 连接数量
} MapPoint_MAP;

/**
 * @brief 迷宫路径追踪器主结构体
 */
typedef struct {
    // 路径点数组 - 存储所有访问过的唯一网格点
    MapPoint_MAP path_points[MAX_MAP_POINTS_MAP];
    uint16_t point_count;           // 当前路径点数量
    
    // 当前状态
    float current_x;                // 当前车辆实际X坐标(cm)
    float current_y;                // 当前车辆实际Y坐标(cm)
    int16_t current_grid_x;         // 当前所在网格X
    int16_t current_grid_y;         // 当前所在网格Y
    uint16_t current_point_index;   // 当前点在path_points数组中的索引
    
    // 控制状态
    bool is_recording;              // 是否正在记录
    bool is_initialized;            // 是否已初始化
    
    // 网格访问标记 (位图)
    // 计算所需字节数: (MAP_GRIDS_MAP * MAP_GRIDS_MAP + 7) / 8
    uint8_t visited_grid[(MAP_GRIDS_MAP * MAP_GRIDS_MAP + 7) / 8];
    
    // 最短路径结果缓存
    MapPoint_MAP shortest_path[MAX_SHORTEST_PATH_MAP];  // 最短路径结果
    uint8_t shortest_path_count;    // 最短路径点数量
} PathTracker_MAP;

/* ==================== 简化的高封装度API ==================== */

/**
 * @brief 初始化路径追踪器 (只需调用一次)
 * @param tracker 追踪器指针
 */
void PathTracker_Init_MAP(PathTracker_MAP *tracker);

/**
 * @brief 开始路径记录
 * @param tracker 追踪器指针
 * @param start_x 起始X坐标(cm)
 * @param start_y 起始Y坐标(cm)
 */
void PathTracker_StartRecording_MAP(PathTracker_MAP *tracker, float start_x, float start_y);

/**
 * @brief 结束路径记录
 * @param tracker 追踪器指针
 */
void PathTracker_StopRecording_MAP(PathTracker_MAP *tracker);

/**
 * @brief 更新位置 (使用速度和航向角 - 适合您的应用)
 * @param tracker 追踪器指针
 * @param speed_cm_s 当前速度(cm/s)
 * @param yaw_degrees 当前航向角(度 0-360)
 * @return 是否创建了新的网格点 (1=新点, 0=无变化)
 */
uint8_t PathTracker_UpdateWithSpeed_MAP(PathTracker_MAP *tracker, float speed_cm_s, float yaw_degrees);

/**
 * @brief 更新位置 (在100ms循环中只需调用这一个函数)
 * @param tracker 追踪器指针
 * @param current_x 当前X坐标(cm)
 * @param current_y 当前Y坐标(cm)
 */
void PathTracker_Update_MAP(PathTracker_MAP *tracker, float current_x, float current_y);

/**
 * @brief 获取最新路径点
 * @param tracker 追踪器指针
 * @return 最新路径点指针 (NULL表示无数据)
 */
MapPoint_MAP* PathTracker_GetLatestPoint_MAP(PathTracker_MAP *tracker);

/**
 * @brief 获取所有路径点数组
 * @param tracker 追踪器指针
 * @param count 输出点数量
 * @return 路径点数组指针
 */
MapPoint_MAP* PathTracker_GetAllPoints_MAP(PathTracker_MAP *tracker, uint16_t *count);

/**
 * @brief 检查网格是否已访问过
 * @param tracker 追踪器指针
 * @param grid_x 网格X坐标
 * @param grid_y 网格Y坐标
 * @return true表示已访问
 */
bool PathTracker_IsGridVisited_MAP(PathTracker_MAP *tracker, int16_t grid_x, int16_t grid_y);

/**
 * @brief 获取指定点的相邻连接点
 * @param tracker 追踪器指针
 * @param point_index 点索引
 * @param connections 输出连接点索引数组
 * @return 连接数量
 */
uint8_t PathTracker_GetConnections_MAP(PathTracker_MAP *tracker, uint16_t point_index, uint16_t connections[4]);

/* ==================== 最短路径算法API ==================== */

/**
 * @brief 查找最短路径 (使用广度优先搜索BFS算法)
 * @param tracker 追踪器指针
 * @param target_grid_x 目标网格X坐标
 * @param target_grid_y 目标网格Y坐标
 * @return 最短路径点数量 (0表示未找到路径)
 */
uint8_t PathTracker_FindShortestPath_MAP(PathTracker_MAP *tracker, int8_t target_grid_x, int8_t target_grid_y);

/**
 * @brief 获取最短路径结果 (从内部缓存复制)
 * @param tracker 追踪器指针
 * @param path_buffer 输出路径缓存
 * @param buffer_size 缓存大小
 * @return 复制的路径点数量
 */
uint8_t PathTracker_GetShortestPath_MAP(PathTracker_MAP *tracker, MapPoint_MAP *path_buffer, uint8_t buffer_size);

/**
 * @brief 根据网格坐标查找点索引
 * @param tracker 追踪器指针
 * @param grid_x 网格X坐标
 * @param grid_y 网格Y坐标
 * @return 路径点索引 (-1表示未找到)
 */
int16_t PathTracker_FindPointByGrid_MAP(PathTracker_MAP *tracker, int8_t grid_x, int8_t grid_y);

/* ==================== 路径显示和获取API ==================== */

/**
 * @brief 获取所有历史路径点到数组（显示所有走过的路）
 * @param tracker 追踪器指针
 * @param history_buffer 历史路径数组
 * @param buffer_size 数组大小
 * @return 实际复制的路径点数量
 */
uint16_t PathTracker_GetAllHistoryPath_MAP(PathTracker_MAP *tracker, MapPoint_MAP *history_buffer, uint16_t buffer_size);

/**
 * @brief 一键获取最短路径到用户数组（专为显示设计）
 * @param tracker 追踪器指针
 * @param target_grid_x 目标网格X坐标
 * @param target_grid_y 目标网格Y坐标
 * @param shortest_road_array 用户的最短路径数组
 * @param array_size 数组大小  
 * @return 最短路径长度（0表示未找到）
 */
uint8_t PathTracker_GetShortestRoadToArray_MAP(PathTracker_MAP *tracker, 
                                               int8_t target_grid_x, int8_t target_grid_y,
                                               MapPoint_MAP *shortest_road_array, uint8_t array_size);

#endif // SIMPLE_PATH_TRACKER_H
