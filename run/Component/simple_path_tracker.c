/**
 * @file simple_path_tracker.c
 * @brief 迷宫路径追踪器实现 - 专为Keil和迷宫导航设计
 * @author Generated for STM32
 * @date 2025-01-09
 */

#include "simple_path_tracker.h"
#include <string.h>
#include <math.h>  // 添加数学函数库

/* ==================== 内部辅助函数 ==================== */

/**
 * @brief 实际坐标转换为网格坐标
 */
static void real_to_grid_MAP(float real_x, float real_y, int16_t *grid_x, int16_t *grid_y)
{
    *grid_x = (int16_t)(real_x / GRID_SIZE_CM_MAP);
    *grid_y = (int16_t)(real_y / GRID_SIZE_CM_MAP);
    
    // 确保在范围内
    if (*grid_x < 0) *grid_x = 0;
    if (*grid_x >= MAP_GRIDS_MAP) *grid_x = MAP_GRIDS_MAP - 1;
    if (*grid_y < 0) *grid_y = 0;
    if (*grid_y >= MAP_GRIDS_MAP) *grid_y = MAP_GRIDS_MAP - 1;
}

/**
 * @brief 网格坐标转换为实际坐标(网格中心点)
 */
static void grid_to_real_MAP(int16_t grid_x, int16_t grid_y, float *real_x, float *real_y)
{
    *real_x = grid_x * GRID_SIZE_CM_MAP + GRID_SIZE_CM_MAP / 2.0f;
    *real_y = grid_y * GRID_SIZE_CM_MAP + GRID_SIZE_CM_MAP / 2.0f;
}

/**
 * @brief 设置网格为已访问
 */
static void set_grid_visited_MAP(PathTracker_MAP *tracker, int16_t grid_x, int16_t grid_y)
{
    if (grid_x < 0 || grid_x >= MAP_GRIDS_MAP || grid_y < 0 || grid_y >= MAP_GRIDS_MAP) {
        return;
    }
    
    uint16_t bit_index = grid_y * MAP_GRIDS_MAP + grid_x;
    uint16_t byte_index = bit_index / 8;
    uint8_t bit_offset = bit_index % 8;
    
    if (byte_index < sizeof(tracker->visited_grid)) {
        tracker->visited_grid[byte_index] |= (1 << bit_offset);
    }
}

/**
 * @brief 检查网格是否已访问
 */
static bool is_grid_visited_MAP(PathTracker_MAP *tracker, int16_t grid_x, int16_t grid_y)
{
    if (grid_x < 0 || grid_x >= MAP_GRIDS_MAP || grid_y < 0 || grid_y >= MAP_GRIDS_MAP) {
        return false;
    }
    
    uint16_t bit_index = grid_y * MAP_GRIDS_MAP + grid_x;
    uint16_t byte_index = bit_index / 8;
    uint8_t bit_offset = bit_index % 8;
    
    if (byte_index < sizeof(tracker->visited_grid)) {
        return (tracker->visited_grid[byte_index] & (1 << bit_offset)) != 0;
    }
    
    return false;
}

/**
 * @brief 查找相同网格坐标的已有路径点
 */
static uint16_t find_point_by_grid_MAP(PathTracker_MAP *tracker, int16_t grid_x, int16_t grid_y)
{
    for (uint16_t i = 0; i < tracker->point_count; i++) {
        if (tracker->path_points[i].grid_x == grid_x && 
            tracker->path_points[i].grid_y == grid_y) {
            return i;
        }
    }
    return 0xFFFF; // 未找到
}

/**
 * @brief 添加两点之间的连接
 */
static void add_connection_MAP(PathTracker_MAP *tracker, uint16_t from_index, uint16_t to_index)
{
    if (from_index >= tracker->point_count || to_index >= tracker->point_count) {
        return;
    }
    
    MapPoint_MAP *from_point = &tracker->path_points[from_index];
    
    // 检查是否已经连接
    for (uint8_t i = 0; i < from_point->connection_count; i++) {
        if (from_point->connected[i] == to_index) {
            return; // 已经连接
        }
    }
    
    // 添加连接
    if (from_point->connection_count < 4) {
        from_point->connected[from_point->connection_count] = to_index;
        from_point->connection_count++;
    }
}

/* ==================== 公共API实现 ==================== */

/**
 * @brief 初始化路径追踪器
 */
void PathTracker_Init_MAP(PathTracker_MAP *tracker)
{
    if (tracker == NULL) return;
    
    memset(tracker, 0, sizeof(PathTracker_MAP));
    
    // 初始化所有连接为无效值
    for (uint16_t i = 0; i < MAX_MAP_POINTS_MAP; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            tracker->path_points[i].connected[j] = 0xFFFF;
        }
    }
    
    tracker->current_point_index = 0xFFFF;
    tracker->is_initialized = true;
}

/**
 * @brief 开始路径记录
 */
void PathTracker_StartRecording_MAP(PathTracker_MAP *tracker, float start_x, float start_y)
{
    if (tracker == NULL || !tracker->is_initialized) return;
    
    tracker->is_recording = true;
    tracker->current_x = start_x;
    tracker->current_y = start_y;
    
    // 转换为网格坐标
    real_to_grid_MAP(start_x, start_y, &tracker->current_grid_x, &tracker->current_grid_y);
    
    // 清空所有数据
    tracker->point_count = 0;
    memset(tracker->visited_grid, 0, sizeof(tracker->visited_grid));
    
    // 添加起始点
    if (tracker->point_count < MAX_MAP_POINTS_MAP) {
        MapPoint_MAP *point = &tracker->path_points[tracker->point_count];
        point->grid_x = tracker->current_grid_x;
        point->grid_y = tracker->current_grid_y;
        grid_to_real_MAP(point->grid_x, point->grid_y, &point->x, &point->y);
        point->connection_count = 0;
        
        for (uint8_t i = 0; i < 4; i++) {
            point->connected[i] = 0xFFFF;
        }
        
        tracker->current_point_index = tracker->point_count;
        tracker->point_count++;
        
        // 标记网格为已访问
        set_grid_visited_MAP(tracker, tracker->current_grid_x, tracker->current_grid_y);
    }
}

/**
 * @brief 结束路径记录
 */
void PathTracker_StopRecording_MAP(PathTracker_MAP *tracker)
{
    if (tracker == NULL) return;
    
    tracker->is_recording = false;
}

/**
 * @brief 更新位置 (使用速度和航向角) - 返回是否创建了新网格点
 */
uint8_t PathTracker_UpdateWithSpeed_MAP(PathTracker_MAP *tracker, float speed_cm_s, float yaw_degrees)
{
    if (tracker == NULL || !tracker->is_recording) return 0;
    
    // 记录更新前的点数量
    uint16_t previous_count = tracker->point_count;
    
    // 对速度进行限幅
    if (speed_cm_s > MAX_SPEED_CM_S_MAP) {
        speed_cm_s = MAX_SPEED_CM_S_MAP;
    } else if (speed_cm_s < -MAX_SPEED_CM_S_MAP) {
        speed_cm_s = -MAX_SPEED_CM_S_MAP;
    }
    
    // 将角度标准化到0-360度
    while (yaw_degrees < 0) yaw_degrees += 360.0f;
    while (yaw_degrees >= 360.0f) yaw_degrees -= 360.0f;
    
    // 将角度转换为弧度
    float yaw_rad = yaw_degrees * 3.14159f / 180.0f;
    
    // 根据速度和航向角计算位移 (单位: 秒)
    float delta_time_s = UPDATE_INTERVAL_MS_MAP / 1000.0f;
    float dx = speed_cm_s * cosf(yaw_rad) * delta_time_s;
    float dy = speed_cm_s * sinf(yaw_rad) * delta_time_s;
    
    // 更新当前位置
    tracker->current_x += dx;
    tracker->current_y += dy;
    
    // 确保在地图范围内
    if (tracker->current_x < 0) tracker->current_x = 0;
    if (tracker->current_x > MAP_SIZE_CM_MAP) tracker->current_x = MAP_SIZE_CM_MAP;
    if (tracker->current_y < 0) tracker->current_y = 0;
    if (tracker->current_y > MAP_SIZE_CM_MAP) tracker->current_y = MAP_SIZE_CM_MAP;
    
    // 调用原有的更新函数
    PathTracker_Update_MAP(tracker, tracker->current_x, tracker->current_y);
    
    // 检查是否创建了新的网格点
    return (tracker->point_count > previous_count) ? 1 : 0;
}

/**
 * @brief 更新位置 (核心函数)
 */
void PathTracker_Update_MAP(PathTracker_MAP *tracker, float current_x, float current_y)
{
    if (tracker == NULL || !tracker->is_recording) return;
    
    // 更新当前位置
    tracker->current_x = current_x;
    tracker->current_y = current_y;
    
    // 转换为网格坐标
    int16_t new_grid_x, new_grid_y;
    real_to_grid_MAP(current_x, current_y, &new_grid_x, &new_grid_y);
    
    // 检查是否移动到新的网格
    if (new_grid_x != tracker->current_grid_x || new_grid_y != tracker->current_grid_y) {
        
        uint16_t previous_point_index = tracker->current_point_index;
        
        // 查找是否已经有这个网格的点
        uint16_t existing_point = find_point_by_grid_MAP(tracker, new_grid_x, new_grid_y);
        
        if (existing_point != 0xFFFF) {
            // 已存在的点，建立连接
            tracker->current_point_index = existing_point;
        } else {
            // 新点，添加到路径
            if (tracker->point_count < MAX_MAP_POINTS_MAP) {
                MapPoint_MAP *point = &tracker->path_points[tracker->point_count];
                point->grid_x = new_grid_x;
                point->grid_y = new_grid_y;
                
                // 使用网格中心点作为实际坐标(更规整)
                grid_to_real_MAP(new_grid_x, new_grid_y, &point->x, &point->y);
                
                point->connection_count = 0;
                
                for (uint8_t i = 0; i < 4; i++) {
                    point->connected[i] = 0xFFFF;
                }
                
                tracker->current_point_index = tracker->point_count;
                tracker->point_count++;
            }
        }
        
        // 建立与前一个点的连接
        if (previous_point_index != 0xFFFF && tracker->current_point_index != 0xFFFF) {
            add_connection_MAP(tracker, previous_point_index, tracker->current_point_index);
            add_connection_MAP(tracker, tracker->current_point_index, previous_point_index);
        }
        
        // 更新当前网格坐标
        tracker->current_grid_x = new_grid_x;
        tracker->current_grid_y = new_grid_y;
        
        // 标记网格为已访问
        set_grid_visited_MAP(tracker, new_grid_x, new_grid_y);
    }
}

/**
 * @brief 获取最新路径点
 */
MapPoint_MAP* PathTracker_GetLatestPoint_MAP(PathTracker_MAP *tracker)
{
    if (tracker == NULL || tracker->point_count == 0) {
        return NULL;
    }
    
    return &tracker->path_points[tracker->point_count - 1];
}

/**
 * @brief 获取所有路径点数组
 */
MapPoint_MAP* PathTracker_GetAllPoints_MAP(PathTracker_MAP *tracker, uint16_t *count)
{
    if (tracker == NULL || count == NULL) {
        return NULL;
    }
    
    *count = tracker->point_count;
    return tracker->path_points;
}

/**
 * @brief 检查网格是否已访问过
 */
bool PathTracker_IsGridVisited_MAP(PathTracker_MAP *tracker, int16_t grid_x, int16_t grid_y)
{
    if (tracker == NULL) return false;
    
    return is_grid_visited_MAP(tracker, grid_x, grid_y);
}

/**
 * @brief 获取指定点的相邻连接点
 */
uint8_t PathTracker_GetConnections_MAP(PathTracker_MAP *tracker, uint16_t point_index, uint16_t connections[4])
{
    if (tracker == NULL || point_index >= tracker->point_count || connections == NULL) {
        return 0;
    }
    
    MapPoint_MAP *point = &tracker->path_points[point_index];
    uint8_t valid_connections = 0;
    
    for (uint8_t i = 0; i < point->connection_count && i < 4; i++) {
        if (point->connected[i] < tracker->point_count) {  // 确保连接索引有效
            connections[valid_connections] = point->connected[i];
            valid_connections++;
        }
    }
    
    return valid_connections;
}

/**
 * @brief 根据网格坐标查找点索引
 */
int16_t PathTracker_FindPointByGrid_MAP(PathTracker_MAP *tracker, int8_t grid_x, int8_t grid_y)
{
    if (tracker == NULL) {
        return -1;
    }
    
    for (uint16_t i = 0; i < tracker->point_count; i++) {
        if (tracker->path_points[i].grid_x == grid_x && 
            tracker->path_points[i].grid_y == grid_y) {
            return (int16_t)i;
        }
    }
    
    return -1;  // 未找到
}

/**
 * @brief 使用BFS算法查找最短路径
 */
uint8_t PathTracker_FindShortestPath_MAP(PathTracker_MAP *tracker, int8_t target_grid_x, int8_t target_grid_y)
{
    if (tracker == NULL || tracker->point_count == 0) {
        tracker->shortest_path_count = 0;
        return 0;
    }
    
    // 查找目标点
    int16_t target_idx = PathTracker_FindPointByGrid_MAP(tracker, target_grid_x, target_grid_y);
    if (target_idx < 0) {
        tracker->shortest_path_count = 0;
        return 0;
    }
    
    // 查找起始点（最新点）
    if (tracker->point_count == 0) {
        tracker->shortest_path_count = 0;
        return 0;
    }
    uint16_t start_idx = tracker->point_count - 1;
    
    // 如果起始点就是目标点
    if (start_idx == (uint16_t)target_idx) {
        tracker->shortest_path[0] = tracker->path_points[start_idx];
        tracker->shortest_path_count = 1;
        return 1;
    }
    
    // BFS数据结构（使用数组模拟队列）
    static uint16_t queue[MAX_MAP_POINTS_MAP];
    static int16_t parent[MAX_MAP_POINTS_MAP];
    static uint8_t visited[MAX_MAP_POINTS_MAP];
    
    // 初始化
    for (uint16_t i = 0; i < tracker->point_count; i++) {
        parent[i] = -1;
        visited[i] = 0;
    }
    
    // BFS
    uint16_t queue_front = 0, queue_rear = 0;
    queue[queue_rear++] = start_idx;
    visited[start_idx] = 1;
    
    uint8_t found = 0;
    while (queue_front < queue_rear && !found && queue_rear < MAX_MAP_POINTS_MAP) {
        uint16_t current = queue[queue_front++];
        
        // 获取当前点的连接
        uint16_t connections[4];
        uint8_t conn_count = PathTracker_GetConnections_MAP(tracker, current, connections);
        
        for (uint8_t i = 0; i < conn_count; i++) {
            uint16_t next = connections[i];
            
            if (!visited[next]) {
                visited[next] = 1;
                parent[next] = (int16_t)current;
                if (queue_rear < MAX_MAP_POINTS_MAP) {  // 防止数组越界
                    queue[queue_rear++] = next;
                }
                
                if (next == (uint16_t)target_idx) {
                    found = 1;
                    break;
                }
            }
        }
    }
    
    if (!found) {
        tracker->shortest_path_count = 0;
        return 0;
    }
    
    // 重建路径
    static uint16_t path_indices[MAX_SHORTEST_PATH_MAP];
    uint8_t path_length = 0;
    int16_t current = target_idx;
    
    while (current != -1 && path_length < MAX_SHORTEST_PATH_MAP) {
        path_indices[path_length++] = (uint16_t)current;
        current = parent[current];
    }
    
    // 反转路径（从起点到终点）
    tracker->shortest_path_count = path_length;
    for (uint8_t i = 0; i < path_length; i++) {
        tracker->shortest_path[i] = tracker->path_points[path_indices[path_length - 1 - i]];
    }
    
    return tracker->shortest_path_count;
}

/**
 * @brief 获取最短路径结果
 */
uint8_t PathTracker_GetShortestPath_MAP(PathTracker_MAP *tracker, MapPoint_MAP *path_buffer, uint8_t buffer_size)
{
    if (tracker == NULL || path_buffer == NULL || buffer_size == 0) {
        return 0;
    }
    
    uint8_t copy_count = (tracker->shortest_path_count < buffer_size) ? 
                        tracker->shortest_path_count : buffer_size;
    
    for (uint8_t i = 0; i < copy_count; i++) {
        path_buffer[i] = tracker->shortest_path[i];
    }
    
    return copy_count;
}

/**
 * @brief 获取所有历史路径点到指定数组（用于显示所有走过的路）
 */
uint16_t PathTracker_GetAllHistoryPath_MAP(PathTracker_MAP *tracker, MapPoint_MAP *history_buffer, uint16_t buffer_size)
{
    if (tracker == NULL || history_buffer == NULL || buffer_size == 0) {
        return 0;
    }
    
    uint16_t copy_count = (tracker->point_count < buffer_size) ? 
                         tracker->point_count : buffer_size;
    
    for (uint16_t i = 0; i < copy_count; i++) {
        history_buffer[i] = tracker->path_points[i];
    }
    
    return copy_count;
}

/**
 * @brief 一键获取最短路径并存储到用户数组（专为显示设计）
 * @param tracker 路径跟踪器
 * @param target_grid_x 目标网格X坐标
 * @param target_grid_y 目标网格Y坐标  
 * @param shortest_road_array 用户的最短路径数组
 * @param array_size 数组大小
 * @return 最短路径的实际长度（0表示未找到路径）
 */
uint8_t PathTracker_GetShortestRoadToArray_MAP(PathTracker_MAP *tracker, 
                                               int8_t target_grid_x, int8_t target_grid_y,
                                               MapPoint_MAP *shortest_road_array, uint8_t array_size)
{
    if (tracker == NULL || shortest_road_array == NULL || array_size == 0) {
        return 0;
    }
    
    // 先查找最短路径
    uint8_t path_length = PathTracker_FindShortestPath_MAP(tracker, target_grid_x, target_grid_y);
    
    if (path_length > 0) {
        // 直接复制到用户数组
        uint8_t copy_count = (path_length < array_size) ? path_length : array_size;
        
        for (uint8_t i = 0; i < copy_count; i++) {
            shortest_road_array[i] = tracker->shortest_path[i];
        }
        
        return copy_count;
    }
    
    return 0;
}
