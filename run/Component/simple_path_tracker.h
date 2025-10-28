/**
 * @file simple_path_tracker.h
 * @brief 简单路径追踪器（速度 + Yaw -> 网格路径）- 轻量适配单片机
 *
 * 设计目标：
 * - 起点为原点(0,0)，直角坐标系，yaw=0 朝向 +Y 轴；左转(+yaw)指向 -X，右转(-yaw)指向 +X。
 * - 每隔固定周期(可配置，默认10ms)用线速度(cm/s)与yaw(度)积分得到当前位置(cm)。
 * - 将位置按网格(可配置，单位cm)取整映射到网格坐标(int16)，生成只含(grid_x, grid_y)的点列。
 * - 若新网格与上一个相同则不追加；否则若x相同或y相同直接补齐中间点；否则用Bresenham补点，顺序为旧点->新点。
 * - 强调内存/效率：仅存储必要数据；全部参数通过宏可配。
 */

#ifndef SIMPLE_PATH_TRACKER_H
#define SIMPLE_PATH_TRACKER_H

#include <stdint.h>

/* ===================== 可配置宏（按需修改） ===================== */

/* 最大网格点数量（路径数组长度上限） */
#ifndef SPT_MAX_POINTS
#define SPT_MAX_POINTS           1024
#endif

/* 网格大小（cm/格），影响取整与重合判断 */
#ifndef SPT_GRID_SIZE_CM
#define SPT_GRID_SIZE_CM         4.0f
#endif

/* 任务调用周期（ms），决定位移 = 速度(cm/s) * (SPT_UPDATE_INTERVAL_MS/1000) */
#ifndef SPT_UPDATE_INTERVAL_MS
#define SPT_UPDATE_INTERVAL_MS   10
#endif

/* 速度限幅（cm/s） */
#ifndef SPT_MAX_SPEED_CM_S
#define SPT_MAX_SPEED_CM_S       25
#endif

/* 最短路径算法：采样因子（将网格坐标除以此值后取整再乘以此值） */
#ifndef SPT_SAMPLE_FACTOR
#define SPT_SAMPLE_FACTOR        2
#endif

/* 最短路径算法：采样后可通行点的最大数量 */
#ifndef SPT_MAX_SAMPLED_POINTS
#define SPT_MAX_SAMPLED_POINTS   256
#endif

/* 最短路径算法：BFS队列大小 */
#ifndef SPT_BFS_QUEUE_SIZE
#define SPT_BFS_QUEUE_SIZE       512
#endif

/* 最短路径算法：已访问节点数组大小 */
#ifndef SPT_BFS_VISITED_SIZE
#define SPT_BFS_VISITED_SIZE     512
#endif

/* ===================== 数据结构 ===================== */

/* 仅包含网格坐标，满足“只要x、y网格坐标”的需求 */
typedef struct {
    int16_t grid_x;
    int16_t grid_y;
} SPT_Point;

typedef struct {
    /* 连续域下的当前位置（cm） */
    float x_cm;
    float y_cm;

    /* 最新网格坐标（用于重合判断与连线） */
    int16_t last_grid_x;
    int16_t last_grid_y;
    uint8_t has_last; /* 0=还没有任何网格点，1=已有 */

    /* 路径数组与长度（index） */
    SPT_Point points[SPT_MAX_POINTS];
    int length; /* 当前有效点个数（也是最后一个点的下标+1） */
} SPT_Tracker;

/* ===================== API ===================== */

/* 初始化：原点(0,0)，清空路径并写入首点(0,0) */
void SPT_Init(SPT_Tracker* tracker);

/* 周期调用：以速度(cm/s，int)与yaw(度，float，[-180,180])更新路径 */
void SPT_Update(SPT_Tracker* tracker, int speed_cm_s, float yaw_deg);

/* 获取路径数组与长度（只读指针，长度写到out_len） */
const SPT_Point* SPT_GetPath(const SPT_Tracker* tracker, int* out_len);

/**
 * @brief 基于历史路径生成到终点的最短路径（BFS算法）
 * 
 * 算法步骤：
 * 1. 对历史路径进行采样去重（除以SPT_SAMPLE_FACTOR取整再乘以SPT_SAMPLE_FACTOR）
 * 2. 使用BFS四方向搜索从原点(0,0)到终点的最短路径
 * 3. 只在采样后的可通行点之间移动
 * 4. 如果终点不可达，则以历史路径最新点为终点重试
 * 5. 仍不可达则返回-1
 * 
 * @param history_path 历史路径网格坐标（输入）
 * @param shortest_path 最短路径结果（输出，会被清空重写）
 * @param end_x 终点X网格坐标
 * @param end_y 终点Y网格坐标
 * @return 1=成功找到路径，0=终点不可达但找到到最新点的路径，-1=完全失败
 */
int SPT_FindShortestPath(const SPT_Tracker* history_path, 
                         SPT_Tracker* shortest_path,
                         int16_t end_x, 
                         int16_t end_y);

#endif /* SIMPLE_PATH_TRACKER_H */
