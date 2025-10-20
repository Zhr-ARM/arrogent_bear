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
#define SPT_MAX_POINTS           512
#endif

/* 网格大小（cm/格），影响取整与重合判断 */
#ifndef SPT_GRID_SIZE_CM
#define SPT_GRID_SIZE_CM         5.0f
#endif

/* 任务调用周期（ms），决定位移 = 速度(cm/s) * (SPT_UPDATE_INTERVAL_MS/1000) */
#ifndef SPT_UPDATE_INTERVAL_MS
#define SPT_UPDATE_INTERVAL_MS   10
#endif

/* 速度限幅（cm/s） */
#ifndef SPT_MAX_SPEED_CM_S
#define SPT_MAX_SPEED_CM_S       25
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

/* 从历史网格点中（以2格为采样步长）构建起点(0,0)到终点(x,y, cm)的最短路径，
 * 结果写入 out_path->points 并更新 out_path->length（其余字段不改）。
 * 若不可达，则 length=0。history 为历史网格点数组，长度为 history_len。 */
void SPT_BuildShortestPathFromHistory(const SPT_Point* history, int history_len,
                                      SPT_Tracker* out_path, float end_x_cm, float end_y_cm);

#endif /* SIMPLE_PATH_TRACKER_H */
