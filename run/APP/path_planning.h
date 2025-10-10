/**
 * @file path_planning.h
 * @brief 基于DFS的路径规划系统
 */

#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include "mydefine.h"
#include "PID_app.h"  // 包含TurnType_t枚举
#include <stdint.h>
#include <stdbool.h>

/* 方向定义 */
typedef enum {
    DIR_NORTH = 0,  // 北(上)
    DIR_EAST  = 1,  // 东(右)
    DIR_SOUTH = 2,  // 南(下)
    DIR_WEST  = 3   // 西(左)
} Direction_t;

/* 路口类型(基于cross解码) */
typedef enum {
    CROSS_NONE      = 0,  // 无交叉 000
    CROSS_LEFT      = 1,  // 左转 100
    CROSS_RIGHT     = 2,  // 右转 001
    CROSS_T_FRONT  	= 3,  // 直行 010
    CROSS_STRAIGHT  = 4,  // T型左 110
    CROSS_T_LEFT   	= 5,  // T型右 011
    CROSS_T_RIGHT   = 6,  // T型前(左+右)
    CROSS_FOUR_WAY  = 7,  // 十字路口 111
    CROSS_DEAD_END  = 8   // 尽头(摄像头未识别到黑线)
} CrossType_t;

// 在路径规划中增加转向等待状态
typedef enum {
    STATE_FOLLOW_LINE,   // 循迹中
    STATE_AT_CROSS,      // 到达路口
    STATE_START_TURN,    // 开始转向
    STATE_TURNING,       // 转向中
    STATE_COMPLETE       // 任务完成
} MotionState_t;

/* 栅格单元状态 */
typedef struct {
    uint8_t visited_in;     // 从哪些方向进入过(bit0-3: 北东南西)
    uint8_t visited_out;    // 从哪些方向离开过(bit0-3: 北东南西)
    uint8_t explored;       // 是否完全探索(所有出口都走过)
} GridCell_t;

/* 地图配置 */
#define MAP_MAX_SIZE    20  // 最大栅格数
#define STACK_MAX_DEPTH 100 // DFS栈深度

/* 位置结构 */
typedef struct {
    int8_t x;
    int8_t y;
    Direction_t dir;  // 进入方向
} Position_t;

/* DFS栈 */
typedef struct {
    Position_t stack[STACK_MAX_DEPTH];
    uint8_t top;
} DFSStack_t;

/* 路径规划器 */
typedef struct {
    GridCell_t map[MAP_MAX_SIZE][MAP_MAX_SIZE];
    Position_t current;      // 当前位置
    DFSStack_t path_stack;   // 路径栈
    uint8_t map_center_x;    // 地图中心(初始位置)
    uint8_t map_center_y;
    uint8_t coverage_count;  // 已覆盖单元数
    bool mission_complete;   // 任务完成标志
} PathPlanner_t;

/* 运动控制器 */
typedef struct {
    MotionState_t state;
    Direction_t target_dir;
    uint32_t turn_timer;
    uint16_t turn_duration;  // 转弯持续时间(ms)
    float start_yaw;
    float target_yaw;
    float turn_target_angle;  // 转向目标角度
} MotionController_t;

/* 全局变量声明 */
extern PathPlanner_t g_planner;
extern MotionController_t g_motion_ctrl;

/* API声明 */
void PathPlanning_Init(void);
void PathPlanning_Update(uint8_t cross_val);
CrossType_t PathPlanning_DecodeCross(uint8_t cross);
Direction_t PathPlanning_DecideNextMove(uint8_t cross, float rho);
void PathPlanning_UpdatePosition(Direction_t move);
bool PathPlanning_IsComplete(void);
void MotionControl_ExecuteTurn(Direction_t from, Direction_t to);

/**
 * @brief 检查转向是否完成
 * @param current_yaw 当前航向角
 * @param target_yaw 目标航向角
 * @param tolerance 角度容差（度）
 * @return true:转向完成 false:转向未完成
 */
bool is_turn_completed(float current_yaw, float target_yaw, float tolerance);

/**
 * @brief 计算角度误差（考虑±180度边界）
 * @param current_yaw 当前航向角
 * @param target_yaw 目标航向角
 * @return 角度误差（-180到180度）
 */
float calculate_angle_error(float current_yaw, float target_yaw);

#endif
