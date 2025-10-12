/**
 * @file path_planning.c
 * @brief 基于DFS的路径规划系统实现
 * @details 本文件实现了基于深度优先搜索(DFS)的迷宫遍历路径规划算法
 *          主要功能包括：
 *          1. 基于左手法则的路径决策
 *          2. 路口类型识别与解码
 *          3. 运动状态机控制
 *          4. 位置更新与地图标记
 *          5. 转向动作执行
 */
 
 

#include "path_planning.h"  // 路径规划头文件
#include "PID_app.h"        // PID控制头文件
#include <string.h>         // 字符串处理库
#include <stdio.h>          // 标准输入输出库
#include <math.h>           // 数学函数库

#define basic_speed 15      // 基础行驶速度，与PID_app.c保持一致

// 外部PID控制器声明，用于控制四个电机的速度
extern PID_T pid_speed_A;   // A电机PID控制器
extern PID_T pid_speed_B;   // B电机PID控制器
extern PID_T pid_speed_C;   // C电机PID控制器
extern PID_T pid_speed_D;   // D电机PID控制器
extern MotorDriver_t car;

extern float f_yaw;
extern float f_rho;
extern uint8_t f_cross;
extern uint8_t pid_running;  // 声明PID运行标志位

/* ==================== 全局变量定义 ==================== */
PathPlanner_t g_planner;        // 路径规划器全局实例
MotionController_t g_motion_ctrl; // 运动控制器全局实例

// 稳定性检测相关变量
typedef struct {
    uint8_t last_cross_val;       // 上次路口类型值
    uint32_t stable_start_time;   // 稳定开始时间
    bool is_stable;               // 是否稳定
    uint32_t stable_duration;     // 稳定持续时间(ms)
} StabilityDetector_t;

static StabilityDetector_t g_stability = {0};

/* ==================== 内部辅助函数声明 ==================== */
static Direction_t get_relative_direction(Direction_t current_dir, int8_t turn);  // 计算相对方向
static bool is_direction_allowed(uint8_t cross, Direction_t try_dir, Direction_t current_dir);  // 检查方向是否允许
static bool is_path_visited(Direction_t move_dir);  // 检查路径是否重复
static void execute_turn_action(Direction_t from, Direction_t to);  // 执行转向动作
static bool is_cross_stable(uint8_t cross_val);  // 检测路口类型是否稳定


/* ==================== 路径规划系统初始化 ==================== */
/**
 * @brief 初始化路径规划系统
 * @details 设置地图中心点、当前位置、初始方向等参数
 *          并初始化运动控制状态机
 */
void PathPlanning_Init(void) {
    // 清零所有结构体数据
    memset(&g_planner, 0, sizeof(PathPlanner_t));
    memset(&g_motion_ctrl, 0, sizeof(MotionController_t));
    
    // 设置地图中心为初始位置（地图中心点）
    g_planner.map_center_x = MAP_MAX_SIZE / 2;
    g_planner.map_center_y = MAP_MAX_SIZE / 2;
    
    // 设置当前位置为地图中心
    g_planner.current.x = g_planner.map_center_x;
    g_planner.current.y = g_planner.map_center_y;
    g_planner.current.dir = DIR_NORTH;  // 初始朝向北方向
    
    // 标记起点已访问（使用位掩码标记从北方向进入）
    g_planner.map[g_planner.current.y][g_planner.current.x].visited_in = 1 << DIR_NORTH;
	    // 标记起点从北方向离开（因为初始朝向北）
    g_planner.map[g_planner.current.y][g_planner.current.x].visited_out = 1 << DIR_NORTH;
	
    g_planner.coverage_count = 1;  // 已覆盖区域计数
    
    // 初始化运动控制状态机
    g_motion_ctrl.state = STATE_FOLLOW_LINE;  // 初始状态：循线
    g_motion_ctrl.target_dir = DIR_NORTH;     // 目标方向：北
    g_motion_ctrl.turn_duration = 500;        // 转弯持续时间：500ms
    
    printf("[PathPlanning] Initialized at (%d,%d)\n", 
           g_planner.current.x, g_planner.current.y);
}

/* ==================== 主更新函数 ==================== */
/**
 * @brief 路径规划主更新函数
 * @param f_rho 视觉系统返回的横向偏差值
 * @param cross_val 路口检测值
 * @param yaw_val 当前航向角值
 * @details 根据当前运动状态执行相应的控制逻辑
 */
void PathPlanning_Update(uint8_t cross_val) {
    // 统一处理：f_cross >= 8 都视为尽头
    if (cross_val >= 8) {
        uint8_t original_cross = cross_val;
        cross_val = 8;  // 强制设置为8（尽头）
        // my_printf(&huart4,"[DEBUG] f_cross=%d converted to 8 (dead end)\n", original_cross);
    }

	Direction_t next_dir;           // 下一步移动方向
	// 调用航向控制（非阻塞）
					TurnType_t turn_type = TURN_NONE;
	
    // 根据当前运动状态执行相应逻辑
    switch (g_motion_ctrl.state) {
        case STATE_FOLLOW_LINE:  // 循线状态
            // 启用PID控制
            pid_running = 1;
            
            // 调用视觉PID控制进行循线
            VisionControl_Update(f_rho);
            
            // 检测路口：当路口类型不为直行(4)和十字路口(7)且横向偏差接近中心时
            if (cross_val != 4 && cross_val != 7 && cross_val != 0) {
                if (fabs(f_rho - 76.0f) < 15.0f) {  // 放宽到±15
                    // 延时稳定性检测：确保路口类型稳定
                    if (is_cross_stable(cross_val)) {
                        g_motion_ctrl.state = STATE_AT_CROSS;
                        // my_printf(&huart4,"[DEBUG] Cross detected and stable, entering AT_CROSS state\n");
                    } else {
                        // my_printf(&huart4,"[DEBUG] Cross detected but unstable, waiting...\n");
                    }
                }
            }
            break;
            
        case STATE_AT_CROSS:  // 路口状态
            // 调试信息：显示当前路口类型
            // my_printf(&huart4,"[DEBUG] At cross: f_cross=%d, f_rho=%.2f\n", cross_val, f_rho);
            
            // 直接进行转向决策
            next_dir = PathPlanning_DecideNextMove(cross_val, f_rho);
				
                 // 特殊处理：f_cross=6时，如果所有方向都走过，继续直行一步
            if (cross_val == 6 && next_dir == g_planner.current.dir) {
                // my_printf(&huart4,"[DEBUG] T-right cross (6): all directions visited, continuing straight\n");
                
                // 禁用PID控制，使用固定速度直行
                pid_running = 0;
                
                MotorDriver_SetMotor(&car,MOTOR_A,15);
                MotorDriver_SetMotor(&car,MOTOR_B,15);
                MotorDriver_SetMotor(&car,MOTOR_C,15);
                MotorDriver_SetMotor(&car,MOTOR_D,15);
                
                // 直行一步（200ms）
                HAL_Delay(1000);
                
                // my_printf(&huart4,"[DEBUG] T-right cross: straight step completed, returning to FOLLOW_LINE\n");
                
                // 重新启用PID控制，返回循线状态
                pid_running = 1;
                PID_Init();
                HAL_Delay(10);
                g_motion_ctrl.state = STATE_FOLLOW_LINE;
                break;
            }       
						
             // 判断是否需要转向
            if (next_dir != g_planner.current.dir) {
                // 计算相对转向角度（0=直行，1=右转，2=掉头，3=左转）
                int8_t diff = (next_dir - g_planner.current.dir + 4) % 4;
                
                // 左转前额外走两步
                if (diff == 3) {  // 左转
                    // my_printf(&huart4,"[DEBUG] Left turn detected, taking 2 extra steps\n");
                    
                    // 禁用PID控制，使用固定速度直行
                    pid_running = 0;
                    
                    MotorDriver_SetMotor(&car,MOTOR_A,15);
                    MotorDriver_SetMotor(&car,MOTOR_B,15);
                    MotorDriver_SetMotor(&car,MOTOR_C,15);
                    MotorDriver_SetMotor(&car,MOTOR_D,15);
                    
                    // 走两步（每步150ms）
                    HAL_Delay(75);  // 第一步
                    HAL_Delay(150);  // 第二步
                    
                    // my_printf(&huart4,"[DEBUG] Left turn extra steps completed\n");
                }
                
                // 转弯前固定延时直行300ms
                // my_printf(&huart4,"[DEBUG] Pre-turn delay: continuing straight for 300ms\n");
                
                // 禁用PID控制，使用固定速度直行
                pid_running = 0;
							
                MotorDriver_SetMotor(&car,MOTOR_A,10);
                MotorDriver_SetMotor(&car,MOTOR_B,10);
                MotorDriver_SetMotor(&car,MOTOR_C,10);
                MotorDriver_SetMotor(&car,MOTOR_D,10);
                HAL_Delay(300);
                
                // 重新启用PID控制
                pid_running = 1;
								PID_Init();
								HAL_Delay(10);
                
                // 需要转向：设置目标方向并切换到开始转向状态
                g_motion_ctrl.target_dir = next_dir;
                g_motion_ctrl.state = STATE_START_TURN;
						// my_printf(&huart4,"next_dir:%d",next_dir);
            } else {
                // 直行：启用PID控制，直接返回循线状态
                pid_running = 1;
								PID_Init();
								HAL_Delay(10);
							
                g_motion_ctrl.state = STATE_FOLLOW_LINE;
            }
            break;

            
        case STATE_START_TURN:  // 开始转向状态
            // 禁用PID控制，开始转向
            pid_running = 0;
				
            // 保存当前航向角作为转向起点
            g_motion_ctrl.start_yaw = f_yaw;
            // 执行转向动作（设置目标角度）
            execute_turn_action(g_planner.current.dir, g_motion_ctrl.target_dir);
				
				    // 调试信息：显示转向状态
            // my_printf(&huart4,"[DEBUG] Turning: f_yaw=%.2f, target_yaw=%.2f, turn_target=%.2f\n", 
                    //  f_yaw, g_motion_ctrl.target_yaw, g_motion_ctrl.turn_target_angle);
                               
            // 根据目标角度确定转向类型（顺时针为负值）
            if (fabs(g_motion_ctrl.target_yaw - 0.0f) < 0.1f) {
                turn_type = TURN_NONE;  // 直行
            } else if (fabs(g_motion_ctrl.target_yaw - (-90.0f)) < 0.1f) {
                turn_type = TURN_RIGHT;  // 右转（顺时针-90度）
            } else if (fabs(g_motion_ctrl.target_yaw - 90.0f) < 0.1f) {
                turn_type = TURN_LEFT;   // 左转（逆时针90度）
            } else if (fabs(g_motion_ctrl.target_yaw - (-180.0f)) < 0.1f || fabs(g_motion_ctrl.target_yaw - 180.0f) < 0.1f) {
                turn_type = TURN_U_TURN; // 掉头（±180度）
            }
            
            YAWControl_Update(turn_type);
				
				
				
            g_motion_ctrl.state = STATE_TURNING;  // 切换到转向状态
				
            break;
            
        case STATE_TURNING:  // 转向状态
				
            // 调试信息：显示转向状态
            // my_printf(&huart4,"[DEBUG] Turning: f_yaw=%.2f, target_yaw=%.2f, turn_target=%.2f\n", 
            //         f_yaw, g_motion_ctrl.target_yaw, g_motion_ctrl.turn_target_angle);
            
            // 使用相对转向角度进行转向完成检测
            if (fabs(calculate_angle_error(f_yaw, g_motion_ctrl.turn_target_angle)) <= 5.0f) {
                // 转向完成：停止所有电机
                MotorDriver_StopAll(&car);						
                HAL_Delay(200);
								MotorDriver_SetMotor(&car,MOTOR_A,10);
								MotorDriver_SetMotor(&car,MOTOR_B,10);
								MotorDriver_SetMotor(&car,MOTOR_C,10);
								MotorDriver_SetMotor(&car,MOTOR_D,10);		
								HAL_Delay(200);
								MotorDriver_SetMotor(&car,MOTOR_A,15);
								MotorDriver_SetMotor(&car,MOTOR_B,15);
								MotorDriver_SetMotor(&car,MOTOR_C,15);
								MotorDriver_SetMotor(&car,MOTOR_D,15);		
								HAL_Delay(200);
                // 更新位置并返回循线状态
                PathPlanning_UpdatePosition(g_motion_ctrl.target_dir);
                g_motion_ctrl.state = STATE_FOLLOW_LINE;
                
                // 重置稳定性检测，为下次路口检测做准备
                g_stability.last_cross_val = 0xFF;  // 设置为无效值，强制重新检测
                g_stability.stable_start_time = 0;
                g_stability.is_stable = false;
								
								pid_running = 1;
								PID_Init();
								HAL_Delay(10);
                
                // my_printf(&huart4,"[DEBUG] Turn completed, returning to follow line\n");
            }
            break;
        case STATE_COMPLETE:  // 完成状态
            // 禁用PID控制
            pid_running = 0;
				
            // 停止所有电机
            pid_update_target(&pid_speed_A, 0);
            pid_update_target(&pid_speed_B, 0);
            pid_update_target(&pid_speed_C, 0);
            pid_update_target(&pid_speed_D, 0);
            break;
    }
}

/* ==================== 路口类型解码 ==================== */
/**
 * @brief 解码路口检测值
 * @param cross 硬件返回的路口检测值
 * @return CrossType_t 解码后的标准路口类型
 * @details 将硬件检测的原始值转换为标准的路口类型枚举
 *          硬件映射关系：
 *          000=>0  001=>2  010=>4  011=>6
 *          100=>1  101=>3  110=>5  111=>7
 */
CrossType_t PathPlanning_DecodeCross(uint8_t cross) {
    // 根据实际硬件映射关系解码
    // 从左往右方框返回值=>cross对应关系：
    // 000=>0  001=>2  010=>4  011=>6
    // 100=>1  101=>3  110=>5  111=>7
    
    // 重新映射到标准路口类型
    switch (cross) {
        case 0: return CROSS_NONE;      // 000 - 无交叉路口
        case 2: return CROSS_RIGHT;     // 001 - 右转路口
        case 4: return CROSS_STRAIGHT;  // 010 - 直行路口
        case 6: return CROSS_T_RIGHT;   // 011 - T型右路口(右转+直行)
        case 1: return CROSS_LEFT;      // 100 - 左转路口
        case 5: return CROSS_T_LEFT;    // 110 - T型左路口(左转+直行)
        case 3: return CROSS_T_FRONT;   // 101 - T型前路口(左转+右转)
        case 7: return CROSS_FOUR_WAY;  // 111 - 十字路口
		default: return CROSS_NONE;     // 处理所有未定义的情况，返回默认值
    }
}

/* ==================== 路径决策核心函数 ==================== */
/**
 * @brief 决策下一步移动方向
 * @param cross 当前路口类型值
 * @param rho 当前横向偏差值
 * @return Direction_t 下一步移动方向，0xFF表示需要回溯
 * @details 基于改进的左手法则进行路径决策：
 *          1. 优先选择左转
 *          2. 其次选择直行
 *          3. 再次选择右转
 *          4. 最后选择掉头
 *          5. 如果所有方向都访问过，返回回溯信号
 */
Direction_t PathPlanning_DecideNextMove(uint8_t cross, float rho) {
    Direction_t current_dir = g_planner.current.dir;
    
    // 根据路口类型确定可选方向
    Direction_t available_dirs[4];
    int dir_count = 0;
    
	  uint32_t current_time = HAL_GetTick();
	
    // 根据cross值确定可选方向（按左手法则优先级：左转 > 直行 > 右转 > 掉头）
    switch (cross) {        
        case 0:  // 无交叉路口 - 只允许直行
            available_dirs[dir_count++] = current_dir;  // 直行
            break;

        case 1:  // 左转路口 - 只能左转
            available_dirs[dir_count++] = get_relative_direction(current_dir, -1);  // 左转
            break;
            
        case 2:  // 右转路口 - 只能右转
            available_dirs[dir_count++] = get_relative_direction(current_dir, 1);   // 右转
            break;
            
        case 3:  // T型前路口 - 左转+右转
            available_dirs[dir_count++] = get_relative_direction(current_dir, -1);  // 左转
            available_dirs[dir_count++] = get_relative_direction(current_dir, 1);   // 右转
            break;
            
        case 5:  // T型左路口 - 左转+直行
					  available_dirs[dir_count++] = get_relative_direction(current_dir, -1);  // 左转
				    available_dirs[dir_count++] = current_dir;                              // 直行
            break;
            
        case 6:  // T型右路口 - 右转+直行
					  available_dirs[dir_count++] = get_relative_direction(current_dir, 1);   // 右转
				    available_dirs[dir_count++] = current_dir;                              // 直行

            break;
            
        case 7:  // 十字路口 - 所有方向可选
            available_dirs[dir_count++] = current_dir;                              // 直行
            break;
            
        case 8:  // 摄像头未识别到黑线 - 尽头，需要掉头
            // my_printf(&huart4,"[DEBUG] Dead end detected (f_cross=8), preparing U-turn\n");
            available_dirs[dir_count++] = get_relative_direction(current_dir, 2);  // 掉头
            break;
            
        default:  // 未知路口类型，尝试所有方向
            available_dirs[dir_count++] = current_dir;                              // 直行
            break;
    }
    
    // 按优先级检查每个方向
    for (int i = 0; i < dir_count; i++) {
        Direction_t try_dir = available_dirs[i];
        
        // 检查该方向是否在当前路口允许
        if (is_direction_allowed(cross, try_dir, current_dir)) {
            // 特殊处理：f_cross=8时强制掉头，不管是否走过
            if (cross == 8) {
                return try_dir;  // 尽头时强制掉头
            }
            // 检查该路径是否未走过
            if (!is_path_visited(try_dir)) {
                return try_dir;  // 找到未走过的路径
            }
        }
    }
    
    // 所有方向都走过，触发回溯
    return (Direction_t)current_dir;
}

/* ==================== 方向可行性检查函数 ==================== */
/**
 * @brief 检查指定方向是否在当前路口允许
 * @param cross 路口类型值
 * @param try_dir 要尝试的方向
 * @param current_dir 当前朝向方向
 * @return bool true表示该方向允许，false表示不允许
 * @details 根据路口类型和相对转向角度判断方向是否可行
 */
bool is_direction_allowed(uint8_t cross, Direction_t try_dir, Direction_t current_dir) {
    // 计算相对转向角度（0=直行，1=右转，2=掉头，3=左转）
    int8_t diff = (try_dir - current_dir + 4) % 4;
    
    switch (diff) {
        case 0:  // 直行
            return (cross == 0) || (cross == 4) || (cross == 7); // 检查直行位（无交叉、直行路口或十字路口）
        case 1:  // 右转
            return  (cross == 2) || (cross == 6); // 检查右转位（右转路口或T型右路口）
        case 2:  // 掉头
            return  (cross == 8); // 在尽头允许掉头
        case 3:  // 左转
            return (cross == 1) || (cross == 5) || (cross == 3); // 检查左转位（左转、T型左、T型前路口）
        default:
            return false;  // 无效的相对角度
    }
}

/* ==================== 路径重复检查函数 ==================== */
/**
 * @brief 检查路径是否重复
 * @param move_dir 移动方向
 * @return bool true表示路径已重复，false表示未重复
 * @details 检查目标栅格是否从相反方向离开过，避免重复走相同路径
 */
bool is_path_visited(Direction_t move_dir) {
    // 获取当前位置坐标
    int8_t next_x = g_planner.current.x;
    int8_t next_y = g_planner.current.y;
    
    // 根据移动方向计算目标位置坐标
    switch (move_dir) {
        case DIR_NORTH: next_y--; break;  // 向北移动，Y坐标减1
        case DIR_EAST:  next_x++; break;  // 向东移动，X坐标加1
        case DIR_SOUTH: next_y++; break;  // 向南移动，Y坐标加1
        case DIR_WEST:  next_x--; break;  // 向西移动，X坐标减1
    }
    
    // 边界检查：如果目标位置超出地图范围，视为已访问
    if (next_x < 0 || next_x >= MAP_MAX_SIZE || 
        next_y < 0 || next_y >= MAP_MAX_SIZE) {
        return true;
    }
    
    // 计算相反方向（目标栅格到当前栅格的方向）
    Direction_t opposite_dir = get_relative_direction(move_dir, 2);  // 相反方向 = 当前方向 + 180度
    
    // 检查目标栅格是否从相反方向离开过
    uint8_t opposite_mask = 1 << opposite_dir;  // 创建相反方向位掩码
    return (g_planner.map[next_y][next_x].visited_out & opposite_mask) != 0;  // 检查对应位是否已设置
}

/* ==================== 辅助函数 ==================== */
/**
 * @brief 计算相对方向
 * @param current_dir 当前方向
 * @param turn 转向参数（-1=左转, 0=直行, 1=右转, 2=掉头）
 * @return Direction_t 计算后的相对方向
 * @details 根据当前方向和转向参数计算目标方向
 */
static Direction_t get_relative_direction(Direction_t current_dir, int8_t turn) {
    // turn: -1=左转, 0=直行, 1=右转, 2=回头
    // 使用模运算确保结果在0-3范围内
    return (Direction_t)((current_dir + turn + 4) % 4);
}


/* ==================== 位置更新函数 ==================== */
/**
 * @brief 更新机器人位置和方向
 * @param move 移动方向
 * @details 将当前位置压入路径栈，然后根据移动方向更新坐标和朝向
 */
void PathPlanning_UpdatePosition(Direction_t move) {
    // 记录当前栅格从该方向离开
    uint8_t exit_mask = 1 << move;
    g_planner.map[g_planner.current.y][g_planner.current.x].visited_out |= exit_mask;
    
    // 根据移动方向更新坐标
    switch (move) {
        case DIR_NORTH: g_planner.current.y--; break;  // 向北移动，Y坐标减1
        case DIR_EAST:  g_planner.current.x++; break;  // 向东移动，X坐标加1
        case DIR_SOUTH: g_planner.current.y++; break;  // 向南移动，Y坐标加1
        case DIR_WEST:  g_planner.current.x--; break;  // 向西移动，X坐标减1
    }
    g_planner.current.dir = move;  // 更新当前朝向
    
    // 将更新后的位置压入路径栈（用于回溯）
    if (g_planner.path_stack.top < STACK_MAX_DEPTH - 1) {
        g_planner.path_stack.stack[g_planner.path_stack.top++] = g_planner.current;
    }
    
    // 记录新栅格从该方向进入（使用位或操作）
    uint8_t enter_mask = 1 << move;
    g_planner.map[g_planner.current.y][g_planner.current.x].visited_in |= enter_mask;
    
    // my_printf(&huart4,"[Position] Moved to (%d,%d) facing %d\n", 
        //    g_planner.current.x, g_planner.current.y, g_planner.current.dir);
}



/* ==================== 转向动作执行函数 ==================== */
/**
 * @brief 执行转向动作
 * @param from 当前方向
 * @param to 目标方向
 * @details 根据转向角度执行相应的转向动作，包括直行、右转、掉头、左转
 */
static void execute_turn_action(Direction_t from, Direction_t to) {
    // 保存当前航向角，避免全局变量f_yaw在后续执行中发生变化
    float current_yaw = f_yaw;
    
    // 计算相对转向角度（0=直行，1=右转，2=掉头，3=左转）
    int8_t diff = (to - from + 4) % 4;
    
    // 根据转向类型计算目标角度（顺时针为负值）
    switch (diff) {
        case 0:  // 直行
            g_motion_ctrl.target_yaw = 0.0f;
            g_motion_ctrl.turn_target_angle = current_yaw;
            break;
        case 1:  // 右转（顺时针-90度）
            g_motion_ctrl.target_yaw = -90.0f;
            g_motion_ctrl.turn_target_angle = current_yaw - 90.0f;
            break;
        case 2:  // 掉头（±180度）
            g_motion_ctrl.target_yaw = 180.0f;
            g_motion_ctrl.turn_target_angle = current_yaw + 180.0f;
            break;
        case 3:  // 左转（逆时针+90度）
            g_motion_ctrl.target_yaw = 90.0f;
            g_motion_ctrl.turn_target_angle = current_yaw + 90.0f;
            break;
        default:
            g_motion_ctrl.target_yaw = 0.0f;
            g_motion_ctrl.turn_target_angle = current_yaw;
            break;
    }
    
    // 角度归一化到-180到180度
    while (g_motion_ctrl.turn_target_angle > 180.0f) g_motion_ctrl.turn_target_angle -= 360.0f;
    while (g_motion_ctrl.turn_target_angle < -180.0f) g_motion_ctrl.turn_target_angle += 360.0f;
    
    // 调试信息
    // my_printf(&huart4,"[DEBUG] Turn action: from=%d to=%d, diff=%d, target_yaw=%.2f, turn_target=%.2f\n", 
            //  from, to, diff, g_motion_ctrl.target_yaw, g_motion_ctrl.turn_target_angle);
    
    // ✅ 不阻塞，只设置目标
    // 实际转向在STATE_TURNING状态中执行
}

/**
 * @brief 检查转向是否完成
 * @param current_yaw 当前航向角
 * @param target_yaw 目标航向角
 * @param tolerance 角度容差（度）
 * @return true:转向完成 false:转向未完成
 */
bool is_turn_completed(float current_yaw, float target_yaw, float tolerance) {
    float error = calculate_angle_error(current_yaw, target_yaw);
    return (fabs(error) <= tolerance);
}

/**
 * @brief 计算角度误差（考虑±180度边界）
 * @param current_yaw 当前航向角
 * @param target_yaw 目标航向角
 * @return 角度误差（-180到180度）
 */
float calculate_angle_error(float current_yaw, float target_yaw) {
    float error = target_yaw - current_yaw;
    
    // 处理角度跨越±180°边界（最短路径原则）
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }
    
    return error;
}

/**
 * @brief 基于延时的稳定性检测
 * @param cross_val 当前路口类型值
 * @return true:稳定 false:不稳定
 * @details 通过检测路口类型值在指定时间内是否保持一致来判断稳定性
 */
static bool is_cross_stable(uint8_t cross_val) {
    uint32_t current_time = HAL_GetTick();
    
    // 如果路口类型发生变化，重新开始计时
    if (cross_val != g_stability.last_cross_val) {
        g_stability.last_cross_val = cross_val;
        g_stability.stable_start_time = current_time;
        g_stability.is_stable = false;
        // my_printf(&huart4,"[DEBUG] Stability: Cross changed to %d, restarting timer\n", cross_val);
        return false;
    }
    
    // 如果路口类型相同，检查是否已经稳定足够长时间
    if (g_stability.stable_start_time == 0) {
        g_stability.stable_start_time = current_time;
        g_stability.is_stable = false;
        // my_printf(&huart4,"[DEBUG] Stability: Starting timer for cross=%d\n", cross_val);
        return false;
    }
    
    // 计算稳定持续时间
    g_stability.stable_duration = current_time - g_stability.stable_start_time;
    
    // 如果稳定时间超过阈值（180ms），认为稳定
    if (g_stability.stable_duration >= 110) {
        if (!g_stability.is_stable) {
            g_stability.is_stable = true;
            // my_printf(&huart4,"[DEBUG] Stability: Cross %d is now stable after %lu ms\n", 
            //          cross_val, g_stability.stable_duration);
        }
        return true;
    } else {
        g_stability.is_stable = false;
        // my_printf(&huart4,"[DEBUG] Stability: Cross %d stable for %lu ms (need 110ms)\n", 
        //          cross_val, g_stability.stable_duration);
        return false;
    }
}
