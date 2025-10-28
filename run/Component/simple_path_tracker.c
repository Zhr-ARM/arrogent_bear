/**
 * @file simple_path_tracker.c
 * @brief 简单路径追踪器实现（速度 + Yaw -> 网格路径）
 */

#include "simple_path_tracker.h"
#include <math.h>

/* 局部内联与静态函数，保证效率 */

/* 将连续坐标(cm)映射为网格坐标：按网格大小做四舍五入到最近格点 */
static inline int16_t spt_to_grid(float v_cm) {
    const float inv = 1.0f / SPT_GRID_SIZE_CM;
    float g = v_cm * inv;
    /* 自定义的就近取整：等价 roundf，但避免引入额外库 */
    if (g >= 0.0f) g = (float)((int)(g + 0.5f));
    else           g = (float)((int)(g - 0.5f));
    return (int16_t)g;
}

/* 安全追加一个网格点到数组（若满则忽略） */
static inline void spt_push_point(SPT_Tracker* t, int16_t gx, int16_t gy) {
    if (t->length >= SPT_MAX_POINTS) return;
    t->points[t->length].grid_x = gx;
    t->points[t->length].grid_y = gy;
    t->length++;
}

/* 用Bresenham算法从上一个点连到新点（不重复包含起点，仅追加中间点与终点） */
static void spt_connect_line(SPT_Tracker* t, int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = (y1 > y0) ? (y0 - y1) : (y1 - y0); /* 注意 dy 取反用于经典写法 */
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy; /* err = dx + dy(已为负) */

    int x = x0;
    int y = y0;

    /* 迭代，跳过第一个点(避免重复)，直到到达终点 */
    for (;;) {
        if (x == x1 && y == y1) {
            /* 终点：追加 */
            spt_push_point(t, (int16_t)x, (int16_t)y);
            break;
        }

        int e2 = err << 1;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }

        /* 跳过起点后每一步都写入 */
        if (!(x == x0 && y == y0)) {
            /* 若达到终点，循环开头也会写入，这里不提前 break 以保持代码简洁 */
            if (x == x1 && y == y1) continue; /* 终点写在循环顶部 */
            spt_push_point(t, (int16_t)x, (int16_t)y);
        }
    }
}

void SPT_Init(SPT_Tracker* tracker) {
    if (!tracker) return;
    tracker->x_cm = 0.0f;
    tracker->y_cm = 0.0f;
    tracker->last_grid_x = 0;
    tracker->last_grid_y = 0;
    tracker->has_last = 0;
    tracker->length = 0;

    /* 将原点映射网格并作为第一个点 */
    int16_t gx = spt_to_grid(0.0f);
    int16_t gy = spt_to_grid(0.0f);
    spt_push_point(tracker, gx, gy);
    tracker->last_grid_x = gx;
    tracker->last_grid_y = gy;
    tracker->has_last = 1;
}

void SPT_Update(SPT_Tracker* tracker, int speed_cm_s, float yaw_deg) {
    if (!tracker) return;

    /* 限幅速度 */
    if (speed_cm_s >  SPT_MAX_SPEED_CM_S) speed_cm_s =  SPT_MAX_SPEED_CM_S;
    if (speed_cm_s < -SPT_MAX_SPEED_CM_S) speed_cm_s = -SPT_MAX_SPEED_CM_S;

    /* 时间步长(s) */
    const float dt = ((float)SPT_UPDATE_INTERVAL_MS) * 0.001f;

    /* 坐标系：yaw=0 -> +Y；左转(+yaw) -> -X，右转(-yaw) -> +X
       因此：
         dx = speed * sin(-yaw) = -speed * sin(yaw)
         dy = speed * cos(yaw)
       其中 yaw 单位为度。*/
    const float rad = yaw_deg * 3.1415926f / 180.0f;
    const float s = (float)speed_cm_s;
    const float dx = (-s) * sinf(rad) * dt;
    const float dy = ( s) * cosf(rad) * dt;

    tracker->x_cm += dx;
    tracker->y_cm += dy;

    /* 计算新网格坐标（就近取整受 SPT_GRID_SIZE_CM 影响） */
    int16_t gx = spt_to_grid(tracker->x_cm);
    int16_t gy = spt_to_grid(tracker->y_cm);

    if (!tracker->has_last) {
        spt_push_point(tracker, gx, gy);
        tracker->last_grid_x = gx;
        tracker->last_grid_y = gy;
        tracker->has_last = 1;
        return;
    }

    /* 与最新点重合则不更新 */
    if (gx == tracker->last_grid_x && gy == tracker->last_grid_y) {
        return;
    }

    /* 若某一轴相同，直接按该轴方向逐格补点（包含终点，不重复起点） */
    if (gx == tracker->last_grid_x || gy == tracker->last_grid_y) {
        int16_t x0 = tracker->last_grid_x;
        int16_t y0 = tracker->last_grid_y;
        int16_t x1 = gx;
        int16_t y1 = gy;

        if (x0 == x1) {
            int step = (y1 > y0) ? 1 : -1;
            for (int16_t y = y0 + step; ; y += step) {
                spt_push_point(tracker, x0, y);
                if (y == y1) break;
            }
        } else { /* y0 == y1 */
            int step = (x1 > x0) ? 1 : -1;
            for (int16_t x = x0 + step; ; x += step) {
                spt_push_point(tracker, x, y0);
                if (x == x1) break;
            }
        }
    } else {
        /* 无单轴连接，用Bresenham补齐：顺序为旧点->新点 */
        spt_connect_line(tracker, tracker->last_grid_x, tracker->last_grid_y, gx, gy);
    }

    tracker->last_grid_x = gx;
    tracker->last_grid_y = gy;
}

const SPT_Point* SPT_GetPath(const SPT_Tracker* tracker, int* out_len) {
    if (!tracker) return 0;
    if (out_len) *out_len = tracker->length;
    return tracker->points;
}

/* ===================== 最短路径算法实现 ===================== */

/* 采样网格点结构 */
typedef struct {
    int16_t x;
    int16_t y;
} SampledPoint;

/* BFS节点结构 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t parent_idx;  /* 父节点索引，-1表示起点 */
} BFSNode;

/* 对网格坐标进行采样（除以因子取整再乘以因子） */
static inline void spt_sample_point(int16_t grid_x, int16_t grid_y, int16_t* out_x, int16_t* out_y) {
    *out_x = (grid_x / SPT_SAMPLE_FACTOR) * SPT_SAMPLE_FACTOR;
    *out_y = (grid_y / SPT_SAMPLE_FACTOR) * SPT_SAMPLE_FACTOR;
}

/* 检查采样点是否已存在于数组中 */
static int spt_sampled_exists(const SampledPoint* arr, int len, int16_t x, int16_t y) {
    for (int i = 0; i < len; i++) {
        if (arr[i].x == x && arr[i].y == y) return 1;
    }
    return 0;
}

/* 检查BFS访问数组中是否已访问过某点 */
static int spt_bfs_visited(const BFSNode* arr, int len, int16_t x, int16_t y) {
    for (int i = 0; i < len; i++) {
        if (arr[i].x == x && arr[i].y == y) return 1;
    }
    return 0;
}

/* BFS核心实现：从(0,0)搜索到(end_x, end_y) */
static int spt_bfs_search(const SampledPoint* walkable, int walkable_count,
                          int16_t end_x, int16_t end_y,
                          BFSNode* visited, int* visited_count) {
    /* 静态队列（环形缓冲区） */
    static BFSNode queue[SPT_BFS_QUEUE_SIZE];
    int queue_head = 0;
    int queue_tail = 0;
    
    /* 起点入队 */
    queue[queue_tail].x = 0;
    queue[queue_tail].y = 0;
    queue[queue_tail].parent_idx = -1;
    queue_tail = (queue_tail + 1) % SPT_BFS_QUEUE_SIZE;
    
    /* 起点加入访问列表 */
    visited[0].x = 0;
    visited[0].y = 0;
    visited[0].parent_idx = -1;
    *visited_count = 1;
    
    /* 四方向：上、下、左、右 */
    const int16_t dx[4] = {0, 0, -SPT_SAMPLE_FACTOR, SPT_SAMPLE_FACTOR};
    const int16_t dy[4] = {-SPT_SAMPLE_FACTOR, SPT_SAMPLE_FACTOR, 0, 0};
    
    /* BFS主循环 */
    while (queue_head != queue_tail) {
        /* 出队 */
        BFSNode current = queue[queue_head];
        queue_head = (queue_head + 1) % SPT_BFS_QUEUE_SIZE;
        
        /* 找到当前节点在visited中的索引 */
        int current_idx = -1;
        for (int i = 0; i < *visited_count; i++) {
            if (visited[i].x == current.x && visited[i].y == current.y) {
                current_idx = i;
                break;
            }
        }
        
        /* 检查是否到达终点 */
        if (current.x == end_x && current.y == end_y) {
            return current_idx;  /* 返回终点在visited中的索引 */
        }
        
        /* 四方向扩展 */
        for (int dir = 0; dir < 4; dir++) {
            int16_t nx = current.x + dx[dir];
            int16_t ny = current.y + dy[dir];
            
            /* 检查是否可通行 */
            if (!spt_sampled_exists(walkable, walkable_count, nx, ny)) continue;
            
            /* 检查是否已访问 */
            if (spt_bfs_visited(visited, *visited_count, nx, ny)) continue;
            
            /* 检查visited数组是否已满 */
            if (*visited_count >= SPT_BFS_VISITED_SIZE) return -1;
            
            /* 加入访问列表 */
            visited[*visited_count].x = nx;
            visited[*visited_count].y = ny;
            visited[*visited_count].parent_idx = current_idx;
            
            /* 入队 */
            int next_tail = (queue_tail + 1) % SPT_BFS_QUEUE_SIZE;
            if (next_tail == queue_head) return -1;  /* 队列满 */
            
            queue[queue_tail].x = nx;
            queue[queue_tail].y = ny;
            queue[queue_tail].parent_idx = current_idx;
            queue_tail = next_tail;
            
            (*visited_count)++;
        }
    }
    
    return -1;  /* 未找到路径 */
}

/* 回溯路径并存入shortest_path */
static void spt_backtrack_path(const BFSNode* visited, int end_idx, SPT_Tracker* shortest_path) {
    /* 临时数组存储反向路径 */
    static SPT_Point temp_path[SPT_MAX_POINTS];
    int temp_len = 0;
    
    /* 从终点回溯到起点 */
    int idx = end_idx;
    while (idx != -1 && temp_len < SPT_MAX_POINTS) {
        temp_path[temp_len].grid_x = visited[idx].x;
        temp_path[temp_len].grid_y = visited[idx].y;
        temp_len++;
        idx = visited[idx].parent_idx;
    }
    
    /* 反转路径并存入shortest_path */
    shortest_path->length = 0;
    for (int i = temp_len - 1; i >= 0 && shortest_path->length < SPT_MAX_POINTS; i--) {
        shortest_path->points[shortest_path->length].grid_x = temp_path[i].grid_x;
        shortest_path->points[shortest_path->length].grid_y = temp_path[i].grid_y;
        shortest_path->length++;
    }
    
    /* 更新shortest_path的其他字段 */
    if (shortest_path->length > 0) {
        int last = shortest_path->length - 1;
        shortest_path->last_grid_x = shortest_path->points[last].grid_x;
        shortest_path->last_grid_y = shortest_path->points[last].grid_y;
        shortest_path->has_last = 1;
    }
}

int SPT_FindShortestPath(const SPT_Tracker* history_path, 
                         SPT_Tracker* shortest_path,
                         int16_t end_x, 
                         int16_t end_y) {
    if (!history_path || !shortest_path) return -1;
    if (history_path->length == 0) return -1;
    
    /* 第一步：采样历史路径并去重 */
    static SampledPoint sampled[SPT_MAX_SAMPLED_POINTS];
    int sampled_count = 0;
    
    for (int i = 0; i < history_path->length && sampled_count < SPT_MAX_SAMPLED_POINTS; i++) {
        int16_t sx, sy;
        spt_sample_point(history_path->points[i].grid_x, 
                        history_path->points[i].grid_y, 
                        &sx, &sy);
        
        /* 去重：只添加不存在的点 */
        if (!spt_sampled_exists(sampled, sampled_count, sx, sy)) {
            sampled[sampled_count].x = sx;
            sampled[sampled_count].y = sy;
            sampled_count++;
        }
    }
    
    if (sampled_count == 0) return -1;
    
    /* 第二步：对终点也进行采样 */
    int16_t sampled_end_x, sampled_end_y;
    spt_sample_point(end_x, end_y, &sampled_end_x, &sampled_end_y);
    
    /* 确保终点在可通行区域内 */
    if (!spt_sampled_exists(sampled, sampled_count, sampled_end_x, sampled_end_y)) {
        /* 终点不可达，尝试以历史路径最新点为终点 */
        if (history_path->length > 0) {
            int last = history_path->length - 1;
            spt_sample_point(history_path->points[last].grid_x,
                           history_path->points[last].grid_y,
                           &sampled_end_x, &sampled_end_y);
            
            /* 如果最新点也不在采样区域（理论上不会发生），返回失败 */
            if (!spt_sampled_exists(sampled, sampled_count, sampled_end_x, sampled_end_y)) {
                return -1;
            }
        } else {
            return -1;
        }
    }
    
    /* 第三步：BFS搜索 */
    static BFSNode visited[SPT_BFS_VISITED_SIZE];
    int visited_count = 0;
    
    int end_idx = spt_bfs_search(sampled, sampled_count, 
                                 sampled_end_x, sampled_end_y,
                                 visited, &visited_count);
    
    if (end_idx < 0) {
        /* 原始终点不可达，尝试历史路径最新点 */
        if (history_path->length > 0) {
            int last = history_path->length - 1;
            int16_t fallback_x, fallback_y;
            spt_sample_point(history_path->points[last].grid_x,
                           history_path->points[last].grid_y,
                           &fallback_x, &fallback_y);
            
            /* 重置访问数组，重新搜索 */
            visited_count = 0;
            end_idx = spt_bfs_search(sampled, sampled_count,
                                    fallback_x, fallback_y,
                                    visited, &visited_count);
            
            if (end_idx < 0) return -1;  /* 仍然失败 */
            
            /* 找到到最新点的路径 */
            spt_backtrack_path(visited, end_idx, shortest_path);
            return 0;  /* 返回0表示找到的是到最新点的路径 */
        }
        return -1;
    }
    
    /* 第四步：回溯路径 */
    spt_backtrack_path(visited, end_idx, shortest_path);
    return 1;  /* 成功找到到目标终点的路径 */
}

