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

/* ==================== 最短路径（基于历史点采样+BFS） ==================== */

/* 简单哈希：将(grid_x,grid_y)映射到桶，用于快速存在性查询；
   由于资源受限，使用固定大小开放定址表。 */
typedef struct { int16_t x, y; int used; } spt_hash_entry_t;

static unsigned spt_hash_key(int16_t x, int16_t y) {
    /* 32位混合 */
    unsigned ux = (unsigned)((uint16_t)x);
    unsigned uy = (unsigned)((uint16_t)y);
    unsigned h = ux * 2654435761u ^ (uy * 97531u + 0x9e3779b9u);
    return h;
}

static int spt_hash_put(spt_hash_entry_t* table, int cap, int16_t x, int16_t y) {
    unsigned h = spt_hash_key(x, y);
    int i = (int)(h % (unsigned)cap);
    for (int k = 0; k < cap; ++k) {
        int idx = (i + k) % cap;
        if (!table[idx].used) {
            table[idx].x = x; table[idx].y = y; table[idx].used = 1; return 1;
        }
        if (table[idx].x == x && table[idx].y == y) return 0; /* 已存在 */
    }
    return 0; /* 满 */
}

static int spt_hash_has(const spt_hash_entry_t* table, int cap, int16_t x, int16_t y) {
    unsigned h = spt_hash_key(x, y);
    int i = (int)(h % (unsigned)cap);
    for (int k = 0; k < cap; ++k) {
        int idx = (i + k) % cap;
        if (!table[idx].used) return 0; /* 提前失败 */
        if (table[idx].x == x && table[idx].y == y) return 1;
    }
    return 0;
}

/* 线性查找采样数组中的(x,y)下标（规模有限，简单可靠） */
static inline int spt_find_index_linear(const SPT_Point* sampled, int samp_cnt, int16_t x, int16_t y) {
    for (int i = 0; i < samp_cnt; ++i) {
        if (sampled[i].grid_x == x && sampled[i].grid_y == y) return i;
    }
    return -1;
}

void SPT_BuildShortestPathFromHistory(const SPT_Point* history, int history_len,
                                      SPT_Tracker* out_path, float end_x_cm, float end_y_cm) {
    if (!history || history_len <= 0 || !out_path) {
        if (out_path) out_path->length = 0;
        return;
    }

    /* 1) 采样步长：两倍网格 => 以“相隔2格”为采样点 */
    const int STEP = 2; /* 2格步长 */

    /* 将历史点放入哈希，便于判定“中点是否存在”与去重 */
    /* 表容量设为最近的2倍幂或简单放大；为简化，取 4x history_len 上限，且最小256 */
    int hist_cap = history_len * 4;
    if (hist_cap < 256) hist_cap = 256;
    spt_hash_entry_t* hist_set = (spt_hash_entry_t*)0;
    spt_hash_entry_t* samp_set = (spt_hash_entry_t*)0;

    /* 由于不使用动态分配，这里采用静态上限；根据单片机内存可调。
       将默认值从 4096 降低为基于 SPT_MAX_POINTS 的可配置宏以减少 BSS 占用。
       用户可在编译时通过定义 SPT_HASH_MAX 覆盖此值（例如 -DSPT_HASH_MAX=1024）。 */
#ifndef SPT_HASH_MAX
#define SPT_HASH_MAX (SPT_MAX_POINTS * 4)
#endif
    static spt_hash_entry_t s_hist[SPT_HASH_MAX];
    static spt_hash_entry_t s_samp[SPT_HASH_MAX];
    for (int i = 0; i < SPT_HASH_MAX; ++i) { s_hist[i].used = 0; s_samp[i].used = 0; }
    hist_set = s_hist;
    samp_set = s_samp;
    hist_cap = SPT_HASH_MAX;

    /* 填充历史集合 */
    for (int i = 0; i < history_len; ++i) {
        spt_hash_put(hist_set, hist_cap, history[i].grid_x, history[i].grid_y);
    }

    /* 2) 构建采样节点集：只保留满足 (gx % STEP == 0 && gy % STEP == 0) 的点 */
    /* 同时收集到数组，以便后续 BFS 构图 */
    static SPT_Point sampled[SPT_HASH_MAX];
    int samp_cnt = 0;
    for (int i = 0; i < history_len && samp_cnt < SPT_HASH_MAX; ++i) {
        int16_t gx = history[i].grid_x;
        int16_t gy = history[i].grid_y;
        if (((gx % STEP) == 0) && ((gy % STEP) == 0)) {
            if (!spt_hash_has(samp_set, SPT_HASH_MAX, gx, gy)) {
                spt_hash_put(samp_set, SPT_HASH_MAX, gx, gy);
                sampled[samp_cnt].grid_x = gx;
                sampled[samp_cnt].grid_y = gy;
                ++samp_cnt;
            }
        }
    }

    /* 起点、终点网格（终点由 cm -> grid） */
    int16_t sx = 0, sy = 0;
    int16_t ex = spt_to_grid(end_x_cm);
    int16_t ey = spt_to_grid(end_y_cm);

    /* 若起点或终点不在采样集，尝试加入（若其本身在历史集中） */
    if (!spt_hash_has(samp_set, SPT_HASH_MAX, sx, sy) && spt_hash_has(hist_set, hist_cap, sx, sy)) {
        spt_hash_put(samp_set, SPT_HASH_MAX, sx, sy);
        sampled[samp_cnt].grid_x = sx; sampled[samp_cnt].grid_y = sy; ++samp_cnt;
    }
    if (!spt_hash_has(samp_set, SPT_HASH_MAX, ex, ey) && spt_hash_has(hist_set, hist_cap, ex, ey)) {
        /* 若终点不是2格对齐，但在历史里存在，也纳入采样集 */
        spt_hash_put(samp_set, SPT_HASH_MAX, ex, ey);
        sampled[samp_cnt].grid_x = ex; sampled[samp_cnt].grid_y = ey; ++samp_cnt;
    }

    /* 在采样节点上做BFS。邻接规则：
       - 只能相差2格的4邻域(x±2,y)或(x,y±2)
       - 且要求“路径中点”(相差1格的位置)在原始历史集合中（保证连通性来自历史） */

    if (samp_cnt == 0) { out_path->length = 0; return; }

    int start_idx = spt_find_index_linear(sampled, samp_cnt, sx, sy);
    int goal_idx  = spt_find_index_linear(sampled, samp_cnt, ex, ey);
    if (start_idx < 0 || goal_idx < 0) { out_path->length = 0; return; }

    /* BFS 队列与父指针 */
    static int queue[SPT_HASH_MAX];
    static int parent[SPT_HASH_MAX];
    static unsigned char visited[SPT_HASH_MAX];
    int qh = 0, qt = 0;
    for (int i = 0; i < samp_cnt; ++i) { parent[i] = -1; visited[i] = 0; }

    queue[qt++] = start_idx;
    visited[start_idx] = 1;

    /* 邻接方向 */
    const int dir[4][2] = { {2,0}, {-2,0}, {0,2}, {0,-2} };
    int found = 0;

    while (qh < qt && !found) {
        int u = queue[qh++];
        int16_t ux = sampled[u].grid_x;
        int16_t uy = sampled[u].grid_y;
        for (int d = 0; d < 4; ++d) {
            int16_t vx = (int16_t)(ux + dir[d][0]);
            int16_t vy = (int16_t)(uy + dir[d][1]);

            int v = spt_find_index_linear(sampled, samp_cnt, vx, vy);
            if (v < 0) continue;
            if (visited[v]) continue;

            /* 中点必须在历史集合中：保证连边合法 */
            int16_t mx = (int16_t)((ux + vx) / 2);
            int16_t my = (int16_t)((uy + vy) / 2);
            if (!spt_hash_has(hist_set, hist_cap, mx, my)) continue;

            visited[v] = 1;
            parent[v] = u;
            queue[qt++] = v;
            if (v == goal_idx) { found = 1; break; }
        }
    }

    /* 重建路径到 out_path->points（顺序：起点->终点），仅更新 length */
    out_path->length = 0;
    if (!found) return;

    /* 回溯 */
    static int stack_idx[SPT_HASH_MAX];
    int sp = 0;
    for (int cur = goal_idx; cur >= 0; cur = parent[cur]) {
        stack_idx[sp++] = cur;
        if (cur == start_idx) break;
        if (sp >= SPT_HASH_MAX) break;
    }
    /* 逆序输出：start->...->goal */
    for (int i = sp - 1; i >= 0; --i) {
        if (out_path->length >= SPT_MAX_POINTS) break;
        int idx = stack_idx[i];
        out_path->points[out_path->length].grid_x = sampled[idx].grid_x;
        out_path->points[out_path->length].grid_y = sampled[idx].grid_y;
        out_path->length++;
    }
}
