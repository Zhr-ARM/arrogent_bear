#ifndef __RECORD_H__
#define __RECORD_H__

#include "main.h"

typedef struct 
{
    char txt[25];
    char txt2[25];
    char txt3[25];
    uint8_t ch1;
    uint8_t ch2;
    uint16_t x1;
    uint16_t x2;
    uint16_t max1;
    uint16_t mid1;
    uint16_t max2;
    uint16_t mid2;
} record_t;

/**
 * @brief 迷宫路径点结构体 (专为DFS算法设计)
 */
typedef struct {
    int16_t grid_x;         // 网格X坐标 (0-19)
    int16_t grid_y;         // 网格Y坐标 (0-19)
    float x;                // 实际X坐标(cm) - 统一使用x,y命名
    float y;                // 实际Y坐标(cm)
    uint16_t connected[4];  // 相邻点索引 [上,下,左,右] (0xFFFF表示无连接)
    uint8_t connection_count; // 连接数量
    uint32_t visit_time;    // 访问时间戳
} MapPoint_MAP;

extern record_t co60_record;
extern record_t co57_record;
extern MapPoint_MAP Shortest_Road[50];
extern MapPoint_MAP Hist_Road[400];
extern uint8_t shortest_count;          // 最短路径长度
extern uint16_t hist_count;             // 历史路径长度

void record_show(record_t record);
void record_show_char(record_t record);
void record_show_name(record_t record);
void road_show(MapPoint_MAP *shpoints,MapPoint_MAP *history_points,uint16_t shcount,uint16_t hist_count);

#endif