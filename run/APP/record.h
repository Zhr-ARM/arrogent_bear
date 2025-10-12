#ifndef __RECORD_H__
#define __RECORD_H__

#include "main.h"
#include "simple_path_tracker.h"
typedef struct 
{
    char txt[25];
    char txt2[25];
    char txt3[25];
    uint16_t ch1;
    uint16_t ch2;
    uint16_t x1;
    uint16_t x2;
    uint16_t max1;
    uint16_t mid1;
    uint16_t max2;
    uint16_t mid2;
} record_t;


extern uint8_t record_num;
extern record_t co60_record;
extern record_t co57_record;
extern record_t Ba133_record;
extern record_t K40_record;
extern record_t H3_record;
extern record_t C14_record;
extern record_t Sr90_record;
extern record_t Pu239_record;
extern record_t Am241_record;
extern record_t Pu242_record;
extern record_t Cf252_record;

/* 全局计数器（在 record.c 中定义） */
extern uint8_t shortest_count;
extern uint16_t hist_count;

void record_show(record_t record);
void record_show_char(record_t record);
void record_show_name(record_t record);
void road_show(SPT_Point *shpoints,SPT_Point *history_points,uint16_t shcount,uint16_t hist_count);
void history_show1_record(record_t record,uint8_t count);
void history_show2_record(record_t record,uint8_t count);

#endif

