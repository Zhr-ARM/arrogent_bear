#ifndef __RECORD_H__
#define __RECORD_H__

#include "main.h"
#include "simple_path_tracker.h"
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


extern record_t co60_record;
extern record_t co57_record;

void record_show(record_t record);
void record_show_char(record_t record);
void record_show_name(record_t record);
void road_show(MapPoint_MAP *shpoints,MapPoint_MAP *history_points,uint16_t shcount,uint16_t hist_count);

#endif