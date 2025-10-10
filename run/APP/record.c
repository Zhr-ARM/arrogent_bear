#include "record.h"
#include "uart2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "simple_path_tracker.h"

uint8_t shortest_count = 0;          // 最短路径长度
uint16_t hist_count = 0;             // 历史路径长度

record_t co60_record={"Co60","1173keV","1332keV",106,121,1173,1332,150,50,149,49};
record_t co57_record={"Co57","122keV","136keV",10,14,122,136,65,25,7,4};

uint16_t x_cal(uint16_t x)
{
    if (x < 256)
        return x * 50 / 128;
    else
    {
        printf("%d\r\n",x-256);
        printf("%d\r\n",((x - 256)*50));
        printf("%d\r\n",(((x - 256)*50)/256));
        printf("%d\r\n",((((x - 256)*50)/256) + 100));
        return ((((x - 256)*50)/256) + 100);
    }
}

uint16_t y_cal(uint16_t y)
{
    if(y < 100)
        return ((y * 30)/100);
    else if(y < 200)
        return (y-100)*35/100 + 30;
    else if(y < 300)
        return (y-200)*33/100 + 65;
    else
        return (y-300)*32/100 + 98;
}

void record_show(record_t record)
{
    char buffer[50];
    char show[450]={0};
    uint16_t x1,x2,max1,max2,mid1,mid2;
    x1 = x_cal(record.x1);
    x2 = x_cal(record.x2);
    max1 = y_cal(record.max1);
    mid1 = y_cal(record.mid1);
    max2 = y_cal(record.max2);
    mid2 = y_cal(record.mid2);
    show[x1-1]=max1-(max1-mid1)/5;
    show[x1-2]=max1-3*(max1-mid1)/5;
    show[x1-3]=max1-(max1-mid1)/4;
    show[x1-4]=mid1+(max1-mid1)/3;
    show[x1-5]=mid1+(max1-mid1)/2;
    show[x1-6]=2;
    show[x1]=max1;
    show[x1+1]=max1-(max1-mid1)/6;
    show[x1+2]=max1-2*(max1-mid1)/3;
    show[x1+3]=mid1+(max1-mid1)/2;
    show[x1+4]=mid1+(max1-mid1)/4;
    show[x1+5]=mid1;
    show[x1+6]=1;
    show[x2-1]=max2-(max2-mid2)/7;
    show[x2-2]=max2-2*(max2-mid2)/5;
    show[x2-3]=max2-2*(max2-mid2)/3;
    show[x2-4]=mid2+2*(max2-mid2)/3;
    show[x2-5]=mid2;
    show[x2-6]=3;
    show[x2]=max2-2;
    show[x2+1]=max2-(max2-mid2)/4;
    show[x2+2]=max2-(max2-mid2)/2;
    show[x2+3]=mid2+(max2-mid2)/2;
    show[x2+4]=mid2+(max2-mid2)/5;
    show[x2+5]=mid2+(max2-mid2)/9;
    show[x2+6]=1;
    for(int i=0;i<450;i++)
    {
    snprintf(buffer, sizeof(buffer), "add main.s0.id,0,%d\xff\xff\xff", show[i]);
    u2printf(buffer);
    }
}

void record_show_name(record_t record)
{
    char buffer[100];
    osDelay(250);
    snprintf(buffer, sizeof(buffer), "main.t1.txt=\"%s Ch:%d Cnts:%d Ch:%d Cnts:%d\"\xff\xff\xff", record.txt, record.x1, record.max1, record.x2, record.max2);
    u2printf(buffer);
}

void record_show_char(record_t record)
{
    char buffer[50];
    uint16_t x1,x2,y1,y2;
    if(record.x1<128)
        x1=x_cal(record.x1)-30;
    else
        x1=x_cal(record.x1)-40;
    if(record.x2<128)
        x2=x_cal(record.x2)+10;
    else
        x2=x_cal(record.x2);
    if(record.max1>120)
        y1=record.max1+25;
    else if(record.max1>50)
        y1=220-record.max1+25;
    else
        y1=210-record.max1+25;
    if(record.max2>120)
        y2=record.max2+25;
    else if(record.max2>50)
        y2=220-record.max2+25;
    else
        y2=190-record.max2+25;
    osDelay(250);
    snprintf(buffer, sizeof(buffer), "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\"%s\"\xff\xff\xff", x1, y1, 100, 50, 2, 0, 0, 1, 1, 3, record.txt2);
    u2printf(buffer);
    osDelay(250);
    snprintf(buffer, sizeof(buffer), "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\"%s\"\xff\xff\xff", x2, y2, 100, 50, 2, 0, 0, 1, 1, 3, record.txt3);
    u2printf(buffer);
}

void road_show(MapPoint_MAP *shpoints,MapPoint_MAP *history_points,uint16_t shcount,uint16_t hist_count)
{
    char buffer[100];
    osDelay(250);
    for(int i=0;i<shcount-1;i++)
    {
        snprintf(buffer, sizeof(buffer), "line %d,%d,%d,%d,%d\xff\xff\xff", (int)(shpoints[i].x*48/200)+10, (int)(shpoints[i].y*200/200)+70, (int)(shpoints[i+1].x*48/200)+10, (int)(shpoints[i+1].y*200/200)+70, 65535);
        u2printf(buffer);
        osDelay(10);
    }
    for(int i=0;i<hist_count-1;i++)
    {
        snprintf(buffer, sizeof(buffer), "line %d,%d,%d,%d,%d\xff\xff\xff", (int)(history_points[i].x*48/200)+10, (int)(history_points[i].y*200/200)+70, (int)(history_points[i+1].x*48/200)+10, (int)(history_points[i+1].y*200/200)+70, 65535);
        u2printf(buffer);
        osDelay(10);
    }
}