#include "record.h"
#include "uart2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "simple_path_tracker.h"

uint8_t shortest_count = 0;          // 最短路径长度
uint16_t hist_count = 0;             // 历史路径长度
uint8_t record_num = 2;               // 记录次数
record_t co60_record={"Co60","1173keV","1332keV",106,121,1173,1332,150,50,149,49};
record_t co57_record={"Co57","122keV","136keV",10,14,122,136,65,25,7,4};
record_t Ba133_record={"Ba133","81keV","356keV",7,32,80,356,0,0,0,0};
record_t K40_record={"K40","1460keV","",132,0,1460,0,0,0,0,0};
record_t H3_record={"H3","16keV","",1,0,16,0,0,0,0,0};
record_t C14_record={"C14","50KeV","",4,0,50,0,0,0,0,0};
record_t Sr90_record={"Sr90","180KeV","",16,0,180,0,0,0,0,0};
record_t Pu239_record={"Pu239","5156KeV","",469,0,5156,0,0,0,0,0};
record_t Am241_record={"Am241","5485KeV","",499,0,5485,0,0,0,0,0};
record_t Pu242_record={"Pu242","4902KeV","",446,0,4902,0,0,0,0,0};
record_t Cf252_record={"Cf252","6118KeV","",557,0,6118,0,0,0,0,0};

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

void road_show(SPT_Point *shpoints,SPT_Point *history_points,uint16_t shcount,uint16_t hist_count)
{
    char buffer[100];
    osDelay(250);
    for(int i=0;i<shcount-1;i++)
    {
        if(shpoints[i].grid_x!=shpoints[i+1].grid_x || shpoints[i].grid_y!=shpoints[i+1].grid_y)
        {
            snprintf(buffer, sizeof(buffer), "line %d,%d,%d,%d,%d\xff\xff\xff", (int)(shpoints[i].grid_x*SPT_GRID_SIZE_CM/2)+80, (200-(int)(shpoints[i].grid_y*SPT_GRID_SIZE_CM/2)), (int)(shpoints[i+1].grid_x*SPT_GRID_SIZE_CM/2)+80, (200-(int)(shpoints[i+1].grid_y*SPT_GRID_SIZE_CM/2)), 0);
            u2printf(buffer);
            osDelay(20);
        }
    }
    for(int i=0;i<hist_count-1;i++)
    {
        if(history_points[i].grid_x!=history_points[i+1].grid_x || history_points[i].grid_y!=history_points[i+1].grid_y)
        {
            snprintf(buffer, sizeof(buffer), "line %d,%d,%d,%d,%d\xff\xff\xff", (int)(history_points[i].grid_x*SPT_GRID_SIZE_CM/2)+180, (200-(int)(history_points[i].grid_y*SPT_GRID_SIZE_CM/2)), (int)(history_points[i+1].grid_x*SPT_GRID_SIZE_CM/2)+180, (200-(int)(history_points[i+1].grid_y*SPT_GRID_SIZE_CM/2)), 0);
            u2printf(buffer);
            osDelay(20);
        }
    }
}

void history_cal1(uint8_t count)
{
    if(count==1)
        u2printf("t4.txt+=t14.txt\xff\xff\xff");
    else if(count==2)
        u2printf("t5.txt+=t14.txt\xff\xff\xff");
    else if(count==3)
        u2printf("t11.txt+=t14.txt\xff\xff\xff");
    else if(count==4)
        u2printf("t7.txt+=t14.txt\xff\xff\xff");
    else if(count==5)
        u2printf("t12.txt+=t14.txt\xff\xff\xff");
    else if(count==6)
        u2printf("t13.txt+=t14.txt\xff\xff\xff");
}

void history_cal2(uint8_t count)
{
    if(count==7)
        u2printf("t4.txt+=t14.txt\xff\xff\xff");
    else if(count==8)
        u2printf("t5.txt+=t14.txt\xff\xff\xff");
    else if(count==9)
        u2printf("t11.txt+=t14.txt\xff\xff\xff");
    else if(count==10)
        u2printf("t7.txt+=t14.txt\xff\xff\xff");
    else if(count==11)
        u2printf("t12.txt+=t14.txt\xff\xff\xff");
    else if(count==12)
        u2printf("t13.txt+=t14.txt\xff\xff\xff");
}

void history_show1_record(record_t record,uint8_t count)
{
    char buffer[100];
    if(count>=6)
        count=6;
    for(uint8_t i=1;i<=count;i++)
    {
    u2printf("covx n3.val,t14.txt,0,0\xff\xff\xff");
    history_cal1(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("t14.txt=\"/\"\xff\xff\xff");
    history_cal1(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("covx n4.val,t14.txt,0,0\xff\xff\xff");
    history_cal1(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("t14.txt=\"/\"\xff\xff\xff");
    history_cal1(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("covx n5.val,t14.txt,0,0\xff\xff\xff");
    history_cal1(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("covx n0.val,t14.txt,0,0\xff\xff\xff");
    history_cal1(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("t14.txt=\":\"\xff\xff\xff");
    history_cal1(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("covx n1.val,t14.txt,0,0\xff\xff\xff");
    history_cal1(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("t14.txt=\":\"\xff\xff\xff");
    history_cal1(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("covx n2.val,t14.txt,0,0\xff\xff\xff");
    history_cal1(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    snprintf(buffer, sizeof(buffer), "t14.txt=\"%s Ch:%d Cnts:%d Ch:%d Cnts:%d\"\xff\xff\xff", record.txt, record.x1, record.max1, record.x2, record.max2);
    u2printf(buffer);
    history_cal1(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");
    }
}

void history_show2_record(record_t record,uint8_t count)
{
    char buffer[100];
    if(count>=7)
    {
    for(uint8_t i=7;i<=count;i++)
    {
    u2printf("covx n3.val,t14.txt,0,0\xff\xff\xff");
    history_cal2(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("t14.txt=\"/\"\xff\xff\xff");
    history_cal2(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("covx n4.val,t14.txt,0,0\xff\xff\xff");
    history_cal2(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("t14.txt=\"/\"\xff\xff\xff");
    history_cal2(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("covx n5.val,t14.txt,0,0\xff\xff\xff");
    history_cal2(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("covx n0.val,t14.txt,0,0\xff\xff\xff");
    history_cal2(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("t14.txt=\":\"\xff\xff\xff");
    history_cal2(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("covx n1.val,t14.txt,0,0\xff\xff\xff");
    history_cal2(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("t14.txt=\":\"\xff\xff\xff");
    history_cal2(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    u2printf("covx n2.val,t14.txt,0,0\xff\xff\xff");
    history_cal2(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");

    snprintf(buffer, sizeof(buffer), "t14.txt=\"%s Ch:%d Cnts:%d Ch:%d Cnts:%d\"\xff\xff\xff", record.txt, record.x1, record.max1, record.x2, record.max2);
    u2printf(buffer);
    history_cal2(i);
    u2printf("t14.txt=\"\"\xff\xff\xff");
    }
    }
}