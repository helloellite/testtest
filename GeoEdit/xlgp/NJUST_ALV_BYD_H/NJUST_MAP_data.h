////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C), 2015, �Ͼ�����ѧ�������ѧ�빤��ѧԺ, ���ܿ�ѧ�뼼��ϵ
//  FileName:  NJUST_MAP_data.h
//  Author: ���Ʒ�
//  Date:   2015.6.6
//  Description: ��ͼ����
//
////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _NJUST_MAP_DATA_H_
#define _NJUST_MAP_DATA_H_
#include <stdio.h>
#include <stdlib.h>
#include "NJUST_Global_Def.h"
#include "NJUST_IP_comm.h"

#define NJUST_MAP_GPS_POINT    20
//��·����
enum  NJUST_MAP_ROAD_TYPE
{
    NJUST_MAP_ROAD_TYPE_NONE =0x00,         //δ֪
	NJUST_MAP_ROAD_TYPE_ASPHALT,            //����·
	NJUST_MAP_ROAD_TYPE_CEMENT,             //ˮ��·
	NJUST_MAP_ROAD_TYPE_DIRT,               //��·
	NJUST_MAP_ROAD_TYPE_COBBLED,            //ʯ��·
	NJUST_MAP_ROAD_TYPE_HIGHWAY,            //���ٹ�·
	NJUST_MAP_ROAD_TYPE_BRIDGE,             //����
	NJUST_MAP_ROAD_TYPE_TUNNEL,             //���
	NJUST_MAP_ROAD_TYPE_CULVERT,            //����
};

 //�е���
enum   NJUST_MAP_LANE_LINE_TYPE
{
    NJUST_MAP_LANE_LINE_TYPE_NONE = 0x00,              //���е���
    NJUST_MAP_LANE_LINE_TYPE_WHITEDOTTEDLINE,          //��ɫ����
    NJUST_MAP_LANE_LINE_TYPE_WHITESOLIDLINE,           //��ɫʵ��
    NJUST_MAP_LANE_LINE_TYPE_YELLOWDOTTEDLINE,         //��ɫ����
    NJUST_MAP_LANE_LINE_TYPE_YELLOWSOLIDLINE,          //��ɫʵ��
};

//��·��߽�
enum   NJUST_MAP_ROAD_BOUNDARY_TYPE
{
    NJUST_MAP_ROAD_BOUNDARY_TYPE_NONE = 0x00,                    //δ֪
    NJUST_MAP_ROAD_BOUNDARY_TYPE_CONVEXBOUNDARY,                 //͹�߽�
    NJUST_MAP_ROAD_BOUNDARY_TYPE_CONCAVEBOUNDARY,                //���߽�
    NJUST_MAP_ROAD_BOUNDARY_TYPE_DANGEROUSBOUNDARY,              //Σ�ձ߽磨�����£�̽�ⲻ���ף�
};

//·������
enum   NJUST_MAP_NODE_TYPE
{
    NJUST_MAP_NODE_TYPE_NONE = 0x00,                    //δ֪
    NJUST_MAP_NODE_TYPE_CROSSROADS ,                    //ʮ��·��
    NJUST_MAP_NODE_TYPE_TNODE,                          //����·��
    NJUST_MAP_NODE_TYPE_YNODE,                          //Y·��
    NJUST_MAP_NODE_TYPE_RIGHTANGLENODE,                 //ֱ��·��
    NJUST_MAP_NODE_TYPE_STRAIGHTLINENODE,               //ֱ��·��
};

enum   NJUST_MAP_NODE_PASS_TYPE
{
    NJUST_MAP_NODE_PASS_TYPE_NONE = 0x00,                //δ֪
    NJUST_MAP_NODE_PASS_TYPE_STRAIGHTLINE,               //ֱ��ͨ��
    NJUST_MAP_NODE_PASS_TYPE_TURNLEFT,                   //��ת��
    NJUST_MAP_NODE_PASS_TYPE_TURNRIGHT,                  //��ת��
    NJUST_MAP_NODE_PASS_TYPE_TURNAROUND,                 //��ͷ
};
//���̵�λ��
enum   NJUST_MAP_TRAFFIC_LIGHTS_POSITION
{
    NJUST_MAP_TRAFFIC_LIGHTS_POSITION_NONE = 0x00,            //δ֪
    NJUST_MAP_TRAFFIC_LIGHTS_POSITION_RIGHTFRONT,             //��ǰ
    NJUST_MAP_TRAFFIC_LIGHTS_POSITION_LEFTRONT,               //��ǰ
    NJUST_MAP_TRAFFIC_LIGHTS_POSITION_FRONT,                  //��ǰ
};

//���̵�����
enum   NJUST_MAP_TRAFFIC_LIGHTS_TYPE
{
    NJUST_MAP_TRAFFIC_LIGHTS_TYPE_NONE = 0x00,         //û�к��̵�
    NJUST_MAP_TRAFFIC_LIGHTS_TYPE_LEFTSTRAIGHTRIGHT,   //��ֱ��
    NJUST_MAP_TRAFFIC_LIGHTS_TYPE_LEFTSTRAIGHT,        //��ֱ
    NJUST_MAP_TRAFFIC_LIGHTS_TYPE_RIGHTSTRAIGHT,       //��ֱ
    NJUST_MAP_TRAFFIC_LIGHTS_TYPE_LEFTRIGHT,           //����
    NJUST_MAP_TRAFFIC_LIGHTS_TYPE_STRAIGHT,            //ֱ

};
typedef struct gpspoint
{
	double longtitude;
	double latitude;
}MAP_GPSPoint;
//��·�ṹ��
struct NJUST_MAP_INFO_ROAD
{
	NJUST_IP_TIME                     synTime;                             //ʱ���
	int                               FrameID;                             //֡��
    int                               roadnum;                             //��·���
	NJUST_MAP_ROAD_TYPE               roadType;                            //��·����
	NJUST_MAP_ROAD_BOUNDARY_TYPE      leftRoadBoundaryType;                //��߽�������
	NJUST_MAP_ROAD_BOUNDARY_TYPE      rightRoadBoundaryType;               //�ұ߽�������
	int                               roadWidth_cm;                         //��·���,��λ��cm
	int                               curbWidth_cm;                         //��·���ӿ��,��λ��cm (0: ����·����)
	NJUST_MAP_LANE_LINE_TYPE          leftLaneLineType;                    //����������е�������
	NJUST_MAP_LANE_LINE_TYPE          centerLaneLineType;                  //�����м��е�������
	NJUST_MAP_LANE_LINE_TYPE          rightLaneLineType;                   //�����ұ����е�������
	int                               nLaneNum;                            //��������
	int                               idealSpeed_kmh;                      //���鳵�� ��λ: km/h
	int                               distToNextNodeM;                    //������һ·�ھ��� ��λ: m
	MAP_GPSPoint                      nextGPSPointQueue[NJUST_MAP_GPS_POINT];//��20��������
	int                               GPSPointQueuelength;                      //�������е���Ч��ĸ���
	int                               nSize;                               //�ýṹ��Ĵ�С
	unsigned char                     checksum;                            //����:��������֮��
};

//����·�ڽṹ��
struct NJUST_MAP_INFO_NODE
{
	NJUST_IP_TIME                      synTime;                  //ʱ���
	int                                FrameID;                  //֡��
    int                                nodenum;                 //�ڵ���
	int                                x_cm;                    //�ڵ����ĵ�������x,��λ:cm
	int                                y_cm;                    //�ڵ����ĵ�������y,��λ:cm
	NJUST_MAP_NODE_TYPE                nodeType;                //·������
	NJUST_MAP_NODE_PASS_TYPE           nodepassType;            //ͨ��·�ڷ�ʽ
	NJUST_MAP_TRAFFIC_LIGHTS_POSITION  trafficLightsPosition;   //���̵�λ��
	NJUST_MAP_TRAFFIC_LIGHTS_TYPE      trafficLightsType;       //���̵�����
	int                                zebraCrossing;           //�Ƿ��а�����
	MAP_GPSPoint                       nextGPSPointQueue[NJUST_MAP_GPS_POINT];//��20��������
	int                                GPSPointQueuelength;                      //�������е���Ч��ĸ���
	int                                nSize;                   //�ýṹ��Ĵ�С
	unsigned char                      checksum;                //����:��������֮��
};

#endif
