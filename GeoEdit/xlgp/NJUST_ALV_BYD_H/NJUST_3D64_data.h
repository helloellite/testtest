////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C), 2015, �Ͼ�����ѧ�������ѧ�빤��ѧԺ, ���ܿ�ѧ�뼼��ϵ
//  FileName:  NJUST_3D64_data.h
//  Author: ���Ʒ�
//  Date:   2015.6.27
//  Description: 3Dģ������ݶ���
//
////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _NJUST_3D64_DATA_H_
#define _NJUST_3D64_DATA_H_
//typedef unsigned char       BYTE;
#include "NJUST_Global_Def.h"
#include "NJUST_IP_comm.h"

////////////////////////////////////////////////////////////////////////////////////////////
//
//  ��������
//
////////////////////////////////////////////////////////////////////////////////////////////
#define NJUST_3D64_MAX_OBS_POINT_NUM             4             //һ���ϰ�������ö��ٵ��ʾ
#define NJUST_3D64_MAX_OBS_NUM		             150             //�ϰ����������
#define NJUST_3D64_MAX_ROAD_POINT_NUM	         200             //���ߵ�������Ŀ
////////////////////////////////////////////////////////////////////////////////////////////
//
//  64���״�
//
////////////////////////////////////////////////////////////////////////////////////////////
//����Χ
#define NJUST_LIDAR64_VERTICAL_DISTANCE_CM       8000           //64���״�Ĵ�ֱ����8000cm,��ǰ6000cm,����2000cm            
#define NJUST_LIDAR64_HORIZONTAL_DISTANCE_CM     4000           //64���״��ˮƽ����4000cm,����2000cm,����2000cm            
//�ֱ���
#define NJUST_LIDAR64_VERTICAL_RESOLUTION_CM       20           //64���״�Ĵ�ֱ�ֱ���20cm
#define NJUST_LIDAR64_HORIZONTAL_RESOLUTION_CM      5           //64���״��ˮƽ�ֱ���5cm
//��ֱդ����400��,���뱣֤��8�ı���
#define NJUST_LIDAR64_VERTICAL_GRID_NUM          (NJUST_LIDAR64_VERTICAL_DISTANCE_CM/NJUST_LIDAR64_VERTICAL_RESOLUTION_CM)           
//ˮƽդ����800��,���뱣֤��8�ı���
#define NJUST_LIDAR64_HORIZONTAL_GRID_NUM        (NJUST_LIDAR64_HORIZONTAL_DISTANCE_CM/NJUST_LIDAR64_HORIZONTAL_RESOLUTION_CM)
//�洢
#define NJUST_LIDAR64_VERTICAL_GRID_SIZE         NJUST_LIDAR64_VERTICAL_GRID_NUM 
#define NJUST_LIDAR64_HORIZONTAL_GRID_SIZE       (NJUST_LIDAR64_HORIZONTAL_GRID_NUM/8)
////////////////////////////////////////////////////////////////////////////////////////////
//
//  ���Ͷ���
//
////////////////////////////////////////////////////////////////////////////////////////////
//��������  
enum  NJUST_3D64_ROAD_POINT_TYPE
{
	NJUST_3D64_ROAD_POINT_TYPE_LEFT = 0x00, //��߽�
	NJUST_3D64_ROAD_POINT_TYPE_RIGHT,       //�ұ߽�
	NJUST_3D64_ROAD_POINT_TYPE_TOTAL_NUM    //�߽���������
};               

//�¶�����
enum  NJUST_3D64_ROAD_SLOPE_TYPE
{
	NJUST_3D64_ROAD_SLOPE_TYPE_FLAT = 0x00, //ƽ��
	NJUST_3D64_ROAD_SLOPE_TYPE_UP,          //����
	NJUST_3D64_ROAD_SLOPE_TYPE_DOWN,        //����
	NJUST_3D64_ROAD_SLOPE_TYPE_TOTAL_NUM	  //��������Ŀ
};

//3Dģ���еĵ㶨��
struct NJUST_3D64_POINT_2D
{
	int      x_cm;                               //x���굥λ cm
	int      y_cm;                               //y���굥λ cm
};

//�ϰ���
struct NJUST_3D64_OBS_DATA  
{
	unsigned int         OBSID;                                  //�ϰ���ID
	NJUST_3D64_POINT_2D  pPoint[NJUST_3D64_MAX_OBS_POINT_NUM];     //Ŀ���ϰ��������һ���ɼ��㣬����Ϊ��ʱ����ת���������ǵ㡣
	int                  nPoint;                                 //�ϰ�����Ч����
	int                  z_cm;                                   //2D�ϰ���z=0cm
    int                  fbSpeed_cmps;                           //�ϰ����ǰ���ٶ��ϵķ���,��λ:����/��
	int                  lrSpeed_cmps;                           //�ϰ�������ҷ����ϵ��ٶȷ���,��λ:����/��
};
////////////////////////////////////////////////////////////////////////////////////////////
//
//  3D���ں�ģ����ϰ�����
//
////////////////////////////////////////////////////////////////////////////////////////////
typedef struct NJUST_3D64_OBS                              
{
	int                    frameID;                               //֡ID(��0��ʼ)
	NJUST_IP_TIME          synTime;                               //ʱ���
	unsigned int           navID;                                 //��ͼ���ȡʱ����ӽ��ĵ������ݱ��(��0��ʼ)
	NJUST_3D64_OBS_DATA      pObj[NJUST_3D64_MAX_OBS_NUM];            //�ϰ�������
	int                    nObj;                                  //�ϰ�����Ч����
	int                    nSize;                                 //�ýṹ��Ĵ�С
	unsigned char checksum;                                       //����:��������֮��
}NJUST_3D64_OBS_TO_FU;
////////////////////////////////////////////////////////////////////////////////////////////
//
//  3D���ں�ģ��ĵ�·��Ե
//
////////////////////////////////////////////////////////////////////////////////////////////
typedef struct NJUST_3D64_ROAD_BOUNDARY 
{
	int                          frameID;                            //֡ID(��0��ʼ)
	NJUST_IP_TIME                synTime;                            //ʱ���
	unsigned int                 navID;                              //��ͼ���ȡʱ����ӽ��ĵ������ݱ��(��0��ʼ)
	NJUST_3D64_POINT_2D            Point[NJUST_3D64_MAX_ROAD_POINT_NUM]; //���ߵ�����
	int                          nPoint;                             //���ߵ���Ч����
	NJUST_3D64_ROAD_POINT_TYPE     Type[NJUST_3D64_MAX_ROAD_POINT_NUM];  //���ߵ����� ��ߡ��ұߣ����·������һһ��Ӧ
	int                          nSize;                              //�ýṹ��Ĵ�С
	unsigned char                checksum;                           //����:��������֮��
}NJUST_3D64_ROAD_BOUNDARY_TO_FU;
////////////////////////////////////////////////////////////////////////////////////////////
//
//  3D���ں�ģ��ĵ�·�����¶�����
//
////////////////////////////////////////////////////////////////////////////////////////////
typedef struct NJUST_3D64_ROAD_SLOPE
{
	int                                frameID;                  //֡ID(��0��ʼ)
	NJUST_IP_TIME                      synTime;                  //ʱ���
	unsigned int                       navID;                    //��ͼ���ȡʱ����ӽ��ĵ������ݱ��(��0��ʼ)
	NJUST_3D64_ROAD_SLOPE_TYPE    	   type;                     //�¶����ͣ�ƽ�ء����¡�����
	int                                degree;                   //�¶ȣ���λ: ��
	int                                nSize;                    //�ýṹ��Ĵ�С
	unsigned char                      checksum;                 //����:��������֮��
}NJUST_3D64_ROAD_SLOPE_TO_FU;
////////////////////////////////////////////////////////////////////////////////////////////
//
//  3D���ں�ģ�����������
//
////////////////////////////////////////////////////////////////////////////////////////////
typedef struct tagNJUST_3D64_GRID_TO_FU  
{
	int                    frameID;                               //֡ID(��0��ʼ)
	NJUST_IP_TIME          synTime;                               //ʱ���
	unsigned int           navID;                                 //��ͼ���ȡʱ����ӽ��ĵ������ݱ��(��0��ʼ)
	BYTE                   gridMsk[NJUST_LIDAR64_VERTICAL_GRID_SIZE][NJUST_LIDAR64_HORIZONTAL_GRID_SIZE];  //��400*��800: 400*100���ֽ�
}NJUST_3D64_GRID_TO_FU;

#endif
