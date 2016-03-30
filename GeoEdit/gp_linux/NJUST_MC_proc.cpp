////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C), 2015, 南京理工大学计算机科学与工程学院, 智能科学与技术系
//  FileName:  NJUST_MC_proc.h
//  Author: 袁峻 
//  Date:   2015.8.3
//
//  参考的调用代码:
//
//  int errCode;
//  NJUST_MC_STATE_INFO  *pState;
//  NJUST_MC_NAV_INFO  *pNav;
//  NJUST_MC_DRIVE_INFO  *pDrive;
//
//  errCode=NJUST_MC_Decode_IP_Data(pIPData,nBytes,&pState,&pNav,&pDrive);
//
//  if (pState)
//  {
//      ....
//  }
//
//  if (pNav)
//  {
//      ....
//  }
//
//  if (pDrive)
//  {
//      ....
//  }
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////

//包头
//导航的包是 0x 0824
//状态定 是  0x0924
//驱动 是  0x0724


#include <stdio.h>
#include <math.h>
#include <string.h>

#include "gp_linux/NJUST_ALV_BYD_H/NJUST_MC_proc.h"

////////////////////////////////////////////////////////////////////////////////////////////////
//
// 常量宏定义
//
////////////////////////////////////////////////////////////////////////////////////////////////
#define Lat_Origin   32.01602  		// 地面平面坐标系原点的纬度
#define Lon_Origin    119.116887   // 地面平面坐标系原点的经度
#define Re  6378137
#define Rn  6356755
#define deg_rad   0.01745329252e0     // Transfer from angle degree to rad
////////////////////////////////////////////////////////////////////////////////////////////////
//
// 全局变量
//
////////////////////////////////////////////////////////////////////////////////////////////////
static NJUST_MC_STATE_INFO  gMCState;
static NJUST_MC_NAV_INFO    gMCNav;
static NJUST_MC_DRIVE_INFO  gMCDrive;
////////////////////////////////////////////////////////////////////////////////////////////////
//
//函数申明
//
////////////////////////////////////////////////////////////////////////////////////////////////
int NJUST_MC_Decode_State(const void* pIPData, const int nBytes);
int NJUST_MC_Decode_NAV(const void* pIPData, const int nBytes);
int NJUST_MC_Decode_Drive(const void* pIPData, const int nBytes);
////////////////////////////////////////////////////////////////////////////////////////////////
//
// 解包函数
//
////////////////////////////////////////////////////////////////////////////////////////////////
int NJUST_MC_Decode_IP_Data( const void* pIPData, const int nBytes,
                             NJUST_MC_STATE_INFO  **pState, //当不是状态数据时,值为NULL
							 NJUST_MC_NAV_INFO  **pNav, //当不是导航信息时,值为NULL
							 NJUST_MC_DRIVE_INFO  **pDrive  //当不是执行数据时,值为NULL
						   )
{
	int errCode;

	// step.1----------初始化--------------------------------//
	*pState=NULL;
	*pNav=NULL;
	*pDrive=NULL;
	errCode=1;   //无有效数据 
	//step.2----------数据解析-------------------------------//
	if(((char *)pIPData)[0]==0x24&&((char *)pIPData)[1]==0x09){//状态
		errCode=NJUST_MC_Decode_State(pIPData,nBytes);
	    if (!errCode)
		{
			*pState=&gMCState;
		}
	}else if(((char *)pIPData)[0]==0x24&&((char *)pIPData)[1]==0x08){//导航
		errCode=NJUST_MC_Decode_NAV(pIPData,nBytes);
	    if (!errCode)
		{
			*pNav=&gMCNav;
		}
	
	}else if(((char *)pIPData)[0]==0x24&&((char *)pIPData)[1]==0x07){//驱动
		errCode=NJUST_MC_Decode_Drive(pIPData,nBytes);
	    if (!errCode)
		{
			*pDrive=&gMCDrive;
		}
	}

	//step.3----------返回-----------------------------------//
	return errCode;
}

int NJUST_MC_Decode_State(const void* pIPData, const int nBytes)
{   //解析导航信息:将数据解析到gMCState

	int errCode=0; //无错误
	char *pBuf;
	signed char checksum;
	int i;
	//step.1----------初始化---------------------------------//
	memset(&gMCState,0,sizeof(NJUST_MC_STATE_INFO));


	//step.2----------将数据解析到gMCState-------------------//
	pBuf = (char *)pIPData;
	for(i=0,checksum=0;i<nBytes-1;i++)
	{	
		checksum +=pBuf[i];    
	}
	i=sizeof(NJUST_MC_STATE_INFO);
	if((signed char)(pBuf[nBytes-1])!=checksum||(nBytes)!=sizeof(NJUST_MC_STATE_INFO)){//||(nBytes)!=sizeof(NJUST_MC_STATE_INFO)
		errCode=1;//两个校验位或checksum或大小不对，有错误
	}else{
		memcpy(&gMCState,pBuf,sizeof(NJUST_MC_STATE_INFO));
		if(gMCState.nSize!=sizeof(NJUST_MC_STATE_INFO)){//结构体大小对不上
			memset(&gMCState,0,sizeof(NJUST_MC_STATE_INFO));
			errCode=1;
		}
	}

	//step.3----------返回错误码-----------------------------//
	return errCode;
}

int NJUST_MC_Decode_NAV(const void* pIPData, const int nBytes)
{   //解析状态数据,将数据解析到gMCNav
	int errCode=0; //无错误
	char *pBuf;
	signed char checksum;
	int i;
	//step.1----------初始化---------------------------------//
	memset(&gMCNav,0,sizeof(NJUST_MC_NAV_INFO));
	//step.2----------将数据解析到gMCNav---------------------//
	pBuf = (char *)pIPData;
	for(i=0,checksum=0;i<nBytes-1;i++)
	{	
		checksum +=pBuf[i];    
	}
	i=sizeof(NJUST_MC_NAV_INFO);

	//pBuf[nBytes-1]!=0x0a||pBuf[nBytes-2]!=0x0d
	if((signed char)(pBuf[nBytes-1])!=checksum||(nBytes)!=sizeof(NJUST_MC_NAV_INFO)){
		errCode=1;//两个校验位或checksum或大小不对，有错误
	}else{
		memcpy(&gMCNav,pBuf,sizeof(NJUST_MC_NAV_INFO));
		if(gMCNav.nSize!=sizeof(NJUST_MC_NAV_INFO)){//结构体大小对不上
			memset(&gMCNav,0,sizeof(NJUST_MC_NAV_INFO));
			errCode=1;
		}
	}



	//step.3----------返回错误码-----------------------------//
	return errCode;

	return errCode;
}

int NJUST_MC_Decode_Drive(const void* pIPData, const int nBytes)
{   //解析执行数据,将数据解析到gMCDrive
	int errCode=0; //无错误
	char *pBuf;
	unsigned char checksum;
	int i;
	//step.1----------初始化---------------------------------//
	memset(&gMCDrive,0,sizeof(NJUST_MC_DRIVE_INFO));
	//step.2----------将数据解析到gMCDrive-------------------//
	pBuf = (char *)pIPData;
	for(i=0,checksum=0;i<nBytes-1;i++)
	{	
		checksum +=pBuf[i];    
	}

	if((unsigned char)(pBuf[nBytes-1])!=checksum||(nBytes)!=sizeof(NJUST_MC_DRIVE_INFO)){
		errCode=1;//两个校验位或checksum或大小不对，有错误
	}else{
		memcpy(&gMCDrive,pBuf,sizeof(NJUST_MC_DRIVE_INFO));
		if(gMCDrive.nSize!=sizeof(NJUST_MC_DRIVE_INFO)){//结构体大小对不上
			memset(&gMCDrive,0,sizeof(NJUST_MC_DRIVE_INFO));
			errCode=1;
		}
	}

	//step.3----------返回错误码-----------------------------//
	return errCode;
}

////////////////////////////////////////////////////////////////////////////////////////////////
//
//  提供经纬度坐标转地面平面坐标
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 提供经纬度坐标转地面平面坐标函数
 * @param Longitude_degree {double} [in] 输入的经度 单位 度
 * @param Latitude_degree  {double} [in] 输入的纬度 单位 度 
 * @param EarthRefCoord_x_m{double *} [out] 转换后的平面坐标值 东向位置坐标值 单位 米
 * @param EarthRefCoord_y_m{double *} [out] 转换后的平面坐标值 北向位置坐标值 单位 米
 * @return {int}, 成功返回0, 否则-1
 */
int Conver_E_N(double Longitude_degree,double Latitude_degree,double *EarthRefCoord_x_m,double *EarthRefCoord_y_m)
{
	
	*EarthRefCoord_y_m = (Latitude_degree - Lat_Origin)*Re*deg_rad;  
	*EarthRefCoord_x_m = (Longitude_degree - Lon_Origin)*Re*deg_rad*cos(Lat_Origin*deg_rad);
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////
//
//  提供地面平面坐标转车载坐标系
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 提供地面平面坐标转车载坐标系函数
 * @param in_yaw_rad {double *} [in] 输入转换时的航向角 单位 弧度
 * @param TargetEarthCoord_x_m {double} [in] 目标点地面坐标x值(东向位置坐标值) 单位 米
 * @param TargetEarthCoord_y_m {double} [in] 目标点地面坐标y值(北向位置坐标值) 单位 米
 * @param CarEarthCoord_x_m {double} [in] 车地面坐标x值(东向位置坐标值) 单位 米
 * @param CarEarthCoord_y_m {double} [in] 车地面坐标y值(北向位置坐标值) 单位 米
 * @param out_Coord_B_x_cm {double *} [out] 转换后的车载坐标x值 右向车载坐标值 单位 厘米
 * @param out_Coord_B_y_cm {double *} [out] 转换后的车载坐标y值 前方位置坐标值 单位 厘米						
 * @return {int}, 成功返回0, 否则-1
 */
int Conver_N_B(double in_yaw_rad,
			   double TargetEarthCoord_x_m,
			   double TargetEarthCoord_y_m,
			   double CarEarthCoord_x_m,
			   double CarEarthCoord_y_m,
			   int *out_Coord_B_x_cm,
			   int *out_Coord_B_y_cm)
{

	double s_psi= sin(in_yaw_rad);
	double c_psi= cos(in_yaw_rad);
	
	*out_Coord_B_x_cm= ((-TargetEarthCoord_y_m*s_psi+TargetEarthCoord_x_m*c_psi)-(-CarEarthCoord_y_m*s_psi+CarEarthCoord_x_m*c_psi))*100;
	*out_Coord_B_y_cm= ((TargetEarthCoord_y_m*c_psi+TargetEarthCoord_x_m*s_psi)-(-CarEarthCoord_y_m*c_psi+CarEarthCoord_x_m*s_psi))*100;

	return 0;
}
