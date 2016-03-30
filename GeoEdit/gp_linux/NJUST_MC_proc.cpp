////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C), 2015, �Ͼ�����ѧ�������ѧ�빤��ѧԺ, ���ܿ�ѧ�뼼��ϵ
//  FileName:  NJUST_MC_proc.h
//  Author: Ԭ�� 
//  Date:   2015.8.3
//
//  �ο��ĵ��ô���:
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

//��ͷ
//�����İ��� 0x 0824
//״̬�� ��  0x0924
//���� ��  0x0724


#include <stdio.h>
#include <math.h>
#include <string.h>

#include "gp_linux/NJUST_ALV_BYD_H/NJUST_MC_proc.h"

////////////////////////////////////////////////////////////////////////////////////////////////
//
// �����궨��
//
////////////////////////////////////////////////////////////////////////////////////////////////
#define Lat_Origin   32.01602  		// ����ƽ������ϵԭ���γ��
#define Lon_Origin    119.116887   // ����ƽ������ϵԭ��ľ���
#define Re  6378137
#define Rn  6356755
#define deg_rad   0.01745329252e0     // Transfer from angle degree to rad
////////////////////////////////////////////////////////////////////////////////////////////////
//
// ȫ�ֱ���
//
////////////////////////////////////////////////////////////////////////////////////////////////
static NJUST_MC_STATE_INFO  gMCState;
static NJUST_MC_NAV_INFO    gMCNav;
static NJUST_MC_DRIVE_INFO  gMCDrive;
////////////////////////////////////////////////////////////////////////////////////////////////
//
//��������
//
////////////////////////////////////////////////////////////////////////////////////////////////
int NJUST_MC_Decode_State(const void* pIPData, const int nBytes);
int NJUST_MC_Decode_NAV(const void* pIPData, const int nBytes);
int NJUST_MC_Decode_Drive(const void* pIPData, const int nBytes);
////////////////////////////////////////////////////////////////////////////////////////////////
//
// �������
//
////////////////////////////////////////////////////////////////////////////////////////////////
int NJUST_MC_Decode_IP_Data( const void* pIPData, const int nBytes,
                             NJUST_MC_STATE_INFO  **pState, //������״̬����ʱ,ֵΪNULL
							 NJUST_MC_NAV_INFO  **pNav, //�����ǵ�����Ϣʱ,ֵΪNULL
							 NJUST_MC_DRIVE_INFO  **pDrive  //������ִ������ʱ,ֵΪNULL
						   )
{
	int errCode;

	// step.1----------��ʼ��--------------------------------//
	*pState=NULL;
	*pNav=NULL;
	*pDrive=NULL;
	errCode=1;   //����Ч���� 
	//step.2----------���ݽ���-------------------------------//
	if(((char *)pIPData)[0]==0x24&&((char *)pIPData)[1]==0x09){//״̬
		errCode=NJUST_MC_Decode_State(pIPData,nBytes);
	    if (!errCode)
		{
			*pState=&gMCState;
		}
	}else if(((char *)pIPData)[0]==0x24&&((char *)pIPData)[1]==0x08){//����
		errCode=NJUST_MC_Decode_NAV(pIPData,nBytes);
	    if (!errCode)
		{
			*pNav=&gMCNav;
		}
	
	}else if(((char *)pIPData)[0]==0x24&&((char *)pIPData)[1]==0x07){//����
		errCode=NJUST_MC_Decode_Drive(pIPData,nBytes);
	    if (!errCode)
		{
			*pDrive=&gMCDrive;
		}
	}

	//step.3----------����-----------------------------------//
	return errCode;
}

int NJUST_MC_Decode_State(const void* pIPData, const int nBytes)
{   //����������Ϣ:�����ݽ�����gMCState

	int errCode=0; //�޴���
	char *pBuf;
	signed char checksum;
	int i;
	//step.1----------��ʼ��---------------------------------//
	memset(&gMCState,0,sizeof(NJUST_MC_STATE_INFO));


	//step.2----------�����ݽ�����gMCState-------------------//
	pBuf = (char *)pIPData;
	for(i=0,checksum=0;i<nBytes-1;i++)
	{	
		checksum +=pBuf[i];    
	}
	i=sizeof(NJUST_MC_STATE_INFO);
	if((signed char)(pBuf[nBytes-1])!=checksum||(nBytes)!=sizeof(NJUST_MC_STATE_INFO)){//||(nBytes)!=sizeof(NJUST_MC_STATE_INFO)
		errCode=1;//����У��λ��checksum���С���ԣ��д���
	}else{
		memcpy(&gMCState,pBuf,sizeof(NJUST_MC_STATE_INFO));
		if(gMCState.nSize!=sizeof(NJUST_MC_STATE_INFO)){//�ṹ���С�Բ���
			memset(&gMCState,0,sizeof(NJUST_MC_STATE_INFO));
			errCode=1;
		}
	}

	//step.3----------���ش�����-----------------------------//
	return errCode;
}

int NJUST_MC_Decode_NAV(const void* pIPData, const int nBytes)
{   //����״̬����,�����ݽ�����gMCNav
	int errCode=0; //�޴���
	char *pBuf;
	signed char checksum;
	int i;
	//step.1----------��ʼ��---------------------------------//
	memset(&gMCNav,0,sizeof(NJUST_MC_NAV_INFO));
	//step.2----------�����ݽ�����gMCNav---------------------//
	pBuf = (char *)pIPData;
	for(i=0,checksum=0;i<nBytes-1;i++)
	{	
		checksum +=pBuf[i];    
	}
	i=sizeof(NJUST_MC_NAV_INFO);

	//pBuf[nBytes-1]!=0x0a||pBuf[nBytes-2]!=0x0d
	if((signed char)(pBuf[nBytes-1])!=checksum||(nBytes)!=sizeof(NJUST_MC_NAV_INFO)){
		errCode=1;//����У��λ��checksum���С���ԣ��д���
	}else{
		memcpy(&gMCNav,pBuf,sizeof(NJUST_MC_NAV_INFO));
		if(gMCNav.nSize!=sizeof(NJUST_MC_NAV_INFO)){//�ṹ���С�Բ���
			memset(&gMCNav,0,sizeof(NJUST_MC_NAV_INFO));
			errCode=1;
		}
	}



	//step.3----------���ش�����-----------------------------//
	return errCode;

	return errCode;
}

int NJUST_MC_Decode_Drive(const void* pIPData, const int nBytes)
{   //����ִ������,�����ݽ�����gMCDrive
	int errCode=0; //�޴���
	char *pBuf;
	unsigned char checksum;
	int i;
	//step.1----------��ʼ��---------------------------------//
	memset(&gMCDrive,0,sizeof(NJUST_MC_DRIVE_INFO));
	//step.2----------�����ݽ�����gMCDrive-------------------//
	pBuf = (char *)pIPData;
	for(i=0,checksum=0;i<nBytes-1;i++)
	{	
		checksum +=pBuf[i];    
	}

	if((unsigned char)(pBuf[nBytes-1])!=checksum||(nBytes)!=sizeof(NJUST_MC_DRIVE_INFO)){
		errCode=1;//����У��λ��checksum���С���ԣ��д���
	}else{
		memcpy(&gMCDrive,pBuf,sizeof(NJUST_MC_DRIVE_INFO));
		if(gMCDrive.nSize!=sizeof(NJUST_MC_DRIVE_INFO)){//�ṹ���С�Բ���
			memset(&gMCDrive,0,sizeof(NJUST_MC_DRIVE_INFO));
			errCode=1;
		}
	}

	//step.3----------���ش�����-----------------------------//
	return errCode;
}

////////////////////////////////////////////////////////////////////////////////////////////////
//
//  �ṩ��γ������ת����ƽ������
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief �ṩ��γ������ת����ƽ�����꺯��
 * @param Longitude_degree {double} [in] ����ľ��� ��λ ��
 * @param Latitude_degree  {double} [in] �����γ�� ��λ �� 
 * @param EarthRefCoord_x_m{double *} [out] ת�����ƽ������ֵ ����λ������ֵ ��λ ��
 * @param EarthRefCoord_y_m{double *} [out] ת�����ƽ������ֵ ����λ������ֵ ��λ ��
 * @return {int}, �ɹ�����0, ����-1
 */
int Conver_E_N(double Longitude_degree,double Latitude_degree,double *EarthRefCoord_x_m,double *EarthRefCoord_y_m)
{
	
	*EarthRefCoord_y_m = (Latitude_degree - Lat_Origin)*Re*deg_rad;  
	*EarthRefCoord_x_m = (Longitude_degree - Lon_Origin)*Re*deg_rad*cos(Lat_Origin*deg_rad);
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////
//
//  �ṩ����ƽ������ת��������ϵ
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief �ṩ����ƽ������ת��������ϵ����
 * @param in_yaw_rad {double *} [in] ����ת��ʱ�ĺ���� ��λ ����
 * @param TargetEarthCoord_x_m {double} [in] Ŀ����������xֵ(����λ������ֵ) ��λ ��
 * @param TargetEarthCoord_y_m {double} [in] Ŀ����������yֵ(����λ������ֵ) ��λ ��
 * @param CarEarthCoord_x_m {double} [in] ����������xֵ(����λ������ֵ) ��λ ��
 * @param CarEarthCoord_y_m {double} [in] ����������yֵ(����λ������ֵ) ��λ ��
 * @param out_Coord_B_x_cm {double *} [out] ת����ĳ�������xֵ ����������ֵ ��λ ����
 * @param out_Coord_B_y_cm {double *} [out] ת����ĳ�������yֵ ǰ��λ������ֵ ��λ ����						
 * @return {int}, �ɹ�����0, ����-1
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
