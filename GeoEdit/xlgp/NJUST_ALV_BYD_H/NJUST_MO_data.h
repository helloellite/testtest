////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C), 2015, �Ͼ�����ѧ�������ѧ�빤��ѧԺ, ���ܿ�ѧ�뼼��ϵ
//  FileName:  NJUST_MO_data.h
//  Author: ������
//  Date:   2015.6.6
//  Description: �Կ�ģ�������ģ���������״̬
//  Modification: 
//          2015.7.2, ������
//  Declare:
//          NJUST_FROM_MO_COMMAND
//          NJUST_TO_MO_WORKSTAT
//
////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _NJUST_MO_DATA_H_
#define _NJUST_MO_DATA_H_

#include "NJUST_Global_Def.h"
#include "NJUST_IP_comm.h" 

//ϵͳ����
#define  NJUST_MO_MAX_COMMAND_PARA_LEN          32     //��������������
#define  NJUST_MO_WORK_STAT_ERRMSG_MAXLEN       32     //����״̬������Ϣ����󳤶�
#define  NJUST_MO_DEBUGMSG_MAXLEN              128     //������Ϣ����󳤶�

////////////////////////////////////////////////////////////////////////////////////////////////
//
//  ����: �Կط�����ģ��
//
////////////////////////////////////////////////////////////////////////////////////////////////
enum NJUST_MO_COMMAND_TYPE
{
	NJUST_MO_COMMAND_TYPE_NO = 0x00,                  //������
	NJUST_MO_COMMAND_TYPE_COMPUTER_RESTART,           //�������������
	NJUST_MO_COMMAND_TYPE_COMPUTER_SHUTDOWN,          //������ػ�
	NJUST_MO_COMMAND_TYPE_MODULE_RESTART,             //ģ����������
	NJUST_MO_COMMAND_TYPE_MODULE_DEBUG,               //ģ���л�������ģʽ
	NJUST_MO_COMMAND_TYPE_MODULE_RELEASE,             //ģ���л�������ģʽ
    NJUST_MO_COMMAND_TYPE_TOTAL_NUM	                 //�������������
};

enum NJUST_MO_MODULE_ID
{

};

typedef struct tagNJUST_FROM_MO_COMMAND
{   
	int frameID; //������(��0��ʼ)
	NJUST_IP_TIME synTime;  //ϵͳʱ��
    int moduleID;   //ģ��ID
    NJUST_MO_COMMAND_TYPE cmd; //����
    char pPara[NJUST_MO_MAX_COMMAND_PARA_LEN+1]; //�������
	int nPara; //��Ч�������ֽ���
	int nSize; //�ýṹ��Ĵ�С 
}NJUST_FROM_MO_COMMAND;

////////////////////////////////////////////////////////////////////////////////////////////////
//
//  ģ�鹤��״̬: ��ģ�鷢���Կ�
//
////////////////////////////////////////////////////////////////////////////////////////////////
enum NJUST_MO_MODULE_STATUS
{ 
    NJUST_MO_WORK_STAT_INVAILD = 0x00, //ģ���޽�����
    NJUST_MO_WORK_STAT_VAILD,          //ģ�������Ч
    NJUST_MO_WORK_STAT_TOTAL_NUM	  //���״̬������
};
enum NJUST_MO_MODULE_ERRCODE
{ 
    NJUST_MO_ERRCODE_NOERR = 0x00,   //�޴���
    NJUST_MO_ERRCODE_ERROR,          //����
    NJUST_MO_ERRCODE_TOTAL_NUM	    //��������������
};

//��ģ����ԿصĹ���״̬����
typedef struct tagNJUST_TO_MO_WORKSTAT
{
	NJUST_IP_TIME synTime;  //ϵͳʱ��
    int moduleID;   //ģ��ID
	int myselfTimeOutMS; //ģ��ĳ�ʱ,��λ:ms;���ܿ��ڸ�ʱ����û���յ�״̬����,����Ϊ��ģ�鲻����
    NJUST_MO_MODULE_STATUS stat;    // ����״̬
	int PELR; //��ǧ�ֱȱ�ʾ�Ķ�����
	int timeConsumingMS; //��֡�������õ�ʱ��,��λms
	NJUST_MO_MODULE_ERRCODE errCode;     //������
    char pErrMsg[NJUST_MO_WORK_STAT_ERRMSG_MAXLEN+1];   //������Ϣ
}NJUST_TO_MO_WORKSTAT;

#endif

