#include "gpmodule.h"
#include "robix4/protocols/app_gp.h"
#include "robix4/protocols/protocol_status.h"

namespace UGV_GP
{

#ifndef USING_ROBIX4
#include <QDebug>
#define MBUG qDebug
#else
#include "rbx4api.h"
#endif
enum GP_STATE
{
	Recover = 0,
	Run     = 1,
	Off     = 2
};

	MAP gp;
	unsigned int myFrameid =1U;		//��ʼΪ0ʱ�����г�ʼ�����˴�Ϊ�˵����ã���shell����ʹ�ã�
	STATUS robixStatus;
	
	double gps[3];
	//���빫��
	 GP_FUNC_INPUT * gpInput  =NULL;
	 GP_INFO * gpOutput  =NULL;
	 GP_LOCAL_DATA * gpEvent  =NULL;

	//@NOTE: ��Ҫʹ��·��ָ����򽫳�ʼ״̬��ΪRecover��������ΪRun����Ҫ����GP����ΪOff
	GP_STATE gpState  = Recover;      //initialized with Recover
	bool triggerRecover  = true;


	bool trySwitch()
	{
		gp.MercatorProjInvCal(gps[0],gps[1],&gps[0],&gps[1]);
		gp.context(gps,gpOutput);	
		return true;
	}

	bool tryRecover()
	{
		//����Զ��ָ�·����
		if(Recover != gpState)
			return false;
		if(true != triggerRecover)
			return false;
		MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "Recover", "N/A");
	//	gp.intit(static_cast<unsigned int>(gpInput->reco_point.recovery),gps);
		
		if(RNDF_ERROR == gp.stat())
				{
					MBUG("\t<tryRecover> GP recover faild at %d.\n", gpInput->reco_point.recovery);
					gpEvent->event = E_FAILURE;//���ʹ����¼�
					return 1;
				}
		
		MBUG("\t<tryRecover> GP has recovered from %d.\n", gpInput->reco_point.recovery);
		//����ָ���λ����1�ŵ㣬����ֱ���л�����·����
		if(0 == gpInput->reco_point.recovery)
		{
			gpEvent->event = E_WAIT_LIGHT;
			MBUG("\t<tryRecover> GP Send Event = E_WAIT_LIGHT, No Recovery Information(start from first)!\n");
		}
		else if(1 == gpInput->reco_point.recovery)
		{
			gpEvent->event = E_WAIT_LIGHT;
			MBUG("\t<tryRecover> GP Send Event = E_WAIT_LIGHT, Specific Recovery Information(first point)!\n");
		}
		else
		{
			gpEvent->event = E_RUN_ROAD;
			MBUG("\t<tryRecover> GP Send Event = E_RUN_ROAD, Specific Recovery Infomation!\n");
		}
		//ִ����Ϻ��л�Run��������
		triggerRecover = false;
		gpState = Run;
		return true;
	}

	int Global_Plan(GP_FUNC_INPUT * pInput, GP_INFO * pData, GP_LOCAL_DATA * pLocal)
	{
		
		/* 0 ȫ�ֹ滮Ԥ���� */
		/* 0.1 ���ü��������� */
		if(myFrameid==1)
			MBUG("WARNING:this shell is for simulate only, no use any more\n");
		++myFrameid;
		pData->header.id = myFrameid;
		MBUG("\n*************************FRAME_ID %d***********************************\n", myFrameid);
		/* 0.2 ���滮��״̬ */
		
		if(RNDF_ERROR == gp.stat())
			{
				MBUG("GP@%d : GP is INVALID(RNDF STATUS ERROR, CHECK FILE task OR RECOVERY POINT)!\n", myFrameid);
				MBUG("**********************************************************************\n");
				return 1;
			}
		
		/* 0.3 Ԥ��ȫ�ֹ滮��Ϣ�����¶�̬���� */
		
		gpInput = pInput;
		gpOutput = pData;
		gpEvent = pLocal;
		memcpy(&pData->state, &pInput->state, sizeof(STATE));
		gps[0] = pInput->state.pos.com_coord.x;
		gps[1] = pInput->state.pos.com_coord.y;
		gps[2] = pInput->state.pos.yaw*(180.0*1e-8)/3.1415926535898;

#ifdef USING_ROBIX4
		rbxReadStatus(&robixStatus);
#else
		if(myFrameid==1)
			robixStatus=S_INVALID;
		else
			robixStatus=S_ROAD_NAV;
#endif
		/* 1 ִ��ȫ�ֹ滮 */
		switch(robixStatus)
		{
		case S_INVALID:
			MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "N/A", "S_INVALID");
			tryRecover();
			MBUG("**********************************************************************\n");
			break;
		case S_WAIT:
			MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "N/A", "S_WAIT");
			tryRecover();
			MBUG("**********************************************************************\n");
			break;
		case S_WAIT_LIGHT:
			MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "Run", "S_WAIT_LIGHT");
			gp.context(gps,gpOutput);
			MBUG("**********************************************************************\n");
			break;
		case S_ROAD_NAV:
			MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "Run", "S_ROAD_NAV");
			trySwitch();
			MBUG("**********************************************************************\n");
			break;
		case S_CROSS_UND:
			MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "Run", "S_CROSS_UND");
			trySwitch();
			MBUG("**********************************************************************\n");
			break;
		case S_STRAIGHT:
			MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "Run", "S_STRAIGHT");
			trySwitch();
			MBUG("**********************************************************************\n");
			break;
		case S_LEFT:
			MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "Run", "S_LEFT");
			trySwitch();
			MBUG("**********************************************************************\n");
			break;
		case S_RIGHT:
			MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "Run", "S_RIGHT");
			trySwitch();
			MBUG("**********************************************************************\n");
			break;
		case S_UTURN:
			MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "Run", "S_UTURN");
			trySwitch();
			MBUG("**********************************************************************\n");
			break;
		case S_PARKING:
			MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "Run", "S_PARKING");
			trySwitch();
			MBUG("**********************************************************************\n");
			break;
		case S_FREE_RUN:
			MBUG("GP@%d : gpState = {%s} (robixStatus = %s)\n", myFrameid, "Run", "S_FREE_RUN");
			trySwitch();
			MBUG("**********************************************************************\n");
			break;
		case S_TASK_OVER:
			MBUG("GP@%d: gpState = {%s} (robixStatus = %s)\n", myFrameid, "Off", "S_TASK_OVER");
			gpState = Off;
			gp.context(gps,gpOutput);
			MBUG("**********************************************************************\n");
			break;
		default:
			break;
		}
		return 0;
	}

};