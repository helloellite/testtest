
#pragma once
#include <vector>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <cfloat>
 #include<iomanip>
using namespace std;

#include "gp_linux/gpmodule.h"
#include "gp_linux/robix4/protocols/app_gp.h"
#define _USE_MATH_DEFINES
#define TASK_TURN_ENTER 5000



class LineTool :public MAP			//����GuideLine����
{
public:
	#define TURN_EXTEND 0		//4*1.5=6�ף�ǰ��ӳ�6��
	#define  MAX_INTEGER (DBL_MAX)
	#define  MIN_INTEGER (-DBL_MAX) 
	#define NOISE_ANGLETHRESHOLD 60
	
	//Ԥ������
	LineTool();
	void taskTurnComplete();
	void noiseDelete(GuideLine* line);
	void reduceSample(GuideLine* line,double LINE_FACTOR=-1,double TURN_FACTOR=-1);
	GuideLine* vehiclePartition(GuideLine* line,const int ,double ,int LINE_LEAST_NUM=-1,int TURN_LEAST_NUM=-1);
	//TASK���
	
	bool taskAtrributeComplete();
	
	//�ļ���д
	GuideLine* loadGuideLine(char *filname);		//��filename�м���һ���µ������ߣ����ظ������ߵ��±�
	//void saveTaskXYReverse(char* filename);
	
	
	
	//��ɾ�Ĳ�
	void appendAtLine(double x,double y,double yaw)
	{
		GuideLine* line=pLineArray[pLineArray.size()-1];
		GLPoint p={x,y,yaw};
		line->points.push_back(p);
	}
	void appendLine()
	{
		pLineArray.push_back(new GuideLine);
	}
	void deletePoint(GuideLine* line ,int pointID)
	{
		line->points.erase(line->points.begin()+pointID);
		if(line->lineID.size()!=0)
		{
			line->lineID.clear();
			line->index.clear();
		}
	};


	

	

	vector<GuideLine*>* getLines(){return &pLineArray;};
	vector<GuideLine*>* getNavigator(){return &pNavigatorArray;};
	double* getBound(){return bound;};
	int getLinesCount(){return pLineArray.size();};
	GuideLine* getTask(){return &task;};
	

	
	//���ܺ���
	
	void angleSmooth(double *da,int length);

	double getEquatorRADIUS(){	//ֻ��Ϊ�˱�֤����ͳһ
		return EQUATOR_RADIUS;
	};
};
